// =============================================================================
// Pixhawk Orientation RMSE Validator — Browser application
// Three.js 3D views, RMSE computation, WebSocket client
// =============================================================================

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

// =============================================================================
// QUATERNION HELPERS
// All quaternions represented as {w, x, y, z}
// =============================================================================

function quatMul(a, b) {
  return {
    w: a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
    x: a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
    y: a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
    z: a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w,
  };
}

function quatConj(q) {
  return { w: q.w, x: -q.x, y: -q.y, z: -q.z };
}

function quatNorm(q) {
  const n = Math.sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
  if (n < 1e-9) return { w: 1, x: 0, y: 0, z: 0 };
  return { w: q.w/n, x: q.x/n, y: q.y/n, z: q.z/n };
}

/** Geodesic angle between two unit quaternions (radians) */
function quatAngle(a, b) {
  const rel = quatMul(quatConj(a), b);
  const rn  = quatNorm(rel);
  const absW = Math.min(1.0, Math.abs(rn.w));
  return 2.0 * Math.acos(absW);
}

// =============================================================================
// ORIENTATION PRESETS
// Stored as unit quaternions {w,x,y,z}.
// correction = quatConj(preset) — applied as post-multiply:
//   q_corrected = quatMul(q_raw, correction)
// =============================================================================

const ORIENT_PRESETS = {
  none:     { w: 1,          x: 0,          y: 0,          z: 0          },
  yaw90:    { w: 0.70710678, x: 0,          y: 0,          z: 0.70710678 },
  yaw180:   { w: 0,          x: 0,          y: 0,          z: 1          },
  yaw270:   { w: 0.70710678, x: 0,          y: 0,          z:-0.70710678 },
  roll180:  { w: 0,          x: 1,          y: 0,          z: 0          },
  pitch180: { w: 0,          x: 0,          y: 1,          z: 0          },
};

function getCorrection(presetKey) {
  const p = ORIENT_PRESETS[presetKey] || ORIENT_PRESETS.none;
  return quatConj(p);
}

// =============================================================================
// SAMPLE BUFFER
// Stores quaternion samples keyed by normalized boot-ms timestamp.
// =============================================================================

class SampleBuffer {
  constructor(maxAge = 2000) {
    this._map     = new Map(); // t_norm -> {w,x,y,z}
    this._maxAge  = maxAge;
    this._t0      = null;   // first received timeBootMs — used to normalize timestamps
    this.latestQuat = null;
    this.latestT    = null;
  }

  add(t, q) {
    // Normalize so each buffer's timestamps start at 0 regardless of board uptime.
    // Without this, two Pixhawks that booted at different times have raw timeBootMs
    // values thousands of ms apart, and drainMatches (which starts with offset=0)
    // can never find matching pairs.
    if (this._t0 === null) this._t0 = t;
    const tNorm = t - this._t0;
    this._map.set(tNorm, q);
    this.latestQuat = q;
    this.latestT    = tNorm;
    this._prune();
  }

  _prune() {
    if (this._map.size < 2) return;
    const cutoff = this.latestT - this._maxAge;
    for (const [k] of this._map) {
      if (k < cutoff) this._map.delete(k);
      else break; // Map insertion order = time order
    }
  }

  /** Return the entry whose timestamp is closest to t within ±windowMs, or null */
  popNearest(t, windowMs) {
    let bestKey  = null;
    let bestDiff = Infinity;
    for (const [k] of this._map) {
      const d = Math.abs(k - t);
      if (d < bestDiff && d <= windowMs) {
        bestDiff = d;
        bestKey  = k;
      }
    }
    if (bestKey === null) return null;
    const q = this._map.get(bestKey);
    this._map.delete(bestKey);
    return { t: bestKey, q };
  }

  oldestTime() {
    const first = this._map.keys().next();
    return first.done ? null : first.value;
  }

  size() { return this._map.size; }
}

// =============================================================================
// RMSE COMPUTATION — drainMatches
//
// Matches samples from buf1 & buf2 by clock-offset-corrected timestamp.
// Accumulates squared geodesic angles, returns updated RMSE and offset.
// =============================================================================

const OFFSET_HISTORY_LEN = 200;

/**
 * @param {SampleBuffer} buf1
 * @param {SampleBuffer} buf2
 * @param {number}       windowMs
 * @param {number[]}     offsetHistory  — rolling deque of (t2_norm - t1_norm)
 * @param {number[]}     angleErrs      — accumulated squared angle errors
 * @param {{w,x,y,z}}   corr1          — correction quaternion for PX1
 * @param {{w,x,y,z}}   corr2          — correction quaternion for PX2
 * @returns {{ matched: number, rmse: number|null, offset: number|null }}
 */
/**
 * Shortest signed angular difference in degrees, handling ±180° wrap.
 */
function angleDiffDeg(a, b) {
  let d = a - b;
  while (d >  180) d -= 360;
  while (d < -180) d += 360;
  return d;
}

/**
 * Given an array of raw errors (degrees), return RMSE and variance of e².
 */
function computeStats(errors) {
  const n = errors.length;
  if (n === 0) return null;
  let sumSq = 0, sumSqSq = 0;
  for (const e of errors) {
    const sq = e * e;
    sumSq   += sq;
    sumSqSq += sq * sq;
  }
  const meanSq  = sumSq / n;
  const rmse    = Math.sqrt(meanSq);                  // degrees
  const varSqErr = (sumSqSq / n) - (meanSq * meanSq); // deg⁴ → display as deg²
  return { rmse, varSqErr, n };
}

function drainMatches(buf1, buf2, windowMs, offsetHistory, angleErrs, corr1, corr2, onPair) {
  let matched = 0;

  // Compute mean clock offset
  let meanOffset = 0;
  if (offsetHistory.length > 0) {
    meanOffset = offsetHistory.reduce((a, b) => a + b, 0) / offsetHistory.length;
  }

  // Try to match samples from buf1 to buf2
  for (const [t1] of Array.from(buf1._map.entries())) {
    const t2Target = t1 + meanOffset;
    const entry2 = buf2.popNearest(t2Target, windowMs);
    if (!entry2) continue;

    // Update offset history
    const observedOffset = entry2.t - t1;
    offsetHistory.push(observedOffset);
    if (offsetHistory.length > OFFSET_HISTORY_LEN) offsetHistory.shift();

    // Apply correction quaternions
    const q1 = quatNorm(quatMul(buf1._map.get(t1), corr1));
    const q2 = quatNorm(quatMul(entry2.q, corr2));

    // Remove t1 from buf1 now that it has been matched
    buf1._map.delete(t1);

    // Geodesic angle error
    const angle = quatAngle(q1, q2);
    angleErrs.push(angle * angle);

    // Keep angleErrs bounded
    if (angleErrs.length > 5000) angleErrs.shift();

    // Recording callback — receives corrected matched quaternion pair
    if (onPair) onPair(q1, q2);

    matched++;
  }

  // Compute RMSE
  let rmse = null;
  if (angleErrs.length > 0) {
    const mean = angleErrs.reduce((a, b) => a + b, 0) / angleErrs.length;
    rmse = Math.sqrt(mean) * (180 / Math.PI);
  }

  const offset = offsetHistory.length > 0
    ? offsetHistory.reduce((a, b) => a + b, 0) / offsetHistory.length
    : null;

  return { matched, rmse, offset };
}

// =============================================================================
// THREE.JS VIEWER
// =============================================================================

/**
 * Create a face-colored BoxGeometry board model.
 * @param {number[]} faceColors  Array of 6 hex colors
 * @returns {{ scene, mesh, camera, renderer, controls, animate }}
 */
function createViewer(canvas, faceColors) {
  const renderer = new THREE.WebGLRenderer({
    canvas,
    antialias: true,
    alpha: false,
  });
  renderer.setPixelRatio(window.devicePixelRatio);
  renderer.setClearColor(0x1c2230, 1);

  const scene  = new THREE.Scene();

  // Camera — Z-up convention so the Z axis arrow points upward on screen
  const camera = new THREE.PerspectiveCamera(45, 1, 0.1, 100);
  camera.up.set(0, 0, 1);
  camera.position.set(4, -3, 2.5);
  camera.lookAt(0, 0, 0);

  // Lighting
  const ambient = new THREE.AmbientLight(0xffffff, 0.55);
  scene.add(ambient);
  const dirLight = new THREE.DirectionalLight(0xffffff, 0.9);
  dirLight.position.set(5, 8, 6);
  scene.add(dirLight);
  const fillLight = new THREE.DirectionalLight(0xffffff, 0.25);
  fillLight.position.set(-4, -3, -4);
  scene.add(fillLight);

  // Box: 3.0 wide × 2.0 deep × 0.7 thick.
  // Z is the thin axis — the ±Z faces (perpendicular to Z) are the widest (3.0×2.0).
  const geometry = new THREE.BoxGeometry(3.0, 2.0, 0.7);

  // One material per face (+X, -X, +Y, -Y, +Z, -Z)
  const materials = faceColors.map(hex =>
    new THREE.MeshPhongMaterial({
      color: hex,
      side:  THREE.DoubleSide,
      shininess: 60,
    })
  );
  const mesh = new THREE.Mesh(geometry, materials);

  // NED Z-flip: wrap mesh in a parent group with scale.z = -1
  const group = new THREE.Group();
  group.scale.z = -1;
  group.add(mesh);
  scene.add(group);

  // Axes helper (small, for orientation reference)
  const axesHelper = new THREE.AxesHelper(2.2);
  scene.add(axesHelper);

  // OrbitControls
  const controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping    = true;
  controls.dampingFactor    = 0.08;
  controls.minDistance      = 1.5;
  controls.maxDistance      = 20;
  controls.target.set(0, 0, 0);

  // Resize observer
  const resizeObserver = new ResizeObserver(() => {
    const w = canvas.clientWidth;
    const h = canvas.clientHeight;
    if (w > 0 && h > 0) {
      renderer.setSize(w, h, false);
      camera.aspect = w / h;
      camera.updateProjectionMatrix();
    }
  });
  resizeObserver.observe(canvas);

  // Initial size
  {
    const w = canvas.clientWidth  || canvas.parentElement.clientWidth  || 400;
    const h = canvas.clientHeight || canvas.parentElement.clientHeight - 60 || 300;
    renderer.setSize(w, h, false);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
  }

  return { scene, mesh, camera, renderer, controls };
}

// =============================================================================
// EULER CONVERSION  (ZYX, same convention as ArduPilot)
// =============================================================================

function quatToEulerDeg(w, x, y, z) {
  const roll  = Math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y)) * (180/Math.PI);
  const sinp  = 2*(w*y - z*x);
  const pitch = Math.asin(Math.max(-1, Math.min(1, sinp))) * (180/Math.PI);
  const yaw   = Math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z)) * (180/Math.PI);
  return { roll, pitch, yaw };
}

// =============================================================================
// LIVE RPY CHARTS  (Chart.js loaded as window.Chart via regular <script> tag)
// =============================================================================

const MAX_CHART_PTS = 500; // ~10 s at 50 Hz

function makeChart(canvasId, label, colorPx1, colorPx2) {
  const ctx = document.getElementById(canvasId).getContext('2d');
  return new window.Chart(ctx, {
    type: 'line',
    data: {
      labels: [],
      datasets: [
        {
          label: 'PX1',
          data: [],
          borderColor: colorPx1,
          backgroundColor: 'transparent',
          borderWidth: 1.5,
          pointRadius: 0,
          tension: 0,
        },
        {
          label: 'PX2',
          data: [],
          borderColor: colorPx2,
          backgroundColor: 'transparent',
          borderWidth: 1.5,
          pointRadius: 0,
          tension: 0,
        },
      ],
    },
    options: {
      animation: false,
      responsive: true,
      maintainAspectRatio: false,
      interaction: { mode: 'index', intersect: false },
      plugins: {
        legend: { labels: { color: '#c9d1d9', boxWidth: 12, font: { size: 11 } } },
        title:  { display: true, text: label, color: '#c9d1d9', font: { size: 13, weight: 'bold' } },
      },
      scales: {
        x: {
          display: false,
          ticks: { color: '#8b949e' },
          grid:  { color: '#21262d' },
        },
        y: {
          ticks: { color: '#8b949e', font: { size: 10 } },
          grid:  { color: '#21262d' },
          title: { display: true, text: 'degrees', color: '#8b949e', font: { size: 10 } },
        },
      },
    },
  });
}

const charts = {
  roll:  makeChart('chart-roll',  'Roll',  '#29B6F6', '#EF5350'),
  pitch: makeChart('chart-pitch', 'Pitch', '#29B6F6', '#EF5350'),
  yaw:   makeChart('chart-yaw',   'Yaw',   '#29B6F6', '#EF5350'),
};

function pushRpy(px, w, x, y, z) {
  const { roll, pitch, yaw } = quatToEulerDeg(w, x, y, z);
  const dsIdx = px === 1 ? 0 : 1;

  for (const [key, val] of [['roll', roll], ['pitch', pitch], ['yaw', yaw]]) {
    const ch = charts[key];
    ch.data.datasets[dsIdx].data.push(val);
    if (ch.data.datasets[dsIdx].data.length > MAX_CHART_PTS)
      ch.data.datasets[dsIdx].data.shift();
    // Sync labels on PX1 (single shared x-axis length)
    if (dsIdx === 0) {
      ch.data.labels.push('');
      if (ch.data.labels.length > MAX_CHART_PTS) ch.data.labels.shift();
    }
  }
}

function updateCharts() {
  charts.roll.update('none');
  charts.pitch.update('none');
  charts.yaw.update('none');
}

// =============================================================================
// APPLICATION STATE
// =============================================================================

const state = {
  ws: null,

  // Sample buffers
  buf1: new SampleBuffer(3000),
  buf2: new SampleBuffer(3000),

  // RMSE accumulation
  offsetHistory: [],
  angleErrs:     [],
  totalMatched:  0,

  // Current correction quaternions
  corr1: getCorrection('none'),
  corr2: getCorrection('none'),

  // Latest quats for display
  latestQ1: { w: 1, x: 0, y: 0, z: 0 },
  latestQ2: { w: 1, x: 0, y: 0, z: 0 },

  // Sync window ms
  windowMs: 50,
};

// =============================================================================
// DOM REFERENCES
// =============================================================================

// =============================================================================
// RECORDING STATE
// =============================================================================

const rec = {
  active:      false,
  rollErrors:  [],   // raw signed angle diffs in degrees for each matched pair
  pitchErrors: [],
  yawErrors:   [],
};

// =============================================================================
// DOM REFERENCES
// =============================================================================

const selPort1   = document.getElementById('sel-port1');
const selPort2   = document.getElementById('sel-port2');
const selBaud    = document.getElementById('sel-baud');
const inpWindow  = document.getElementById('inp-window');
const selOrient1 = document.getElementById('sel-orient1');
const selOrient2 = document.getElementById('sel-orient2');
const btnRefresh = document.getElementById('btn-refresh');
const btnStart   = document.getElementById('btn-start');
const btnStop    = document.getElementById('btn-stop');
const btnClearLog= document.getElementById('btn-clear-log');

const statusPx1  = document.getElementById('status-px1');
const statusPx2  = document.getElementById('status-px2');
const ratePx1    = document.getElementById('rate-px1');
const ratePx2    = document.getElementById('rate-px2');

const metricPairs  = document.getElementById('metric-pairs');
const metricRmse   = document.getElementById('metric-rmse');
const metricOffset = document.getElementById('metric-offset');

const gpsPx1     = document.getElementById('gps-px1');
const gpsPx2     = document.getElementById('gps-px2');
const gpsSatsPx1 = document.getElementById('gps-sats-px1');
const gpsSatsPx2 = document.getElementById('gps-sats-px2');

// GPS no-data timers — if a slot doesn't report within 5 s of PX1 streaming,
// mark it "No device" so the user knows that port has nothing connected.
const gpsTimers = { gps1: null, gps2: null };

function gpsSetNoDevice(slot) {
  const badgeEl = slot === 'gps1' ? gpsPx1 : gpsPx2;
  const satsEl  = slot === 'gps1' ? gpsSatsPx1 : gpsSatsPx2;
  badgeEl.className   = 'status-badge badge-disconnected';
  badgeEl.textContent = 'No device';
  satsEl.textContent  = '— sats';
}

function gpsResetBadge(slot) {
  const badgeEl = slot === 'gps1' ? gpsPx1 : gpsPx2;
  const satsEl  = slot === 'gps1' ? gpsSatsPx1 : gpsSatsPx2;
  badgeEl.className   = 'status-badge badge-disconnected';
  badgeEl.textContent = '—';
  satsEl.textContent  = '— sats';
}

function gpsStartTimers() {
  for (const slot of ['gps1', 'gps2']) {
    if (gpsTimers[slot]) clearTimeout(gpsTimers[slot]);
    gpsTimers[slot] = setTimeout(() => gpsSetNoDevice(slot), 5000);
  }
}

function gpsClearTimer(slot) {
  if (gpsTimers[slot]) { clearTimeout(gpsTimers[slot]); gpsTimers[slot] = null; }
}

function gpsStopTimers() {
  for (const slot of ['gps1', 'gps2']) {
    if (gpsTimers[slot]) { clearTimeout(gpsTimers[slot]); gpsTimers[slot] = null; }
    gpsResetBadge(slot);
  }
}

const logArea    = document.getElementById('log-area');
const quatPx1    = document.getElementById('quat-px1');
const quatPx2    = document.getElementById('quat-px2');

const canvasPx1  = document.getElementById('canvas-px1');
const canvasPx2  = document.getElementById('canvas-px2');

const btnRecord  = document.getElementById('btn-record');
const btnRecStop = document.getElementById('btn-rec-stop');
const recResults = document.getElementById('rec-results');
const recSamples = document.getElementById('rec-samples');

const selPortTeensy       = document.getElementById('sel-port-teensy');
const btnTeensyConnect    = document.getElementById('btn-teensy-connect');
const btnTeensyDisconnect = document.getElementById('btn-teensy-disconnect');
const btnVerify           = document.getElementById('btn-verify');
const verifyLog           = document.getElementById('verify-log');
const teensyStatusBadge   = document.getElementById('teensy-status-badge');
const btnClearVerify      = document.getElementById('btn-clear-verify');

// =============================================================================
// THREE.JS VIEWERS
// PX1 — blue theme, PX2 — red theme
// =============================================================================

const PX1_COLORS = [0x4FC3F7, 0x0277BD, 0xB3E5FC, 0x01579B, 0x29B6F6, 0x0288D1];
const PX2_COLORS = [0xEF9A9A, 0xB71C1C, 0xFFCDD2, 0xC62828, 0xEF5350, 0xD32F2F];

const viewer1 = createViewer(canvasPx1, PX1_COLORS);
const viewer2 = createViewer(canvasPx2, PX2_COLORS);

// =============================================================================
// RENDER LOOP
// =============================================================================

let lastDrainTime = 0;
const DRAIN_INTERVAL_MS = 100; // drain/compute RMSE at ~10 Hz

function renderLoop(now) {
  requestAnimationFrame(renderLoop);

  // Apply latest quaternions to meshes (Hamilton: Three.js uses x,y,z,w order)
  const q1 = state.latestQ1;
  const q2 = state.latestQ2;
  viewer1.mesh.quaternion.set(q1.x, q1.y, q1.z, q1.w);
  viewer2.mesh.quaternion.set(q2.x, q2.y, q2.z, q2.w);

  viewer1.controls.update();
  viewer2.controls.update();

  viewer1.renderer.render(viewer1.scene, viewer1.camera);
  viewer2.renderer.render(viewer2.scene, viewer2.camera);

  // Drain RMSE computation at reduced rate
  if (now - lastDrainTime > DRAIN_INTERVAL_MS) {
    lastDrainTime = now;
    drainAndUpdate();
  }
}
requestAnimationFrame(renderLoop);

// =============================================================================
// DRAIN + UPDATE METRICS
// =============================================================================

function drainAndUpdate() {
  // When recording, collect per-axis errors for each time-matched pair
  const onPair = rec.active ? (q1, q2) => {
    const r1 = quatToEulerDeg(q1.w, q1.x, q1.y, q1.z);
    const r2 = quatToEulerDeg(q2.w, q2.x, q2.y, q2.z);
    rec.rollErrors.push(angleDiffDeg(r1.roll,  r2.roll));
    rec.pitchErrors.push(angleDiffDeg(r1.pitch, r2.pitch));
    rec.yawErrors.push(angleDiffDeg(r1.yaw,   r2.yaw));
  } : null;

  const result = drainMatches(
    state.buf1, state.buf2,
    state.windowMs,
    state.offsetHistory,
    state.angleErrs,
    state.corr1, state.corr2,
    onPair,
  );

  if (result.matched > 0) {
    state.totalMatched += result.matched;
    metricPairs.textContent = state.totalMatched.toLocaleString();
  }

  if (result.rmse !== null) {
    metricRmse.textContent = result.rmse.toFixed(3) + '°';
    const deg = result.rmse;
    metricRmse.style.color =
      deg < 2  ? '#3fb950' :
      deg < 5  ? '#d29922' :
                 '#f85149';
  }

  if (result.offset !== null) {
    metricOffset.textContent = result.offset.toFixed(1) + ' ms';
  }

  // Show live sample count while recording
  if (rec.active) {
    recSamples.textContent = rec.rollErrors.length.toLocaleString();
  }

  updateCharts();
}

// =============================================================================
// QUAT DISPLAY HELPER
// =============================================================================

function fmtQuat(q) {
  const f = v => (v >= 0 ? ' ' : '') + v.toFixed(4);
  return `w=${f(q.w)}  x=${f(q.x)}  y=${f(q.y)}  z=${f(q.z)}`;
}

// =============================================================================
// WEBSOCKET CLIENT
// =============================================================================

function connectWS() {
  const wsUrl = `ws://${location.host}`;
  const ws = new WebSocket(wsUrl);
  state.ws = ws;

  ws.addEventListener('open', () => {
    appendLog('[WS] Connected to server');
    ws.send(JSON.stringify({ type: 'listPorts' }));
  });

  ws.addEventListener('message', (ev) => {
    let msg;
    try { msg = JSON.parse(ev.data); } catch { return; }
    handleServerMessage(msg);
  });

  ws.addEventListener('close', () => {
    appendLog('[WS] Connection closed — reconnecting in 2 s…');
    setTimeout(connectWS, 2000);
  });

  ws.addEventListener('error', () => {
    appendLog('[WS] Connection error');
  });
}

function send(obj) {
  if (state.ws && state.ws.readyState === WebSocket.OPEN) {
    state.ws.send(JSON.stringify(obj));
  }
}

// =============================================================================
// SERVER MESSAGE HANDLER
// =============================================================================

function handleServerMessage(msg) {
  switch (msg.type) {

    case 'ports': {
      populatePorts(msg.ports || []);
      break;
    }

    case 'quat': {
      const q = quatNorm({ w: msg.w, x: msg.x, y: msg.y, z: msg.z });
      const t = msg.t; // normalized boot-ms

      if (msg.px === 'px1') {
        state.buf1.add(t, q);
        state.latestQ1 = q;
        quatPx1.textContent = fmtQuat(q);
        pushRpy(1, q.w, q.x, q.y, q.z);
      } else if (msg.px === 'px2') {
        state.buf2.add(t, q);
        state.latestQ2 = q;
        quatPx2.textContent = fmtQuat(q);
        pushRpy(2, q.w, q.x, q.y, q.z);
      }
      break;
    }

    case 'log': {
      appendLog(msg.msg);
      break;
    }

    case 'status': {
      const el = msg.px === 'px1' ? statusPx1 : statusPx2;
      el.className = 'status-badge';
      switch (msg.state) {
        case 'connected':
          el.classList.add('badge-connected');
          el.textContent = `Connected${msg.sysId ? ` (SYS ${msg.sysId})` : ''}`;
          break;
        case 'streaming':
          el.classList.add('badge-streaming');
          el.textContent = `Streaming${msg.sysId ? ` (SYS ${msg.sysId})` : ''}`;
          if (msg.px === 'px1') { btnVerify.disabled = false; gpsStartTimers(); }
          break;
        case 'error':
          el.classList.add('badge-error');
          el.textContent = 'Error';
          if (msg.px === 'px1') { btnVerify.disabled = true; gpsStopTimers(); }
          break;
        default:
          el.classList.add('badge-disconnected');
          el.textContent = 'Disconnected';
          if (msg.px === 'px1') { btnVerify.disabled = true; gpsStopTimers(); }
      }
      break;
    }

    case 'rate': {
      const el = msg.px === 'px1' ? ratePx1 : ratePx2;
      el.textContent = `${msg.hz.toFixed(1)} Hz`;
      break;
    }

    case 'gps': {
      // Accept both new `slot` field and legacy `px` field for robustness
      const slot    = msg.slot !== undefined ? msg.slot : (msg.px === 'px1' ? 'gps1' : 'gps2');
      gpsClearTimer(slot); // GPS reported — cancel the "No device" timeout for this slot
      const badgeEl = slot === 'gps1' ? gpsPx1 : gpsPx2;
      const satsEl  = slot === 'gps1' ? gpsSatsPx1 : gpsSatsPx2;
      const fix = msg.fixType;
      const GPS_LABEL = ['No GPS','No Fix','2D Fix','3D Fix','DGPS','RTK Float','RTK Fixed'];
      const label = GPS_LABEL[fix] || `Fix ${fix}`;
      badgeEl.className = 'status-badge ' + (
        fix >= 6 ? 'badge-streaming'   :  // RTK Float / RTK Fixed
        fix >= 3 ? 'badge-connected'   :  // 3D Fix / DGPS
        fix >= 1 ? 'badge-warning'     :  // No Fix (device present, searching) or 2D Fix
                   'badge-disconnected'   // No GPS (no device detected on port)
      );
      badgeEl.textContent = label;
      satsEl.textContent  = msg.sats != null ? `${msg.sats} sats` : '— sats';
      break;
    }

    case 'statustext': {
      const sevClass = SEV_CLASS[msg.severity] || 'vlog-info';
      const isPass = /PASS/i.test(msg.text);
      const isFail = /FAIL/i.test(msg.text);
      const cls = isPass ? 'vlog-pass' : isFail ? 'vlog-fail' : sevClass;
      const px = (msg.px || 'px1').toUpperCase();
      appendVerifyLog(`[${px}] ${msg.text}`, cls);
      if (msg.px === 'px1') processStatusTextForVerify(msg.text);
      break;
    }

    case 'teensy': {
      const isPass = /PASS/i.test(msg.line);
      const isFail = /FAIL/i.test(msg.line);
      const cls = isPass ? 'vlog-pass' : isFail ? 'vlog-fail' : 'vlog-teensy';
      appendVerifyLog(`[TEENSY] ${msg.line}`, cls);
      processTeensyLine(msg.line);
      break;
    }

    case 'verifyStatus': {
      switch (msg.state) {
        case 'reading':
          appendVerifyLog('[VERIFY] Reading STEST_ENABLE from PX1…', 'vlog-info');
          break;
        case 'triggered':
          appendVerifyLog('[VERIFY] STEST_ENABLE=1 sent — waiting for Lua…', 'vlog-notice');
          setVerifyStage('start'); // immediate visual: pipeline enters 'Start ACK' state
          // Keep button disabled — re-enabled when pipeline completes or errors out
          break;
        case 'error':
          appendVerifyLog(`[VERIFY] Error: ${msg.msg || 'unknown'}`, 'vlog-error');
          btnVerify.disabled = false;
          break;
      }
      break;
    }

    case 'teensyStatus': {
      teensyStatusBadge.className = 'status-badge';
      switch (msg.state) {
        case 'connected':
          teensyStatusBadge.classList.add('badge-connected');
          teensyStatusBadge.textContent = 'Teensy on';
          btnTeensyConnect.disabled    = true;
          btnTeensyDisconnect.disabled = false;
          break;
        case 'error':
          teensyStatusBadge.classList.add('badge-error');
          teensyStatusBadge.textContent = 'Teensy error';
          btnTeensyConnect.disabled    = false;
          btnTeensyDisconnect.disabled = true;
          break;
        default:
          teensyStatusBadge.classList.add('badge-disconnected');
          teensyStatusBadge.textContent = 'Teensy off';
          btnTeensyConnect.disabled    = false;
          btnTeensyDisconnect.disabled = true;
      }
      break;
    }
  }
}

// =============================================================================
// PORT LIST UI
// =============================================================================

function populatePorts(ports) {
  const saved1 = selPort1.value;
  const saved2 = selPort2.value;
  const savedT = selPortTeensy.value;

  selPort1.innerHTML       = '<option value="">— select —</option>';
  selPort2.innerHTML       = '<option value="">— select —</option>';
  selPortTeensy.innerHTML  = '<option value="">— select —</option>';

  for (const p of ports) {
    const label = p.path + (p.desc ? `  [${p.desc}]` : '');
    const opt1 = new Option(label, p.path);
    const opt2 = new Option(label, p.path);
    const optT = new Option(label, p.path);
    selPort1.appendChild(opt1);
    selPort2.appendChild(opt2);
    selPortTeensy.appendChild(optT);
  }

  if (saved1) selPort1.value = saved1;
  if (saved2) selPort2.value = saved2;
  if (savedT) selPortTeensy.value = savedT;

  appendLog(`[UI] Found ${ports.length} serial port(s)`);
}

// =============================================================================
// LOG AREAS
// =============================================================================

const MAX_LOG_LINES = 500;

function appendLog(text) {
  const span = document.createElement('span');
  span.className = 'log-line';
  span.textContent = text;
  logArea.appendChild(span);
  while (logArea.childElementCount > MAX_LOG_LINES) logArea.removeChild(logArea.firstChild);
  logArea.scrollTop = logArea.scrollHeight;
}

// Severity → CSS class mapping (ArduPilot MAV_SEVERITY levels)
const SEV_CLASS = ['vlog-error','vlog-error','vlog-error','vlog-error','vlog-warning','vlog-notice','vlog-info','vlog-info'];

function appendVerifyLog(text, cssClass) {
  const span = document.createElement('span');
  span.className = 'log-line ' + (cssClass || 'vlog-info');
  span.textContent = text;
  verifyLog.appendChild(span);
  while (verifyLog.childElementCount > MAX_LOG_LINES) verifyLog.removeChild(verifyLog.firstChild);
  verifyLog.scrollTop = verifyLog.scrollHeight;
}

// =============================================================================
// VERIFICATION UI STATE + HELPERS
// =============================================================================

const VFY_STAGES = ['start', 'preamble', 'stepping', 'done', 'check', 'complete'];

const verifyState = {
  activeStage: null,
  stepsCompleted: 0,
};

function vfyStageEl(name) {
  return document.querySelector(`.vfy-stage[data-stage="${name}"]`);
}

function resetVerifyUI() {
  for (const name of VFY_STAGES) {
    const el = vfyStageEl(name);
    if (el) el.setAttribute('data-status', 'idle');
  }
  for (const id of ['vfy-ack-t1-start','vfy-ack-t2-start','vfy-ack-t1-done','vfy-ack-t2-done']) {
    const el = document.getElementById(id);
    if (!el) continue;
    el.removeAttribute('data-ack');
    el.textContent = id.includes('t1') ? 'T1' : 'T2';
  }
  for (let i = 0; i < 5; i++) {
    const el = document.getElementById(`vfy-step-${i}`);
    if (el) el.removeAttribute('data-step');
  }
  const resEl = document.getElementById('vfy-results');
  if (resEl) resEl.hidden = true;
  for (const id of ['vfy-badge-t1','vfy-badge-t2']) {
    const el = document.getElementById(id);
    if (!el) continue;
    el.className = 'vfy-result-badge vfy-badge-idle';
    el.textContent = '—';
  }
  const ct = document.getElementById('vfy-complete-text');
  if (ct) ct.textContent = '';
  verifyState.activeStage = null;
  verifyState.stepsCompleted = 0;
}

function setVerifyStage(name) {
  const newIdx = VFY_STAGES.indexOf(name);
  // Mark every stage before this one as pass (handles skipped stages e.g. preamble)
  for (let i = 0; i < newIdx; i++) {
    const el = vfyStageEl(VFY_STAGES[i]);
    if (el) {
      const s = el.getAttribute('data-status');
      if (s === 'idle' || s === 'active') el.setAttribute('data-status', 'pass');
    }
  }
  const el = vfyStageEl(name);
  if (el) el.setAttribute('data-status', 'active');
  verifyState.activeStage = name;
}

function setVerifyStageResult(name, result) {
  const el = vfyStageEl(name);
  if (el) el.setAttribute('data-status', result);
}

function setAckBadge(id, ok) {
  const el = document.getElementById(id);
  if (!el) return;
  el.setAttribute('data-ack', ok ? 'pass' : 'fail');
  const label = id.includes('t1') ? 'T1' : 'T2';
  el.textContent = label + (ok ? ' ✓' : ' ✗');
}

function setStepState(idx, state) {
  const el = document.getElementById(`vfy-step-${idx}`);
  if (el) el.setAttribute('data-step', state);
}

function setVerifyResult(telemNum, text, passed) {
  const el = document.getElementById(telemNum === 1 ? 'vfy-badge-t1' : 'vfy-badge-t2');
  if (!el) return;
  el.className = 'vfy-result-badge ' + (passed ? 'vfy-badge-pass' : 'vfy-badge-fail');
  el.textContent = text;
  const resEl = document.getElementById('vfy-results');
  if (resEl) resEl.hidden = false;

  // Auto-complete pipeline when both Telem results have arrived
  const t1el = document.getElementById('vfy-badge-t1');
  const t2el = document.getElementById('vfy-badge-t2');
  if (t1el && t2el && t1el.textContent !== '—' && t2el.textContent !== '—') {
    const allPass = t1el.classList.contains('vfy-badge-pass') &&
                    t2el.classList.contains('vfy-badge-pass');
    const finalStatus = allPass ? 'pass' : 'fail';
    setVerifyStageResult('check', finalStatus);
    const compEl = vfyStageEl('complete');
    if (compEl) compEl.setAttribute('data-status', finalStatus);
    verifyState.activeStage = 'complete';
    const ct = document.getElementById('vfy-complete-text');
    if (ct) ct.textContent = allPass ? '✓ done' : '✗ issues';
    btnVerify.disabled = false; // re-enable now that the test is fully complete
  }
}

function processStatusTextForVerify(text) {
  const t = text.trim();

  if (/start\s+sent/i.test(t)) {
    setVerifyStage('start');
    return;
  }

  const startAck = t.match(/telem(\d)\s+start\s*:\s*(pass|fail)/i);
  if (startAck) {
    setAckBadge(parseInt(startAck[1]) === 1 ? 'vfy-ack-t1-start' : 'vfy-ack-t2-start',
                startAck[2].toUpperCase() === 'PASS');
    return;
  }

  if (/preamble/i.test(t)) {
    setVerifyStage('preamble');
    return;
  }

  const stepM = t.match(/step\s+(\d)\/5/i);
  if (stepM) {
    const n = parseInt(stepM[1]); // 1-5
    setVerifyStage('stepping');
    for (let i = 0; i < n - 1; i++) setStepState(i, 'done');
    setStepState(n - 1, 'active');
    return;
  }

  if (/done\s+sent/i.test(t)) {
    for (let i = 0; i < 5; i++) setStepState(i, 'done');
    setVerifyStage('done');
    return;
  }

  const doneAck = t.match(/telem(\d)\s+done\s*:\s*(pass|fail)/i);
  if (doneAck) {
    setAckBadge(parseInt(doneAck[1]) === 1 ? 'vfy-ack-t1-done' : 'vfy-ack-t2-done',
                doneAck[2].toUpperCase() === 'PASS');
    return;
  }

  if (/check\s+sent/i.test(t)) {
    setVerifyStage('check');
    return;
  }

  const verM = t.match(/telem(\d)\s+verify[^:]*:\s*.*?(pass|fail)\s*(\d+\/\d+)?/i);
  if (verM) {
    const n   = parseInt(verM[1]);
    const ok  = verM[2].toUpperCase() === 'PASS';
    const cnt = verM[3] ? ' ' + verM[3] : '';
    setVerifyResult(n, (ok ? 'PASS' : 'FAIL') + cnt, ok);
    return;
  }

  if (/complete/i.test(t)) {
    setVerifyStage('complete');
    setVerifyStageResult('complete', 'pass');
    const ct = document.getElementById('vfy-complete-text');
    if (ct) ct.textContent = '✓ done';
    return;
  }
}

// Parses Teensy USB serial lines and drives the pipeline.
// Teensy output format (all lines prefixed with #):
//   # capture: START -> capturing
//   # cmd START on TELEM1 / TELEM2
//   # sample step N expect XXXX us: ...  (N = 0..4)
//   # cmd DONE on TELEM1 / TELEM2
//   # cmd CHECK on TELEM1 -> TELEM1 PASS/FAIL N/16 ch:...
function processTeensyLine(line) {
  const t = line.trim();

  // "# capture: START -> capturing"
  if (/capture.*start/i.test(t)) {
    setVerifyStage('start');
    return;
  }

  // "# cmd START on TELEM1" / "# cmd START on TELEM2"
  const startCmd = t.match(/cmd\s+START\s+on\s+TELEM(\d)/i);
  if (startCmd) {
    setAckBadge(parseInt(startCmd[1]) === 1 ? 'vfy-ack-t1-start' : 'vfy-ack-t2-start', true);
    return;
  }

  // "# sample step N expect XXXX us: ..."  (N is 0-indexed, maps to vfy-step-0..4)
  const stepM = t.match(/sample\s+step\s+(\d+)/i);
  if (stepM) {
    const n = parseInt(stepM[1]);
    setVerifyStage('stepping');
    for (let i = 0; i < n; i++) setStepState(i, 'done');
    setStepState(n, 'active');
    return;
  }

  // "# cmd DONE on TELEM1" / "# cmd DONE on TELEM2"
  const doneCmd = t.match(/cmd\s+DONE\s+on\s+TELEM(\d)/i);
  if (doneCmd) {
    const n = parseInt(doneCmd[1]);
    if (n === 1) {
      for (let i = 0; i < 5; i++) setStepState(i, 'done');
      setVerifyStage('done');
    }
    setAckBadge(n === 1 ? 'vfy-ack-t1-done' : 'vfy-ack-t2-done', true);
    return;
  }

  // "# cmd CHECK on TELEM1 -> TELEM1 FAIL 14/16 ch:28,31,"
  const checkM = t.match(/cmd\s+CHECK\s+on\s+TELEM(\d)\s*->\s*TELEM\d\s+(PASS|FAIL)\s+(\d+\/\d+)/i);
  if (checkM) {
    const n   = parseInt(checkM[1]);
    const ok  = checkM[2].toUpperCase() === 'PASS';
    const cnt = checkM[3];
    if (n === 1) setVerifyStage('check');
    setVerifyResult(n, (ok ? 'PASS' : 'FAIL') + ' ' + cnt, ok);
    return;
  }
}

// =============================================================================
// BUTTON HANDLERS
// =============================================================================

btnRefresh.addEventListener('click', () => {
  send({ type: 'listPorts' });
  appendLog('[UI] Refreshing port list…');
});

btnStart.addEventListener('click', () => {
  const port1 = selPort1.value;
  const port2 = selPort2.value;

  if (!port1 || !port2) {
    appendLog('[UI] Please select both ports before starting');
    return;
  }
  if (port1 === port2) {
    appendLog('[UI] Port 1 and Port 2 must be different');
    return;
  }

  const baud   = parseInt(selBaud.value, 10);
  const window = parseInt(inpWindow.value, 10) || 50;

  state.windowMs    = window;
  state.buf1        = new SampleBuffer(3000);
  state.buf2        = new SampleBuffer(3000);
  state.offsetHistory.length = 0;
  state.angleErrs.length     = 0;
  state.totalMatched         = 0;
  metricPairs.textContent    = '0';
  metricRmse.textContent     = '—°';
  metricOffset.textContent   = '— ms';

  btnStart.disabled = true;
  btnStop.disabled  = false;

  send({ type: 'start', port1, port2, baud, window });
  appendLog(`[UI] Starting — PX1: ${port1}, PX2: ${port2}, ${baud} baud, window: ${window} ms`);
});

btnStop.addEventListener('click', () => {
  send({ type: 'stop' });
  btnStart.disabled = false;
  btnStop.disabled  = true;
  appendLog('[UI] Stopping…');
});

btnClearLog.addEventListener('click', () => {
  logArea.innerHTML = '';
});

// Orientation correction dropdowns
selOrient1.addEventListener('change', () => {
  state.corr1 = getCorrection(selOrient1.value);
  appendLog(`[UI] PX1 correction set to: ${selOrient1.value}`);
});
selOrient2.addEventListener('change', () => {
  state.corr2 = getCorrection(selOrient2.value);
  appendLog(`[UI] PX2 correction set to: ${selOrient2.value}`);
});

inpWindow.addEventListener('change', () => {
  state.windowMs = parseInt(inpWindow.value, 10) || 50;
});

// ── Teensy ───────────────────────────────────────────────────────────────────

btnTeensyConnect.addEventListener('click', () => {
  const port = selPortTeensy.value;
  if (!port) { appendLog('[UI] Select a Teensy port first'); return; }
  send({ type: 'teensyConnect', port, baud: 115200 });
  appendLog(`[UI] Connecting Teensy on ${port}…`);
});

btnTeensyDisconnect.addEventListener('click', () => {
  send({ type: 'teensyDisconnect' });
  appendLog('[UI] Disconnecting Teensy…');
});

// ── Verify ───────────────────────────────────────────────────────────────────

btnVerify.addEventListener('click', () => {
  resetVerifyUI();
  verifyLog.innerHTML = '';
  btnVerify.disabled = true;
  send({ type: 'triggerVerify' });
  appendVerifyLog('─── Step+Verify started ───', 'vlog-notice');
  appendLog('[UI] Sent PARAM_REQUEST_READ STEST_ENABLE → PX1');
  // Safety: re-enable after 15 s in case pipeline never reaches Complete
  setTimeout(() => { btnVerify.disabled = false; }, 15000);
});

btnClearVerify.addEventListener('click', () => {
  verifyLog.innerHTML = '';
});

// ── Recording ────────────────────────────────────────────────────────────────

btnRecord.addEventListener('click', () => {
  rec.active = true;
  rec.rollErrors.length  = 0;
  rec.pitchErrors.length = 0;
  rec.yawErrors.length   = 0;

  btnRecord.disabled  = true;
  btnRecStop.disabled = false;
  recResults.style.display = 'none';
  recSamples.textContent = '0';
  appendLog('[REC] Recording started…');
});

btnRecStop.addEventListener('click', () => {
  rec.active = false;
  btnRecord.disabled  = false;
  btnRecStop.disabled = true;

  const n = rec.rollErrors.length;
  appendLog(`[REC] Recording stopped — ${n} matched samples`);

  if (n === 0) {
    appendLog('[REC] No matched samples — check connection and sync window');
    return;
  }

  const roll  = computeStats(rec.rollErrors);
  const pitch = computeStats(rec.pitchErrors);
  const yaw   = computeStats(rec.yawErrors);

  // Populate results table
  const rows = { roll, pitch, yaw };
  for (const axis of ['roll', 'pitch', 'yaw']) {
    const s = rows[axis];
    document.getElementById(`rec-rmse-${axis}`).textContent  = s.rmse.toFixed(4) + '°';
    document.getElementById(`rec-var-${axis}`).textContent   =
      s.varSqErr.toFixed(4) + ' deg⁴';
  }
  document.getElementById('rec-n').textContent = n.toLocaleString();
  recResults.style.display = 'block';

  appendLog(`[REC] Roll  — RMSE: ${roll.rmse.toFixed(3)}°   Var(e²): ${roll.varSqErr.toFixed(3)} deg⁴`);
  appendLog(`[REC] Pitch — RMSE: ${pitch.rmse.toFixed(3)}°   Var(e²): ${pitch.varSqErr.toFixed(3)} deg⁴`);
  appendLog(`[REC] Yaw   — RMSE: ${yaw.rmse.toFixed(3)}°   Var(e²): ${yaw.varSqErr.toFixed(3)} deg⁴`);
});

// =============================================================================
// KICK-OFF
// =============================================================================

connectWS();
appendLog('[UI] Pixhawk Orientation RMSE Validator loaded');
appendLog('[UI] Click "Refresh" to discover serial ports, then select and Start');
