'use strict';

// ---------------------------------------------------------------------------
// Pixhawk Orientation RMSE Validator — Node.js server
// Serves static files, opens two serial ports, runs MAVLink parser,
// streams quaternion data to browser via WebSocket
// ---------------------------------------------------------------------------

const http        = require('http');
const fs          = require('fs');
const path        = require('path');
const { WebSocketServer } = require('ws');
const { SerialPort }      = require('serialport');
const { MAVLinkParser, buildSetMessageInterval, buildRequestDataStream, buildParamSet, buildParamRequestRead, MSG_GPS_RAW_INT, MSG_GPS2_RAW } = require('./mavlink.js');

const PORT        = 3000;
const PUBLIC_DIR  = path.join(__dirname, 'public');
const STREAM_HZ   = 50;
const INTERVAL_US = Math.round(1e6 / STREAM_HZ); // 20 000 µs

// ---------------------------------------------------------------------------
// MIME types
// ---------------------------------------------------------------------------
const MIME = {
  '.html': 'text/html; charset=utf-8',
  '.js':   'application/javascript; charset=utf-8',
  '.css':  'text/css; charset=utf-8',
  '.ico':  'image/x-icon',
  '.png':  'image/png',
  '.svg':  'image/svg+xml',
};

// ---------------------------------------------------------------------------
// HTTP server — serves public/
// ---------------------------------------------------------------------------
const server = http.createServer((req, res) => {
  let urlPath = req.url.split('?')[0];
  if (urlPath === '/') urlPath = '/index.html';
  const filePath = path.join(PUBLIC_DIR, urlPath);

  // Security: prevent directory traversal
  if (!filePath.startsWith(PUBLIC_DIR)) {
    res.writeHead(403);
    res.end('Forbidden');
    return;
  }

  fs.readFile(filePath, (err, data) => {
    if (err) {
      res.writeHead(404, { 'Content-Type': 'text/plain' });
      res.end('Not found');
      return;
    }
    const ext  = path.extname(filePath).toLowerCase();
    const mime = MIME[ext] || 'application/octet-stream';
    res.writeHead(200, { 'Content-Type': mime });
    res.end(data);
  });
});

// ---------------------------------------------------------------------------
// WebSocket server
// ---------------------------------------------------------------------------
const wss = new WebSocketServer({ server });

// Active serial ports & state
let px1Port = null;
let px2Port = null;
let px1Parser = null;
let px2Parser = null;
let isRunning = false;

// Teensy USB serial port (independent of PX session)
let teensyPort = null;

// Rate tracking
const rateCounters = { px1: 0, px2: 0 };
let rateInterval = null;
let rateLastTime = Date.now();

// System IDs discovered during streaming — kept at module scope so setParam can use them
const activeSysIds = { px1: 1, px2: 1 };

// Pending two-step verify operations: px -> { timeout }
const pendingVerifies = {};

// Sequence counters for outgoing MAVLink frames
const seqCounters = { px1: 0, px2: 0 };

function nextSeq(px) {
  seqCounters[px] = (seqCounters[px] + 1) & 0xFF;
  return seqCounters[px];
}

// ---------------------------------------------------------------------------
// Broadcast to all connected WebSocket clients
// ---------------------------------------------------------------------------
function broadcast(obj) {
  const msg = JSON.stringify(obj);
  for (const ws of wss.clients) {
    if (ws.readyState === ws.OPEN) {
      ws.send(msg);
    }
  }
}

function log(msg) {
  const ts = new Date().toISOString().replace('T', ' ').slice(0, 23);
  broadcast({ type: 'log', msg: `[${ts}] ${msg}` });
  console.log(`[${ts}] ${msg}`);
}

// ---------------------------------------------------------------------------
// List available serial ports and send to client
// ---------------------------------------------------------------------------
async function listAndSendPorts(ws) {
  try {
    const ports = await SerialPort.list();
    const mapped = ports.map(p => ({
      path: p.path,
      desc: [p.manufacturer, p.friendlyName, p.pnpId]
             .filter(Boolean)
             .join(' | ') || p.path,
    }));
    const msg = JSON.stringify({ type: 'ports', ports: mapped });
    if (ws.readyState === ws.OPEN) ws.send(msg);
  } catch (e) {
    log(`Port list error: ${e.message}`);
  }
}

// ---------------------------------------------------------------------------
// Request attitude quaternion from a board
// ---------------------------------------------------------------------------
function requestAttitude(port, px, sysId) {
  const targetSys  = sysId || 1;
  const targetComp = 1;

  // Primary: SET_MESSAGE_INTERVAL for ATTITUDE_QUATERNION (msg 31)
  try {
    const frame = buildSetMessageInterval(
      nextSeq(px), 255, 190, targetSys, targetComp, 31, INTERVAL_US
    );
    port.write(frame, err => {
      if (err) log(`${px.toUpperCase()} SET_MESSAGE_INTERVAL write error: ${err.message}`);
    });
  } catch (e) {
    log(`${px.toUpperCase()} SET_MESSAGE_INTERVAL build error: ${e.message}`);
  }

  // GPS_RAW_INT (msg 24) at 2 Hz — primary GPS
  try {
    const frame = buildSetMessageInterval(
      nextSeq(px), 255, 190, targetSys, targetComp, MSG_GPS_RAW_INT, 500000
    );
    port.write(frame, err => {
      if (err) log(`${px.toUpperCase()} GPS interval write error: ${err.message}`);
    });
  } catch (e) {
    log(`${px.toUpperCase()} GPS interval build error: ${e.message}`);
  }

  // GPS2_RAW (msg 124) at 2 Hz — secondary GPS (dual-GPS setups)
  try {
    const frame = buildSetMessageInterval(
      nextSeq(px), 255, 190, targetSys, targetComp, MSG_GPS2_RAW, 500000
    );
    port.write(frame, err => {
      if (err) log(`${px.toUpperCase()} GPS2 interval write error: ${err.message}`);
    });
  } catch (e) {
    log(`${px.toUpperCase()} GPS2 interval build error: ${e.message}`);
  }

  // Fallback: REQUEST_DATA_STREAM EXTRA1 (stream 10) at STREAM_HZ for attitude
  try {
    const frame = buildRequestDataStream(
      nextSeq(px), 255, 190, targetSys, targetComp, 10, STREAM_HZ, 1
    );
    port.write(frame, err => {
      if (err) log(`${px.toUpperCase()} REQUEST_DATA_STREAM write error: ${err.message}`);
    });
  } catch (e) {
    log(`${px.toUpperCase()} REQUEST_DATA_STREAM build error: ${e.message}`);
  }

  // Fallback: REQUEST_DATA_STREAM RAW_SENSORS (stream 1) at 2 Hz — includes GPS_RAW_INT
  try {
    const frame = buildRequestDataStream(
      nextSeq(px), 255, 190, targetSys, targetComp, 1, 2, 1
    );
    port.write(frame, err => {
      if (err) log(`${px.toUpperCase()} RAW_SENSORS stream write error: ${err.message}`);
    });
  } catch (e) {
    log(`${px.toUpperCase()} RAW_SENSORS stream build error: ${e.message}`);
  }
}

// ---------------------------------------------------------------------------
// Teensy USB serial — line-buffered text stream, independent of PX session
// ---------------------------------------------------------------------------
async function openTeensy(portPath, baudRate) {
  if (teensyPort) {
    if (teensyPort.isOpen) await new Promise(r => teensyPort.close(() => r()));
    teensyPort = null;
  }

  const sp = new SerialPort({ path: portPath, baudRate, autoOpen: false });
  let lineBuf = '';

  sp.on('data', (chunk) => {
    lineBuf += chunk.toString('utf8');
    let idx;
    while ((idx = lineBuf.indexOf('\n')) !== -1) {
      const line = lineBuf.slice(0, idx).replace(/\r$/, '');
      lineBuf = lineBuf.slice(idx + 1);
      if (line.length > 0) broadcast({ type: 'teensy', line });
    }
  });

  sp.on('error', (err) => {
    log(`Teensy error: ${err.message}`);
    broadcast({ type: 'teensyStatus', state: 'error' });
  });

  sp.on('close', () => {
    log('Teensy port closed');
    broadcast({ type: 'teensyStatus', state: 'disconnected' });
    teensyPort = null;
  });

  return new Promise((resolve, reject) => {
    sp.open((err) => {
      if (err) { reject(err); return; }
      teensyPort = sp;
      broadcast({ type: 'teensyStatus', state: 'connected' });
      log(`Teensy opened ${portPath} @ ${baudRate} baud`);
      resolve(sp);
    });
  });
}

// ---------------------------------------------------------------------------
// Open a serial port, attach MAVLink parser
// ---------------------------------------------------------------------------
function openPort(portPath, baudRate, px, onMessage) {
  const sp = new SerialPort({
    path: portPath,
    baudRate: baudRate,
    autoOpen: false,
  });

  const parser = new MAVLinkParser((msg) => {
    rateCounters[px]++;
    onMessage(msg);
  });

  sp.on('data', (chunk) => {
    parser.push(chunk);
  });

  sp.on('error', (err) => {
    log(`${px.toUpperCase()} serial error: ${err.message}`);
    broadcast({ type: 'status', px, state: 'error' });
  });

  sp.on('close', () => {
    log(`${px.toUpperCase()} port closed`);
    broadcast({ type: 'status', px, state: 'disconnected' });
  });

  return new Promise((resolve, reject) => {
    sp.open((err) => {
      if (err) {
        reject(err);
      } else {
        broadcast({ type: 'status', px, state: 'connected' });
        log(`${px.toUpperCase()} opened ${portPath} @ ${baudRate} baud`);
        resolve(sp);
      }
    });
  });
}

// ---------------------------------------------------------------------------
// Start session
// ---------------------------------------------------------------------------
async function startSession(port1Path, port2Path, baudRate, windowMs) {
  if (isRunning) {
    log('Already running — stop first');
    return;
  }

  // Known detected sysIds for request retries
  const sysIds = { px1: null, px2: null };

  function handleMessage(px, msg) {
    // Track sysId for targeting
    if (!sysIds[px]) {
      sysIds[px] = msg.sysId;
      activeSysIds[px] = msg.sysId;  // also store at module scope for setParam
      broadcast({ type: 'status', px, state: 'streaming', sysId: msg.sysId });
      log(`${px.toUpperCase()} detected system ID ${msg.sysId}`);
    }

    if (msg.msgId === 30 || msg.msgId === 31) {
      broadcast({
        type: 'quat',
        px,
        w: msg.w,
        x: msg.x,
        y: msg.y,
        z: msg.z,
        t: msg.timeBootMs,
      });
    } else if (msg.msgId === 24 && px === 'px1') {
      // GPS_RAW_INT: primary GPS port on PX1 only
      broadcast({ type: 'gps', slot: 'gps1', fixType: msg.fixType, sats: msg.sats });
    } else if (msg.msgId === 124 && px === 'px1') {
      // GPS2_RAW: secondary GPS port on PX1 only
      broadcast({ type: 'gps', slot: 'gps2', fixType: msg.fixType, sats: msg.sats });
    } else if (msg.msgId === 22) {
      // PARAM_VALUE — check if a pending triggerVerify is waiting for this param
      if (msg.paramId === 'STEST_ENABLE' && pendingVerifies[px]) {
        const pending = pendingVerifies[px];
        clearTimeout(pending.timeout);
        delete pendingVerifies[px];
        log(`${px.toUpperCase()} STEST_ENABLE read=${msg.value} — sending PARAM_SET=1`);
        const port = px === 'px1' ? px1Port : px2Port;
        if (port && port.isOpen) {
          try {
            const frame = buildParamSet(
              nextSeq(px), 255, 190, activeSysIds[px] || 1, 1, 'STEST_ENABLE', 1.0
            );
            port.write(frame, err => {
              if (err) log(`PARAM_SET write error: ${err.message}`);
              else log(`${px.toUpperCase()} → PARAM_SET STEST_ENABLE=1`);
            });
            broadcast({ type: 'verifyStatus', state: 'triggered' });
          } catch (e) {
            log(`PARAM_SET build error: ${e.message}`);
          }
        }
      }
    } else if (msg.msgId === 253) {
      // STATUSTEXT from Lua script — forward to browser for the verification log
      broadcast({ type: 'statustext', px, severity: msg.severity, text: msg.text });
    }
  }

  try {
    px1Port   = await openPort(port1Path, baudRate, 'px1', (m) => handleMessage('px1', m));
    px2Port   = await openPort(port2Path, baudRate, 'px2', (m) => handleMessage('px2', m));
    isRunning = true;

    // Request streams (will retry after a short delay once sysIds known)
    setTimeout(() => {
      if (px1Port && px1Port.isOpen) requestAttitude(px1Port, 'px1', sysIds.px1 || 1);
      if (px2Port && px2Port.isOpen) requestAttitude(px2Port, 'px2', sysIds.px2 || 1);
    }, 500);

    // Retry request every 5s in case board resets
    const reqInterval = setInterval(() => {
      if (!isRunning) { clearInterval(reqInterval); return; }
      if (px1Port && px1Port.isOpen) requestAttitude(px1Port, 'px1', sysIds.px1 || 1);
      if (px2Port && px2Port.isOpen) requestAttitude(px2Port, 'px2', sysIds.px2 || 1);
    }, 5000);

    // Rate logger every 5 s
    rateLastTime = Date.now();
    rateCounters.px1 = 0;
    rateCounters.px2 = 0;
    rateInterval = setInterval(() => {
      const now     = Date.now();
      const elapsed = (now - rateLastTime) / 1000;
      rateLastTime  = now;
      const hz1 = (rateCounters.px1 / elapsed).toFixed(1);
      const hz2 = (rateCounters.px2 / elapsed).toFixed(1);
      rateCounters.px1 = 0;
      rateCounters.px2 = 0;
      broadcast({ type: 'rate', px: 'px1', hz: parseFloat(hz1) });
      broadcast({ type: 'rate', px: 'px2', hz: parseFloat(hz2) });
      log(`Rate — PX1: ${hz1} Hz  PX2: ${hz2} Hz`);
    }, 5000);

    log(`Session started — window: ${windowMs} ms`);

  } catch (e) {
    log(`Failed to start: ${e.message}`);
    broadcast({ type: 'status', px: 'px1', state: 'error' });
    broadcast({ type: 'status', px: 'px2', state: 'error' });
    await stopSession();
  }
}

// ---------------------------------------------------------------------------
// Stop session
// ---------------------------------------------------------------------------
async function stopSession() {
  isRunning = false;

  if (rateInterval) {
    clearInterval(rateInterval);
    rateInterval = null;
  }

  const closers = [];
  if (px1Port && px1Port.isOpen) {
    closers.push(new Promise(r => px1Port.close(() => r())));
  }
  if (px2Port && px2Port.isOpen) {
    closers.push(new Promise(r => px2Port.close(() => r())));
  }
  await Promise.allSettled(closers);

  px1Port   = null;
  px2Port   = null;
  px1Parser = null;
  px2Parser = null;

  log('Session stopped');
  broadcast({ type: 'status', px: 'px1', state: 'disconnected' });
  broadcast({ type: 'status', px: 'px2', state: 'disconnected' });
}

// ---------------------------------------------------------------------------
// WebSocket message handler
// ---------------------------------------------------------------------------
wss.on('connection', (ws, req) => {
  log(`Browser connected from ${req.socket.remoteAddress}`);

  ws.on('message', async (raw) => {
    let msg;
    try {
      msg = JSON.parse(raw.toString());
    } catch {
      return;
    }

    switch (msg.type) {
      case 'listPorts':
        await listAndSendPorts(ws);
        break;

      case 'start': {
        const baud   = parseInt(msg.baud,   10) || 115200;
        const window = parseInt(msg.window, 10) || 50;
        if (!msg.port1 || !msg.port2) {
          log('start: port1 and port2 are required');
          break;
        }
        if (msg.port1 === msg.port2) {
          log('start: port1 and port2 must be different ports');
          break;
        }
        await startSession(msg.port1, msg.port2, baud, window);
        break;
      }

      case 'stop':
        await stopSession();
        break;

      // ── Verify — two-step: PARAM_REQUEST_READ → PARAM_VALUE ACK → PARAM_SET=1 ──
      case 'triggerVerify': {
        const px   = 'px1';
        const port = px1Port;
        if (!port || !port.isOpen) {
          log('triggerVerify: PX1 not connected');
          broadcast({ type: 'verifyStatus', state: 'error', msg: 'PX1 not connected' });
          break;
        }

        // Cancel any previous pending verify
        if (pendingVerifies[px]) {
          clearTimeout(pendingVerifies[px].timeout);
          delete pendingVerifies[px];
        }

        const sysId = activeSysIds[px] || 1;

        // Step 1: request the current value of STEST_ENABLE to prime the param lookup
        try {
          const frame = buildParamRequestRead(nextSeq(px), 255, 190, sysId, 1, 'STEST_ENABLE');
          port.write(frame, err => {
            if (err) log(`PARAM_REQUEST_READ write error: ${err.message}`);
            else log('PX1 → PARAM_REQUEST_READ STEST_ENABLE');
          });
        } catch (e) {
          log(`PARAM_REQUEST_READ build error: ${e.message}`);
          break;
        }

        broadcast({ type: 'verifyStatus', state: 'reading' });

        // Step 2: wait for PARAM_VALUE ACK in handleMessage; fall back after 2 s
        const timeout = setTimeout(() => {
          if (!pendingVerifies[px]) return;
          delete pendingVerifies[px];
          log('PX1 STEST_ENABLE: param ACK timeout — sending PARAM_SET anyway');
          if (port && port.isOpen) {
            try {
              const frame = buildParamSet(nextSeq(px), 255, 190, sysId, 1, 'STEST_ENABLE', 1.0);
              port.write(frame, err => {
                if (err) log(`PARAM_SET write error: ${err.message}`);
                else log('PX1 → PARAM_SET STEST_ENABLE=1 (fallback after timeout)');
              });
              broadcast({ type: 'verifyStatus', state: 'triggered' });
            } catch (e) {
              log(`PARAM_SET build error: ${e.message}`);
              broadcast({ type: 'verifyStatus', state: 'error', msg: e.message });
            }
          } else {
            // Port closed — still unblock the browser button
            broadcast({ type: 'verifyStatus', state: 'error', msg: 'PX1 port closed' });
          }
        }, 2000);

        pendingVerifies[px] = { timeout };
        break;
      }

      // ── Teensy USB serial ─────────────────────────────────────────────────
      case 'teensyConnect': {
        if (!msg.port) { log('teensyConnect: port required'); break; }
        const baud = parseInt(msg.baud, 10) || 115200;
        openTeensy(msg.port, baud).catch(e => {
          log(`Teensy open failed: ${e.message}`);
          broadcast({ type: 'teensyStatus', state: 'error' });
        });
        break;
      }

      case 'teensyDisconnect': {
        if (teensyPort && teensyPort.isOpen) {
          teensyPort.close(() => log('Teensy disconnected'));
        } else {
          broadcast({ type: 'teensyStatus', state: 'disconnected' });
        }
        break;
      }

      default:
        log(`Unknown message type: ${msg.type}`);
    }
  });

  ws.on('close', () => {
    // No cleanup needed per-connection
  });

  ws.on('error', (e) => {
    console.error('WS error:', e.message);
  });

  // Send current status on connect
  ws.send(JSON.stringify({
    type: 'status', px: 'px1',
    state: isRunning ? 'streaming' : 'disconnected',
  }));
  ws.send(JSON.stringify({
    type: 'status', px: 'px2',
    state: isRunning ? 'streaming' : 'disconnected',
  }));
  ws.send(JSON.stringify({
    type: 'teensyStatus',
    state: (teensyPort && teensyPort.isOpen) ? 'connected' : 'disconnected',
  }));
});

// ---------------------------------------------------------------------------
// Start HTTP + WS server
// ---------------------------------------------------------------------------
server.listen(PORT, '0.0.0.0', () => {
  console.log(`Pixhawk Validator running at http://localhost:${PORT}`);
});

// Graceful shutdown
process.on('SIGINT',  () => stopSession().then(() => process.exit(0)));
process.on('SIGTERM', () => stopSession().then(() => process.exit(0)));
