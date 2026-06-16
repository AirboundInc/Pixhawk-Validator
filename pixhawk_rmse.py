#!/usr/bin/env python3
"""
Pixhawk Orientation RMSE Validator

Connects to two Pixhawks via USB, synchronises ATTITUDE_QUATERNION streams
by MAVLink time_boot_ms, and computes a live geodesic angular RMSE between
the two units in degrees.  A 3D cuboid for each unit shows live orientation.

Usage:
    python pixhawk_rmse.py
"""

import sys
import time
import threading
import math
import queue
import tkinter as tk
from tkinter import ttk
from collections import deque

try:
    import numpy as np
except ImportError:
    sys.exit("numpy not found.      Run:  pip install numpy")

# matplotlib is no longer required — 3-D views use a native tkinter Canvas renderer.

try:
    from pymavlink import mavutil
except ImportError:
    sys.exit("pymavlink not found.  Run:  pip install pymavlink")

try:
    import serial.tools.list_ports as list_ports
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False


# ── Tuneable constants ──────────────────────────────────────────────────────
STREAM_HZ      = 50    # Hz requested from each Pixhawk
BUFFER_MAX     = 2000  # Max unmatched samples per unit
OFFSET_HISTORY = 50    # Pairs used to track clock offset
RMSE_MS        = 200   # RMSE label refresh interval (ms)
VIZ_MS         = 33    # 3D view refresh interval (ms)  ≈ 30 Hz
# ───────────────────────────────────────────────────────────────────────────

BAUD_OPTIONS = ['57600', '115200', '230400', '921600']

# Board orientation presets — quaternion (w,x,y,z) describing the physical
# mounting rotation of the board relative to the "standard" forward orientation.
# The conjugate is applied to q_measured so the corrected quaternion represents
# what the board would report if mounted in the standard orientation.
#   Roll+Yaw inverted, Pitch fine  → Pitch 180°
#   Pitch+Yaw inverted, Roll fine  → Roll 180°
#   Roll+Pitch inverted, Yaw fine  → Yaw 180°
ORIENT_PRESETS: dict[str, tuple] = {
    'None':       (1.0,    0.0,    0.0,    0.0),
    'Yaw 90°':   (0.7071, 0.0,    0.0,    0.7071),
    'Yaw 180°':  (0.0,    0.0,    0.0,    1.0),
    'Yaw 270°':  (0.7071, 0.0,    0.0,   -0.7071),
    'Roll 180°': (0.0,    1.0,    0.0,    0.0),
    'Pitch 180°':(0.0,    0.0,    1.0,    0.0),
}

# ── Cuboid geometry (body frame) ────────────────────────────────────────────
_W, _D, _H = 1.5, 1.0, 0.35
_VERTS = np.array([
    [-_W, -_D, -_H], [ _W, -_D, -_H], [ _W,  _D, -_H], [-_W,  _D, -_H],
    [-_W, -_D,  _H], [ _W, -_D,  _H], [ _W,  _D,  _H], [-_W,  _D,  _H],
], dtype=float)
_FACES = [
    [4,5,6,7], [0,1,2,3], [0,1,5,4], [3,2,6,7], [0,3,7,4], [1,2,6,5],
]
_COLORS_PX1 = ['#4FC3F7','#0277BD','#B3E5FC','#01579B','#29B6F6','#0288D1']
_COLORS_PX2 = ['#EF9A9A','#B71C1C','#FFCDD2','#C62828','#EF5350','#D32F2F']

# Camera view matrix — azimuth 45°, elevation 30°.
# Rows: [screen-right, screen-up, depth] axes expressed in world (NED-flipped) coords.
_az = math.radians(45);  _el = math.radians(30)
_ca, _sa = math.cos(_az), math.sin(_az)
_ce, _se = math.cos(_el), math.sin(_el)
_CAM_MAT = np.array([
    [ _ca,      -_sa,       0   ],
    [ _se*_sa,   _se*_ca,  _ce  ],
    [ _ce*_sa,   _ce*_ca, -_se  ],
])
del _az, _el, _ca, _sa, _ce, _se


# ── Quaternion helpers ──────────────────────────────────────────────────────

def quat_mul(a, b):
    """Hamilton product a ⊗ b, both (w,x,y,z)."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw*bw - ax*bx - ay*by - az*bz,
            aw*bx + ax*bw + ay*bz - az*by,
            aw*by - ax*bz + ay*bw + az*bx,
            aw*bz + ax*by - ay*bx + az*bw)


def quat_conj(q):
    """Conjugate (= inverse for unit quaternions)."""
    w, x, y, z = q
    return (w, -x, -y, -z)


def quat_norm(q):
    """Normalise to unit length."""
    w, x, y, z = q
    n = math.sqrt(w*w + x*x + y*y + z*z)
    return (w/n, x/n, y/n, z/n) if n > 1e-10 else (1.0, 0.0, 0.0, 0.0)


def quat_angle(q):
    """Geodesic angle of a unit quaternion (rotation magnitude, radians)."""
    return 2.0 * math.acos(min(1.0, abs(q[0])))


def euler_to_quat(roll, pitch, yaw):
    """Euler angles (radians) → Hamilton quaternion (w,x,y,z)."""
    cr, sr = math.cos(roll/2),  math.sin(roll/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    cy, sy = math.cos(yaw/2),   math.sin(yaw/2)
    return (cr*cp*cy + sr*sp*sy,
            sr*cp*cy - cr*sp*sy,
            cr*sp*cy + sr*cp*sy,
            cr*cp*sy - sr*sp*cy)


def quat_to_rot(w, x, y, z) -> np.ndarray:
    """Quaternion (w,x,y,z) → 3×3 rotation matrix."""
    return np.array([
        [1-2*(y*y+z*z),   2*(x*y-w*z),   2*(x*z+w*y)],
        [  2*(x*y+w*z), 1-2*(x*x+z*z),   2*(y*z-w*x)],
        [  2*(x*z-w*y),   2*(y*z+w*x), 1-2*(x*x+y*y)],
    ])


def rmse(errors: list) -> float:
    if not errors:
        return 0.0
    return math.sqrt(sum(e*e for e in errors) / len(errors))


# ── Sample buffer ───────────────────────────────────────────────────────────

class SampleBuffer:
    """Thread-safe buffer storing quaternion samples keyed by normalised boot-ms."""

    def __init__(self, name: str):
        self.name = name
        self._buf: dict = {}          # {t_norm_ms: (w, x, y, z)}
        self._t0 = None
        self._lock = threading.Lock()
        self.total_received = 0
        self.latest_quat = (1.0, 0.0, 0.0, 0.0)   # updated every message (GIL-safe)

    def reset(self):
        with self._lock:
            self._buf.clear()
            self._t0 = None
            self.total_received = 0
        # latest_quat is intentionally NOT reset — the 3D view should hold
        # its last known pose rather than snapping to identity on every Start.

    def add(self, time_boot_ms: int, w: float, x: float, y: float, z: float):
        with self._lock:
            if self._t0 is None:
                self._t0 = time_boot_ms
            t = time_boot_ms - self._t0
            self._buf[t] = (w, x, y, z)
            self.total_received += 1
            if len(self._buf) > BUFFER_MAX:
                del self._buf[min(self._buf)]

    def oldest_time(self):
        with self._lock:
            return min(self._buf) if self._buf else None

    def pop_nearest(self, t_query: int, window_ms: int):
        with self._lock:
            if not self._buf:
                return None
            best = min(self._buf, key=lambda t: abs(t - t_query))
            if abs(best - t_query) <= window_ms:
                return (best,) + self._buf.pop(best)
            return None

    def size(self) -> int:
        with self._lock:
            return len(self._buf)


# ── Reader thread ───────────────────────────────────────────────────────────

def reader_thread(port: str, baud: int, buf: SampleBuffer,
                  stop_event: threading.Event, log_q: queue.Queue,
                  status: dict):
    def log(msg):
        log_q.put(f'[{buf.name}] {msg}')

    try:
        conn = mavutil.mavlink_connection(port, baud=baud)
    except Exception as e:
        status['state'] = 'error'
        log(f'Could not open {port}: {e}')
        return

    log(f'Waiting for heartbeat on {port} …')
    if conn.wait_heartbeat(timeout=15) is None:
        status['state'] = 'error'
        log(f'No heartbeat within 15 s on {port}')
        return

    status['sysid'] = conn.target_system
    status['state'] = 'connected'
    log(f'Heartbeat OK — system {conn.target_system}')

    interval_us = int(1_000_000 / STREAM_HZ)   # e.g. 50 Hz → 20 000 µs

    # Request ATTITUDE_QUATERNION (msg ID 31) at STREAM_HZ via the modern API.
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE_QUATERNION,  # 31
        interval_us,
        0, 0, 0, 0, 0,
    )
    # Legacy fallback: also request EXTRA1 stream so older firmware sends ATTITUDE.
    conn.mav.request_data_stream_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, STREAM_HZ, 1,
    )
    log(f'Requested ATTITUDE_QUATERNION at {STREAM_HZ} Hz ({interval_us} µs interval)')

    msg_count = 0
    t_rate_check = time.time()

    while not stop_event.is_set():
        msg = conn.recv_match(
            type=['ATTITUDE', 'ATTITUDE_QUATERNION'],
            blocking=True, timeout=1.0,
        )
        if msg is None:
            continue

        if msg.get_type() == 'ATTITUDE_QUATERNION':
            q = quat_norm((msg.q1, msg.q2, msg.q3, msg.q4))
        else:
            q = quat_norm(euler_to_quat(msg.roll, msg.pitch, msg.yaw))

        buf.latest_quat = q
        buf.add(msg.time_boot_ms, *q)

        msg_count += 1
        now = time.time()
        if now - t_rate_check >= 5.0:
            actual_hz = msg_count / (now - t_rate_check)
            log(f'Actual rate: {actual_hz:.1f} Hz  (requested {STREAM_HZ} Hz)')
            msg_count = 0
            t_rate_check = now

    log('Disconnected.')


# ── Matching ────────────────────────────────────────────────────────────────

def drain_matches(buf1, buf2, window_ms, offset_history, angle_errs,
                  corr1=(1.0, 0.0, 0.0, 0.0),
                  corr2=(1.0, 0.0, 0.0, 0.0)) -> int:
    """
    Match quaternion samples from both buffers by normalised timestamp and
    compute the raw geodesic angular distance between each pair.

        q_err  = q1_corrected^{-1} ⊗ q2_corrected
        angle  = 2·arccos(|w of q_err|)

    Corrections (corr1/corr2 = quat_conj of the mount preset) are applied
    before computing the error so the RMSE matches the 3D visualisation.
    """
    matched = 0
    clock_offset = int(round(sum(offset_history) / len(offset_history))) \
                   if offset_history else 0

    while True:
        t1 = buf1.oldest_time()
        if t1 is None:
            break
        s1 = buf1.pop_nearest(t1, 0)
        if s1 is None:
            break
        t1_norm, w1, x1, y1, z1 = s1
        s2 = buf2.pop_nearest(t1_norm + clock_offset, window_ms)
        if s2 is None:
            continue
        t2_norm, w2, x2, y2, z2 = s2
        offset_history.append(t2_norm - t1_norm)

        q1 = quat_norm(quat_mul((w1, x1, y1, z1), corr1))
        q2 = quat_norm(quat_mul((w2, x2, y2, z2), corr2))
        q_err = quat_mul(quat_conj(q1), q2)
        angle_errs.append(quat_angle(q_err))
        matched += 1

    return matched


# ── Fast 3-D cuboid view (tkinter Canvas + perspective projection) ───────────

class CuboidView(tk.Canvas):
    """
    Software-rendered 3-D cuboid using a plain tkinter Canvas.
    Perspective projection + painter's algorithm — no matplotlib needed.
    Typical frame time: <1 ms (vs 50-150 ms for Poly3DCollection).
    """

    _FOCAL = 5.0   # perspective focal distance (world units)
    _SCALE = 0.32  # fraction of min(w, h) used as projection scale

    def __init__(self, parent, title: str, face_colors: list, **kw):
        super().__init__(parent, bg='#FAFAFA', **kw)
        self._title       = title
        self._face_colors = face_colors
        self._q           = (1.0, 0.0, 0.0, 0.0)
        self.bind('<Configure>', lambda _: self._redraw())

    def set_quat(self, q: tuple):
        self._q = q
        self._redraw()

    def _redraw(self):
        w = self.winfo_width()
        h = self.winfo_height()
        if w < 10 or h < 10:
            return
        self.delete('all')

        R     = quat_to_rot(*self._q)
        verts = (R @ _VERTS.T).T
        verts[:, 2] *= -1                        # NED Z-down → display Z-up
        view  = (_CAM_MAT @ verts.T).T           # apply camera rotation

        cx    = w / 2
        cy    = h / 2
        scale = min(w, h) * self._SCALE
        f     = self._FOCAL

        def proj(v):
            d = v[2] + f
            return cx + scale * v[0] / d, cy - scale * v[1] / d

        pts2d = [proj(v) for v in view]

        # Painter's algorithm: back faces first (largest depth drawn first)
        face_order = sorted(range(len(_FACES)),
                            key=lambda i: sum(view[vi, 2] for vi in _FACES[i]),
                            reverse=True)
        for fi in face_order:
            flat = [c for vi in _FACES[fi] for c in pts2d[vi]]
            self.create_polygon(flat, fill=self._face_colors[fi],
                                outline='#333333', width=1)

        # Axis arrows: X=red, Y=green, Z=blue
        ox, oy = cx, cy   # origin always projects to canvas centre
        for vec, color in [([1.7, 0, 0], '#E53935'),
                            ([0, 1.7, 0], '#43A047'),
                            ([0, 0, 1.7], '#1E88E5')]:
            rv       = R @ np.array(vec, dtype=float)
            rv[2]   *= -1
            rv_cam   = _CAM_MAT @ rv
            ax, ay   = proj(rv_cam)
            self.create_line(ox, oy, ax, ay, fill=color, width=2,
                             arrow='last', arrowshape=(8, 10, 4))

        self.create_text(cx, 14, text=self._title,
                         font=('Arial', 10, 'bold'), fill='#333333')


# ── GUI ─────────────────────────────────────────────────────────────────────

class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title('Pixhawk Orientation RMSE Validator')
        self.resizable(True, True)

        self._stop_event  = threading.Event()
        self._buf1        = SampleBuffer('PX1')
        self._buf2        = SampleBuffer('PX2')
        self._status1     = {'state': 'idle'}
        self._status2     = {'state': 'idle'}
        self._log_q       = queue.Queue()
        self._running     = False
        self._angle_errs: list = []
        self._offset_history: deque = deque(maxlen=OFFSET_HISTORY)
        self._total_pairs = 0

        self._build_ui()
        self._refresh_ports()
        self.after(RMSE_MS, self._rmse_tick)
        self.after(VIZ_MS,  self._viz_tick)

    # ── UI ─────────────────────────────────────────────────────────────────

    def _build_ui(self):
        P = dict(padx=10, pady=6)

        # ── Left panel ───────────────────────────────────────────────────
        left = ttk.Frame(self)
        left.grid(row=0, column=0, sticky='nsew')
        self.columnconfigure(0, weight=0)
        self.columnconfigure(1, weight=1)

        # Configuration
        cfg = ttk.LabelFrame(left, text='Configuration')
        cfg.grid(row=0, column=0, sticky='ew', **P)

        preset_names = list(ORIENT_PRESETS.keys())

        ttk.Label(cfg, text='Pixhawk 1 port:').grid(row=0, column=0, sticky='w', padx=6, pady=4)
        self._port1_var = tk.StringVar()
        self._port1_cb  = ttk.Combobox(cfg, textvariable=self._port1_var, width=28)
        self._port1_cb.grid(row=0, column=1, padx=6, pady=4)
        ttk.Label(cfg, text='Orientation:').grid(row=0, column=2, sticky='w', padx=6, pady=4)
        self._corr1_var = tk.StringVar(value='None')
        ttk.Combobox(cfg, textvariable=self._corr1_var, values=preset_names,
                     width=11, state='readonly').grid(row=0, column=3, padx=6, pady=4)

        ttk.Label(cfg, text='Pixhawk 2 port:').grid(row=1, column=0, sticky='w', padx=6, pady=4)
        self._port2_var = tk.StringVar()
        self._port2_cb  = ttk.Combobox(cfg, textvariable=self._port2_var, width=28)
        self._port2_cb.grid(row=1, column=1, padx=6, pady=4)
        ttk.Label(cfg, text='Orientation:').grid(row=1, column=2, sticky='w', padx=6, pady=4)
        self._corr2_var = tk.StringVar(value='None')
        ttk.Combobox(cfg, textvariable=self._corr2_var, values=preset_names,
                     width=11, state='readonly').grid(row=1, column=3, padx=6, pady=4)

        ttk.Label(cfg, text='Baud rate:').grid(row=2, column=0, sticky='w', padx=6, pady=4)
        self._baud_var = tk.StringVar(value='57600')
        ttk.Combobox(cfg, textvariable=self._baud_var,
                     values=BAUD_OPTIONS, width=12).grid(row=2, column=1, padx=6, pady=4, sticky='w')

        ttk.Label(cfg, text='Sync window (ms):').grid(row=3, column=0, sticky='w', padx=6, pady=4)
        self._window_var = tk.StringVar(value='50')
        ttk.Entry(cfg, textvariable=self._window_var, width=8).grid(row=3, column=1, padx=6, pady=4, sticky='w')

        self._refresh_btn = ttk.Button(cfg, text='⟳  Refresh ports', command=self._refresh_ports)
        self._refresh_btn.grid(row=2, column=2, rowspan=2, columnspan=2, padx=10, pady=4)

        # Thread-safe correction quaternions (conjugate of preset — GIL-safe tuple assignment)
        self._corr1_quat: tuple = (1.0, 0.0, 0.0, 0.0)
        self._corr2_quat: tuple = (1.0, 0.0, 0.0, 0.0)
        self._corr1_var.trace_add('write', lambda *_: self._update_corr(1))
        self._corr2_var.trace_add('write', lambda *_: self._update_corr(2))

        # Controls
        ctrl = ttk.Frame(left)
        ctrl.grid(row=1, column=0, sticky='ew', **P)
        self._start_btn = ttk.Button(ctrl, text='Start', width=14, command=self._start)
        self._start_btn.pack(side='left', padx=6)
        self._stop_btn  = ttk.Button(ctrl, text='Stop',  width=14,
                                     command=self._stop, state='disabled')
        self._stop_btn.pack(side='left', padx=6)

        # Status / RMSE
        st = ttk.LabelFrame(left, text='Status')
        st.grid(row=2, column=0, sticky='ew', **P)

        conn_row = ttk.Frame(st)
        conn_row.grid(row=0, column=0, columnspan=4, sticky='w', padx=6, pady=4)
        ttk.Label(conn_row, text='PX1:').pack(side='left')
        self._px1_ind = ttk.Label(conn_row, text='● Idle', foreground='grey', width=24)
        self._px1_ind.pack(side='left', padx=(2, 20))
        ttk.Label(conn_row, text='PX2:').pack(side='left')
        self._px2_ind = ttk.Label(conn_row, text='● Idle', foreground='grey', width=24)
        self._px2_ind.pack(side='left', padx=2)

        metrics = [
            ('Matched pairs', '_lbl_pairs'),
            ('Angular RMSE',  '_lbl_rmse'),
            ('Clock offset',  '_lbl_clk'),
        ]
        for i, (label, attr) in enumerate(metrics):
            row, col = divmod(i, 2)
            c = col * 2
            ttk.Label(st, text=label + ':', anchor='e', width=15).grid(
                row=row+1, column=c, sticky='e', padx=(10, 2), pady=4)
            lbl = ttk.Label(st, text='—', anchor='w', width=16,
                            font=('Courier', 12, 'bold'))
            lbl.grid(row=row+1, column=c+1, sticky='w', padx=(0, 10), pady=4)
            setattr(self, attr, lbl)

        # Log
        log_f = ttk.LabelFrame(left, text='Log')
        log_f.grid(row=3, column=0, sticky='ew', **P)
        self._log_text = tk.Text(log_f, height=8, width=60,
                                 state='disabled', font=('Courier', 9))
        sb = ttk.Scrollbar(log_f, command=self._log_text.yview)
        self._log_text.configure(yscrollcommand=sb.set)
        self._log_text.grid(row=0, column=0, padx=4, pady=4)
        sb.grid(row=0, column=1, sticky='ns', pady=4)

        # ── Right panel: 3-D views ────────────────────────────────────────
        viz = ttk.LabelFrame(self, text='3D Orientation')
        viz.grid(row=0, column=1, sticky='nsew', padx=10, pady=6)
        self.rowconfigure(0, weight=1)

        self._view1 = CuboidView(viz, 'Pixhawk 1', _COLORS_PX1)
        self._view1.pack(side='left', fill='both', expand=True, padx=4, pady=4)

        ttk.Separator(viz, orient='vertical').pack(side='left', fill='y', pady=8)

        self._view2 = CuboidView(viz, 'Pixhawk 2', _COLORS_PX2)
        self._view2.pack(side='left', fill='both', expand=True, padx=4, pady=4)

    # ── Orientation correction ─────────────────────────────────────────────

    def _update_corr(self, px: int):
        """Called when the user changes an orientation preset dropdown."""
        var = self._corr1_var if px == 1 else self._corr2_var
        q_mount = ORIENT_PRESETS.get(var.get(), (1.0, 0.0, 0.0, 0.0))
        corr = quat_conj(q_mount)   # q_corrected = q_measured ⊗ q_mount^{-1}
        if px == 1:
            self._corr1_quat = corr
        else:
            self._corr2_quat = corr

    # ── Port refresh ───────────────────────────────────────────────────────

    def _refresh_ports(self):
        if HAS_SERIAL:
            ports   = sorted(list_ports.comports(), key=lambda p: p.device)
            entries = [f'{p.device}  —  {p.description}' for p in ports]
            raw     = [p.device for p in ports]
            self._log(f'Found {len(raw)} serial port(s).')
        else:
            entries, raw = [], []
            self._log('pyserial unavailable — type port names manually (e.g. COM3).')

        self._port_raw = raw
        self._port1_cb['values'] = entries or ['(no ports detected)']
        self._port2_cb['values'] = entries or ['(no ports detected)']
        if len(raw) >= 1 and not self._port1_var.get():
            self._port1_cb.current(0)
        if len(raw) >= 2 and not self._port2_var.get():
            self._port2_cb.current(1)

    def _selected_port(self, var: tk.StringVar) -> str:
        return var.get().split('  —  ')[0].strip()

    # ── Session control ────────────────────────────────────────────────────

    def _start(self):
        port1 = self._selected_port(self._port1_var)
        port2 = self._selected_port(self._port2_var)
        if not port1 or not port2:
            self._log('Select a port for both Pixhawks.')
            return
        if port1 == port2:
            self._log('Both Pixhawks must be on different ports.')
            return
        try:
            baud   = int(self._baud_var.get())
            window = int(self._window_var.get())
        except ValueError:
            self._log('Invalid baud / window value.')
            return

        self._stop_event.clear()
        self._buf1.reset()
        self._buf2.reset()
        self._status1     = {'state': 'connecting'}
        self._status2     = {'state': 'connecting'}
        self._angle_errs.clear()
        self._offset_history.clear()
        self._total_pairs = 0
        self._update_metrics()

        self._running = True
        self._start_btn.configure(state='disabled')
        self._stop_btn.configure(state='normal')
        self._refresh_btn.configure(state='disabled')
        self._log(f'Starting — PX1={port1}  PX2={port2}  baud={baud}  window={window} ms')

        for port, buf, status in [(port1, self._buf1, self._status1),
                                  (port2, self._buf2, self._status2)]:
            threading.Thread(
                target=reader_thread,
                args=(port, baud, buf, self._stop_event, self._log_q, status),
                daemon=True,
            ).start()

        threading.Thread(
            target=self._match_loop, args=(window,), daemon=True,
        ).start()

    def _stop(self):
        self._stop_event.set()
        self._running = False
        self._start_btn.configure(state='normal')
        self._stop_btn.configure(state='disabled')
        self._refresh_btn.configure(state='normal')
        self._log('Session stopped.')
        if self._total_pairs > 0:
            self._log_final_summary()

    def _match_loop(self, window_ms: int):
        while not self._stop_event.is_set():
            drain_matches(self._buf1, self._buf2, window_ms,
                          self._offset_history, self._angle_errs,
                          self._corr1_quat, self._corr2_quat)
            time.sleep(0.1)

    # ── RMSE tick ──────────────────────────────────────────────────────────

    def _rmse_tick(self):
        try:
            while True:
                self._log(self._log_q.get_nowait())
        except queue.Empty:
            pass

        self._update_indicator(self._px1_ind, self._status1)
        self._update_indicator(self._px2_ind, self._status2)
        self._total_pairs = len(self._angle_errs)
        self._update_metrics()

        if self._running and (self._status1.get('state') == 'error'
                              and self._status2.get('state') == 'error'):
            self._stop()

        self.after(RMSE_MS, self._rmse_tick)

    def _update_indicator(self, label, status):
        state = status.get('state', 'idle')
        sysid = status.get('sysid', '')
        text, color = {
            'idle':       ('● Idle',                     'grey'),
            'connecting': ('● Connecting …',              'orange'),
            'connected':  (f'● Connected  (sys {sysid})', 'green'),
            'error':      ('● Error',                     'red'),
        }.get(state, ('● ?', 'grey'))
        label.configure(text=text, foreground=color)

    def _update_metrics(self):
        n = self._total_pairs
        self._lbl_pairs.configure(text=str(n))
        if n == 0:
            for lbl in (self._lbl_rmse, self._lbl_clk):
                lbl.configure(text='—')
            return

        ang_rmse = math.degrees(rmse(self._angle_errs))
        clk = sum(self._offset_history) / len(self._offset_history) \
              if self._offset_history else 0.0

        self._lbl_rmse.configure(text=f'{ang_rmse:.4f} °')
        self._lbl_clk.configure(text=f'{clk:+.1f} ms')

    # ── 3-D visualisation tick ─────────────────────────────────────────────

    def _viz_tick(self):
        for view, buf, corr in [
            (self._view1, self._buf1, self._corr1_quat),
            (self._view2, self._buf2, self._corr2_quat),
        ]:
            q = quat_norm(quat_mul(buf.latest_quat, corr))
            view.set_quat(q)

        self.after(VIZ_MS, self._viz_tick)

    # ── Logging ────────────────────────────────────────────────────────────

    def _log(self, msg: str):
        self._log_text.configure(state='normal')
        self._log_text.insert('end', msg + '\n')
        self._log_text.see('end')
        self._log_text.configure(state='disabled')

    def _log_final_summary(self):
        n   = self._total_pairs
        ang = math.degrees(rmse(self._angle_errs))
        for line in [
            '─' * 42,
            f'Final results  ({n} matched pairs)',
            f'  Angular RMSE : {ang:.4f} °  (geodesic distance)',
            f'  PX1 rx       : {self._buf1.total_received} samples',
            f'  PX2 rx       : {self._buf2.total_received} samples',
            '─' * 42,
        ]:
            self._log(line)


# ── Entry point ─────────────────────────────────────────────────────────────

if __name__ == '__main__':
    app = App()
    app.mainloop()
