#!/usr/bin/env python3
"""
serdes_gui.py  —  Real-time SerDes Link Training Dashboard

  Live eye diagrams, MSE convergence, per-lane status, keyboard/button resets.

Usage:
    python serdes_gui.py

Dependencies:
    pip install pyserial numpy matplotlib
"""

import argparse
import struct
import time
import sys
import threading
import queue
import numpy as np
import tkinter as tk
from tkinter import ttk, filedialog, scrolledtext
from datetime import datetime

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from matplotlib.collections import LineCollection

# ═══════════════════════════════════════════════════════════════════════
# Constants (must match FPGA main.c)
# ═══════════════════════════════════════════════════════════════════════

RX_FFE_PRE  = 3
RX_FFE_POST = 10
RX_FFE_LEN  = RX_FFE_PRE + 1 + RX_FFE_POST
N_DFE       = 1

SYNC_TX     = 0xAA
SYNC_RX     = 0x55
CMD_NORMAL  = 0x00
CMD_RESET   = 0x01
STATUS_OK   = 0x00
STATUS_RESET = 0x01

OSF         = 16
N_BIT       = 256
ADC_BITS    = 5
TX_FFE_PRE  = 0
TX_FFE_POST = 0
TX_FFE_LEN  = TX_FFE_PRE + 1 + TX_FFE_POST
MU_FFE      = 0.01
MU_DFE      = 0.00


# ═══════════════════════════════════════════════════════════════════════
# Channel simulation (identical to laptop_bridge.py)
# ═══════════════════════════════════════════════════════════════════════

def load_channel(filename):
    taps = []
    with open(filename) as f:
        for line in f:
            line = line.strip()
            if line and not line.startswith('#'):
                taps.append(float(line))
    return np.array(taps)

def generate_prbs(n_bit):
    return np.random.default_rng(42).choice([-1.0, 1.0], size=n_bit)

def upsample(bits, osf):
    return np.repeat(bits, osf)

def apply_tx_ffe(bits_osf, tx_ffe, osf):
    out = np.copy(bits_osf)
    n = len(bits_osf)
    for i in range(TX_FFE_PRE * osf, n):
        acc = 0.0
        for k in range(TX_FFE_LEN):
            idx = i + (k - TX_FFE_PRE) * osf
            if 0 <= idx < n:
                acc += tx_ffe[k] * bits_osf[idx]
        out[i] = acc
    return out

def apply_channel_fir(signal, h_fir):
    return np.convolve(signal, h_fir, mode='full')[:len(signal)]

def adc_quantize(x, bits):
    clamped = np.clip(x, -1.0, 1.0)
    levels = 1 << bits
    xq = np.round((clamped + 1.0) * (levels - 1) / 2.0).astype(int)
    return xq * 2.0 / (levels - 1) - 1.0

def simple_cdr(signal, osf, n_cdr=1000):
    best_phase, best_metric = 0, -1.0
    for phase in range(osf):
        metric = np.mean(np.abs(signal[phase::osf][:n_cdr]))
        if metric > best_metric:
            best_metric = metric
            best_phase = phase
    return best_phase

def build_signal_chain(channel_file):
    h_fir = load_channel(channel_file)
    bits = generate_prbs(N_BIT)
    bits_osf = upsample(bits, OSF)
    tx_ffe = np.zeros(TX_FFE_LEN); tx_ffe[TX_FFE_PRE] = 1.0
    tx_signal = apply_tx_ffe(bits_osf, tx_ffe, OSF)
    post_channel = apply_channel_fir(tx_signal, h_fir)
    post_adc = adc_quantize(post_channel, ADC_BITS)
    sample_phase = simple_cdr(post_adc, OSF)
    return post_adc, bits, sample_phase, len(h_fir)


# ═══════════════════════════════════════════════════════════════════════
# Serial protocol helpers
# ═══════════════════════════════════════════════════════════════════════

def build_error_packet(task_id, cmd, error, rx_buf, d_hist):
    pkt = bytearray([SYNC_TX, task_id & 0xFF, cmd & 0xFF])
    pkt += struct.pack('<f', error)
    for v in rx_buf:
        pkt += struct.pack('<f', v)
    for v in d_hist:
        pkt += struct.pack('<f', v)
    return bytes(pkt)

def recv_tap_packet(ser):
    n_floats = RX_FFE_LEN + N_DFE
    payload_len = n_floats * 4
    max_scan = payload_len * 4
    for _ in range(max_scan):
        b = ser.read(1)
        if len(b) == 0:
            raise TimeoutError("Sync timeout")
        if b[0] == SYNC_RX:
            break
    else:
        raise TimeoutError("No sync found")
    hdr = ser.read(2)
    if len(hdr) < 2:
        raise TimeoutError("Short header")
    task_id, status = hdr[0], hdr[1]
    raw = ser.read(payload_len)
    if len(raw) < payload_len:
        raise TimeoutError(f"Short payload: {len(raw)}/{payload_len}")
    values = struct.unpack(f'<{n_floats}f', raw)
    return task_id, status, np.array(values[:RX_FFE_LEN]), np.array(values[RX_FFE_LEN:])


# ═══════════════════════════════════════════════════════════════════════
# Background worker thread
# ═══════════════════════════════════════════════════════════════════════

class BridgeWorker(threading.Thread):
    """Runs the UART bridge loop in a background thread."""

    def __init__(self, port, baud, channel_file, n_tasks, gui_queue, cmd_queue):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.channel_file = channel_file
        self.n_tasks = n_tasks
        self.gui_queue = gui_queue    # worker → GUI
        self.cmd_queue = cmd_queue    # GUI → worker (reset commands)
        self.stop_event = threading.Event()

        # Shared state (written by worker, read by GUI under lock)
        self.lock = threading.Lock()
        self.rx_ffe   = [np.zeros(RX_FFE_LEN) for _ in range(n_tasks)]
        self.dfe_taps = [np.zeros(N_DFE) for _ in range(n_tasks)]
        self.mse_history = [[] for _ in range(n_tasks)]
        self.n_symbols = [0] * n_tasks
        self.reset_counts = [0] * n_tasks
        self.current_mse = [0.0] * n_tasks
        self.post_adc = None
        self.eq_output = None
        self.sample_phase = 0
        self.bits = None

    def stop(self):
        self.stop_event.set()

    def _emit(self, event_type, **kwargs):
        self.gui_queue.put((event_type, kwargs))

    def run(self):
        import serial
        try:
            post_adc, bits, sample_phase, n_taps = \
                build_signal_chain(self.channel_file)
        except Exception as e:
            self._emit('ERROR', msg=f"Channel load failed: {e}")
            return

        self._emit('LOG', msg=f"Channel: {n_taps} taps, CDR phase: {sample_phase}")

        with self.lock:
            self.post_adc = post_adc
            self.bits = bits
            self.sample_phase = sample_phase
            self.eq_output = [np.copy(post_adc).astype(float)
                              for _ in range(self.n_tasks)]
            for t in range(self.n_tasks):
                self.rx_ffe[t][:] = 0.0
                self.rx_ffe[t][RX_FFE_PRE] = 1.0

        try:
            ser = serial.Serial(self.port, self.baud, timeout=2.0)
        except Exception as e:
            self._emit('ERROR', msg=f"Serial open failed: {e}")
            return

        # Drain startup text
        ser.reset_input_buffer()
        time.sleep(1.0)
        startup = b""
        while ser.in_waiting:
            startup += ser.read(ser.in_waiting)
            time.sleep(0.1)
        if startup:
            self._emit('LOG', msg=f"FPGA: {startup.decode(errors='replace').strip()}")

        self._emit('LOG', msg=f"Serial open: {self.port} @ {self.baud}")
        self._emit('STARTED')

        # ── Per-lane local state ──────────────────────────────
        n = self.n_tasks
        rx_buf   = [np.zeros(RX_FFE_LEN) for _ in range(n)]
        d_hist   = [np.zeros(N_DFE) for _ in range(n)]
        rx_ffe_l = [np.zeros(RX_FFE_LEN) for _ in range(n)]
        dfe_l    = [np.zeros(N_DFE) for _ in range(n)]
        for t in range(n):
            rx_ffe_l[t][RX_FFE_PRE] = 1.0

        n_symbols = [0] * n
        mse_acc   = [0.0] * n
        report_interval = 100

        total_samples = len(post_adc)
        start_idx = (TX_FFE_PRE + 1) * OSF

        # Pending resets from GUI
        pending_resets = set()

        for pt in range(start_idx, total_samples):
            if self.stop_event.is_set():
                break
            if pt + sample_phase >= total_samples:
                break

            sample = post_adc[pt + sample_phase]

            # Check for reset commands from GUI
            while not self.cmd_queue.empty():
                try:
                    cmd_type, lane_id = self.cmd_queue.get_nowait()
                    if cmd_type == 'RESET' and 0 <= lane_id < n:
                        pending_resets.add(lane_id)
                except queue.Empty:
                    break

            for tid in range(n):
                if self.stop_event.is_set():
                    break

                rx_buf[tid][1:] = rx_buf[tid][:-1]
                rx_buf[tid][0] = sample

                y_ffe = np.dot(rx_ffe_l[tid], rx_buf[tid])
                y = y_ffe - np.dot(dfe_l[tid], d_hist[tid])

                with self.lock:
                    self.eq_output[tid][pt + sample_phase] = y

                d_hat = 1.0 if y >= 0.0 else -1.0
                sym_idx = pt // 1
                desired = bits[sym_idx] if sym_idx < N_BIT else d_hat
                error = desired - y
                mse_acc[tid] += error * error
                n_symbols[tid] += 1

                # Build command
                cmd = CMD_RESET if tid in pending_resets else CMD_NORMAL
                if tid in pending_resets:
                    pending_resets.discard(tid)

                # Send to FPGA
                pkt = build_error_packet(tid, cmd, float(error),
                                         rx_buf[tid].tolist(),
                                         d_hist[tid].tolist())
                ser.write(pkt)
                ser.flush()

                # Receive from FPGA
                try:
                    resp_id, status, new_ffe, new_dfe = recv_tap_packet(ser)
                except TimeoutError as e:
                    self._emit('ERROR', msg=f"Timeout lane {tid}: {e}")
                    ser.close()
                    self._emit('STOPPED')
                    return

                # Handle reset confirmation
                if status == STATUS_RESET:
                    with self.lock:
                        self.reset_counts[resp_id] += 1
                        rc = self.reset_counts[resp_id]
                        # Reset laptop-side state
                        rx_buf[resp_id][:] = 0.0
                        d_hist[resp_id][:] = 0.0
                        rx_ffe_l[resp_id][:] = 0.0
                        rx_ffe_l[resp_id][RX_FFE_PRE] = 1.0
                        dfe_l[resp_id][:] = 0.0
                        self.eq_output[resp_id] = np.copy(post_adc).astype(float)
                        mse_acc[resp_id] = 0.0
                        n_symbols[resp_id] = 0
                    self._emit('RESET', lane=resp_id, count=rc,
                               symbol=n_symbols[resp_id])
                else:
                    rx_ffe_l[resp_id] = new_ffe
                    dfe_l[resp_id] = new_dfe

                if N_DFE > 1:
                    d_hist[tid][1:] = d_hist[tid][:-1]
                d_hist[tid][0] = d_hat

                # Periodic update
                if n_symbols[tid] % report_interval == 0 and n_symbols[tid] > 0:
                    mse = mse_acc[tid] / report_interval
                    mse_acc[tid] = 0.0
                    with self.lock:
                        self.rx_ffe[tid] = rx_ffe_l[tid].copy()
                        self.dfe_taps[tid] = dfe_l[tid].copy()
                        self.mse_history[tid].append(mse)
                        self.n_symbols[tid] = n_symbols[tid]
                        self.current_mse[tid] = mse
                    self._emit('UPDATE', lane=tid, mse=mse,
                               symbols=n_symbols[tid])

        ser.close()
        self._emit('LOG', msg="Equalization complete.")
        self._emit('STOPPED')


# ═══════════════════════════════════════════════════════════════════════
# GUI Application
# ═══════════════════════════════════════════════════════════════════════

# ── Color palette ─────────────────────────────────────────────────────
BG           = "#0f1117"
BG_PANEL     = "#181c25"
BG_CARD      = "#1e2330"
BORDER       = "#2a3040"
TEXT         = "#e0e4ec"
TEXT_DIM     = "#6b7280"
ACCENT       = "#3b82f6"
ACCENT_GLOW  = "#60a5fa"
GREEN        = "#22c55e"
RED          = "#ef4444"
AMBER        = "#f59e0b"
LANE_COLORS  = [
    "#3b82f6", "#f97316", "#22c55e", "#ef4444", "#a855f7",
    "#8b5cf6", "#ec4899", "#6b7280", "#84cc16", "#06b6d4",
]

class SerDesGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("SerDes Link Training Dashboard")
        self.root.configure(bg=BG)
        self.root.geometry("1440x900")
        self.root.minsize(1200, 750)

        self.worker = None
        self.gui_queue = queue.Queue()
        self.cmd_queue = queue.Queue()
        self.n_tasks = 10
        self.selected_lane = tk.IntVar(value=0)
        self.selected_lane.trace_add("write",
                                     lambda *_: self._redraw_eye())
        self.running = False

        # Style
        style = ttk.Style()
        style.theme_use('clam')
        style.configure("Dark.TFrame", background=BG)
        style.configure("Card.TFrame", background=BG_CARD)
        style.configure("Dark.TLabel", background=BG, foreground=TEXT,
                        font=("Consolas", 10))
        style.configure("Title.TLabel", background=BG, foreground=TEXT,
                        font=("Segoe UI", 16, "bold"))
        style.configure("CardTitle.TLabel", background=BG_CARD,
                        foreground=ACCENT_GLOW, font=("Segoe UI", 11, "bold"))
        style.configure("Metric.TLabel", background=BG_CARD, foreground=TEXT,
                        font=("Consolas", 10))
        style.configure("MetricVal.TLabel", background=BG_CARD,
                        foreground=ACCENT_GLOW, font=("Consolas", 12, "bold"))
        style.configure("Reset.TButton", font=("Segoe UI", 9, "bold"))
        style.configure("Start.TButton", font=("Segoe UI", 10, "bold"),
                        foreground="white", background=GREEN)
        style.configure("Stop.TButton", font=("Segoe UI", 10, "bold"),
                        foreground="white", background=RED)

        self._build_ui()
        self._bind_keys()
        self._poll_queue()

    # ── Build the interface ───────────────────────────────────────
    def _build_ui(self):
        # ── Top bar ──
        top = tk.Frame(self.root, bg=BG_PANEL, pady=8, padx=12)
        top.pack(fill=tk.X)

        ttk.Label(top, text="SerDes Link Training",
                  style="Title.TLabel").pack(side=tk.LEFT, padx=(0, 20))

        tk.Label(top, text="Port:", bg=BG_PANEL, fg=TEXT,
                 font=("Segoe UI", 10)).pack(side=tk.LEFT)
        self.port_var = tk.StringVar(value="COM5")
        tk.Entry(top, textvariable=self.port_var, width=8, bg=BG_CARD,
                 fg=TEXT, insertbackground=TEXT, font=("Consolas", 10),
                 relief=tk.FLAT).pack(side=tk.LEFT, padx=(4, 12))

        tk.Label(top, text="Baud:", bg=BG_PANEL, fg=TEXT,
                 font=("Segoe UI", 10)).pack(side=tk.LEFT)
        self.baud_var = tk.StringVar(value="19200")
        tk.Entry(top, textvariable=self.baud_var, width=7, bg=BG_CARD,
                 fg=TEXT, insertbackground=TEXT, font=("Consolas", 10),
                 relief=tk.FLAT).pack(side=tk.LEFT, padx=(4, 12))

        tk.Label(top, text="Channel:", bg=BG_PANEL, fg=TEXT,
                 font=("Segoe UI", 10)).pack(side=tk.LEFT)
        self.channel_var = tk.StringVar(value="channel_taps.txt")
        tk.Entry(top, textvariable=self.channel_var, width=22, bg=BG_CARD,
                 fg=TEXT, insertbackground=TEXT, font=("Consolas", 10),
                 relief=tk.FLAT).pack(side=tk.LEFT, padx=(4, 4))
        tk.Button(top, text="...", command=self._browse_channel,
                  bg=BG_CARD, fg=TEXT, relief=tk.FLAT, width=3,
                  font=("Segoe UI", 9)).pack(side=tk.LEFT, padx=(0, 12))

        tk.Label(top, text="Lanes:", bg=BG_PANEL, fg=TEXT,
                 font=("Segoe UI", 10)).pack(side=tk.LEFT)
        self.tasks_var = tk.StringVar(value="10")
        tk.Entry(top, textvariable=self.tasks_var, width=3, bg=BG_CARD,
                 fg=TEXT, insertbackground=TEXT, font=("Consolas", 10),
                 relief=tk.FLAT).pack(side=tk.LEFT, padx=(4, 16))

        self.start_btn = tk.Button(top, text="▶  Start", command=self._start,
                                   bg=GREEN, fg="white", relief=tk.FLAT,
                                   font=("Segoe UI", 10, "bold"), padx=16)
        self.start_btn.pack(side=tk.LEFT, padx=4)

        self.stop_btn = tk.Button(top, text="■  Stop", command=self._stop,
                                  bg=RED, fg="white", relief=tk.FLAT,
                                  font=("Segoe UI", 10, "bold"), padx=16,
                                  state=tk.DISABLED)
        self.stop_btn.pack(side=tk.LEFT, padx=4)

        # ── Main content area ──
        content = tk.Frame(self.root, bg=BG)
        content.pack(fill=tk.BOTH, expand=True, padx=8, pady=(4, 0))

        # Left column: eye diagrams
        left = tk.Frame(content, bg=BG)
        left.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # Lane selector
        sel_frame = tk.Frame(left, bg=BG)
        sel_frame.pack(fill=tk.X, pady=(0, 2))
        tk.Label(sel_frame, text="Eye Diagram — Lane:",
                 bg=BG, fg=TEXT_DIM,
                 font=("Segoe UI", 10)).pack(side=tk.LEFT)
        for i in range(10):
            rb = tk.Radiobutton(sel_frame, text=str(i),
                                variable=self.selected_lane, value=i,
                                bg=BG, fg=LANE_COLORS[i],
                                selectcolor=BG_CARD, activebackground=BG,
                                activeforeground=LANE_COLORS[i],
                                font=("Consolas", 10, "bold"),
                                indicatoron=0, width=3, relief=tk.FLAT,
                                bd=0)
            rb.pack(side=tk.LEFT, padx=1)

        # Eye diagram figure
        self.eye_fig = Figure(figsize=(7, 3.2), dpi=100, facecolor=BG_PANEL)
        self.eye_ax_before = self.eye_fig.add_subplot(121)
        self.eye_ax_after  = self.eye_fig.add_subplot(122)
        self._style_ax(self.eye_ax_before, "Before Equalization")
        self._style_ax(self.eye_ax_after, "After Equalization")
        self.eye_fig.tight_layout(pad=1.5)
        eye_canvas = FigureCanvasTkAgg(self.eye_fig, master=left)
        eye_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self.eye_canvas = eye_canvas

        # MSE convergence figure
        tk.Label(left, text="MSE Convergence — All Lanes",
                 bg=BG, fg=TEXT_DIM,
                 font=("Segoe UI", 10)).pack(anchor=tk.W, pady=(6, 0))
        self.mse_fig = Figure(figsize=(7, 2.6), dpi=100, facecolor=BG_PANEL)
        self.mse_ax = self.mse_fig.add_subplot(111)
        self._style_ax(self.mse_ax, "")
        self.mse_ax.set_xlabel("Symbol (×100)", color=TEXT_DIM, fontsize=9)
        self.mse_ax.set_ylabel("MSE", color=TEXT_DIM, fontsize=9)
        self.mse_fig.tight_layout(pad=1.5)
        mse_canvas = FigureCanvasTkAgg(self.mse_fig, master=left)
        mse_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self.mse_canvas = mse_canvas

        # Right column: lane dashboard + log
        right = tk.Frame(content, bg=BG, width=380)
        right.pack(side=tk.RIGHT, fill=tk.Y, padx=(8, 0))
        right.pack_propagate(False)

        tk.Label(right, text="Lane Status   (press 0-9 to reset)",
                 bg=BG, fg=TEXT_DIM,
                 font=("Segoe UI", 10)).pack(anchor=tk.W)

        # Lane cards container (scrollable)
        lanes_outer = tk.Frame(right, bg=BG)
        lanes_outer.pack(fill=tk.BOTH, expand=True, pady=(2, 4))

        lanes_canvas = tk.Canvas(lanes_outer, bg=BG, highlightthickness=0)
        lanes_scrollbar = ttk.Scrollbar(lanes_outer, orient=tk.VERTICAL,
                                        command=lanes_canvas.yview)
        self.lanes_frame = tk.Frame(lanes_canvas, bg=BG)

        self.lanes_frame.bind(
            "<Configure>",
            lambda e: lanes_canvas.configure(
                scrollregion=lanes_canvas.bbox("all")))
        lanes_canvas.create_window((0, 0), window=self.lanes_frame,
                                   anchor=tk.NW)
        lanes_canvas.configure(yscrollcommand=lanes_scrollbar.set)

        lanes_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        lanes_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        self.lane_widgets = []
        for i in range(10):
            card = self._build_lane_card(self.lanes_frame, i)
            self.lane_widgets.append(card)

        # Event log
        tk.Label(right, text="Event Log", bg=BG, fg=TEXT_DIM,
                 font=("Segoe UI", 10)).pack(anchor=tk.W, pady=(4, 2))
        self.log_text = scrolledtext.ScrolledText(
            right, height=8, bg=BG_CARD, fg=TEXT, insertbackground=TEXT,
            font=("Consolas", 9), relief=tk.FLAT, wrap=tk.WORD, bd=0)
        self.log_text.pack(fill=tk.X)
        self.log_text.configure(state=tk.DISABLED)

        # Status bar
        self.status_var = tk.StringVar(value="Ready — configure and press Start")
        tk.Label(self.root, textvariable=self.status_var, bg=BG_PANEL,
                 fg=TEXT_DIM, font=("Consolas", 9),
                 anchor=tk.W, padx=12, pady=4).pack(fill=tk.X, side=tk.BOTTOM)

    def _build_lane_card(self, parent, lane_id):
        """Build a compact status card for one lane."""
        color = LANE_COLORS[lane_id]
        card = tk.Frame(parent, bg=BG_CARD, highlightbackground=BORDER,
                        highlightthickness=1, padx=6, pady=4)
        card.pack(fill=tk.X, pady=1)

        # Header row
        hdr = tk.Frame(card, bg=BG_CARD)
        hdr.pack(fill=tk.X)

        tk.Label(hdr, text=f"Lane {lane_id}", bg=BG_CARD, fg=color,
                 font=("Consolas", 10, "bold")).pack(side=tk.LEFT)

        # Status indicator
        status_lbl = tk.Label(hdr, text="● IDLE", bg=BG_CARD, fg=TEXT_DIM,
                              font=("Consolas", 9))
        status_lbl.pack(side=tk.RIGHT, padx=(8, 0))

        reset_btn = tk.Button(hdr, text="Reset", bg=AMBER, fg="black",
                              relief=tk.FLAT, font=("Segoe UI", 8, "bold"),
                              padx=8, pady=0,
                              command=lambda lid=lane_id: self._reset_lane(lid))
        reset_btn.pack(side=tk.RIGHT, padx=4)

        # Metrics row
        met = tk.Frame(card, bg=BG_CARD)
        met.pack(fill=tk.X, pady=(2, 0))

        mse_lbl = tk.Label(met, text="MSE: —", bg=BG_CARD, fg=TEXT,
                           font=("Consolas", 9))
        mse_lbl.pack(side=tk.LEFT, padx=(0, 10))

        ffe_lbl = tk.Label(met, text="FFE: —", bg=BG_CARD, fg=TEXT,
                           font=("Consolas", 9))
        ffe_lbl.pack(side=tk.LEFT, padx=(0, 10))

        sym_lbl = tk.Label(met, text="Sym: 0", bg=BG_CARD, fg=TEXT_DIM,
                           font=("Consolas", 9))
        sym_lbl.pack(side=tk.LEFT, padx=(0, 10))

        rst_lbl = tk.Label(met, text="Rst: 0", bg=BG_CARD, fg=TEXT_DIM,
                           font=("Consolas", 9))
        rst_lbl.pack(side=tk.LEFT)

        return {
            'card': card, 'status': status_lbl,
            'mse': mse_lbl, 'ffe': ffe_lbl,
            'sym': sym_lbl, 'rst': rst_lbl,
            'reset_btn': reset_btn,
        }

    def _style_ax(self, ax, title):
        ax.set_facecolor(BG_PANEL)
        ax.set_title(title, color=TEXT_DIM, fontsize=10, pad=6)
        ax.tick_params(colors=TEXT_DIM, labelsize=8)
        for spine in ax.spines.values():
            spine.set_color(BORDER)
        ax.grid(True, alpha=0.15, color=TEXT_DIM)

    # ── Keyboard bindings ─────────────────────────────────────────
    def _bind_keys(self):
        for i in range(10):
            self.root.bind(str(i),
                           lambda e, lid=i: self._reset_lane(lid))

    # ── Actions ───────────────────────────────────────────────────
    def _browse_channel(self):
        path = filedialog.askopenfilename(
            filetypes=[("Text files", "*.txt"), ("All files", "*.*")])
        if path:
            self.channel_var.set(path)

    def _reset_lane(self, lane_id):
        if self.running and lane_id < self.n_tasks:
            self.cmd_queue.put(('RESET', lane_id))
            self._log(f"Reset queued for lane {lane_id}")

    def _start(self):
        if self.running:
            return

        port = self.port_var.get().strip()
        baud = int(self.baud_var.get().strip())
        channel = self.channel_var.get().strip()
        self.n_tasks = int(self.tasks_var.get().strip())

        if not channel:
            self._log("ERROR: No channel file specified")
            return

        self.gui_queue = queue.Queue()
        self.cmd_queue = queue.Queue()

        self.worker = BridgeWorker(port, baud, channel, self.n_tasks,
                                   self.gui_queue, self.cmd_queue)

        # Reset GUI state
        for i, w in enumerate(self.lane_widgets):
            w['mse'].config(text="MSE: —")
            w['ffe'].config(text="FFE: —")
            w['sym'].config(text="Sym: 0")
            w['rst'].config(text="Rst: 0")
            vis = tk.NORMAL if i < self.n_tasks else tk.DISABLED
            w['reset_btn'].config(state=vis)
            st_text = "● IDLE" if i < self.n_tasks else "● OFF"
            st_color = TEXT_DIM
            w['status'].config(text=st_text, fg=st_color)

        self.worker.start()
        self.running = True
        self.start_btn.config(state=tk.DISABLED)
        self.stop_btn.config(state=tk.NORMAL)
        self.status_var.set("Connecting...")
        self._log("Starting bridge worker...")

    def _stop(self):
        if self.worker:
            self.worker.stop()
        self.running = False
        self.start_btn.config(state=tk.NORMAL)
        self.stop_btn.config(state=tk.DISABLED)
        self.status_var.set("Stopped")
        self._log("Stopped.")

    # ── Logging ───────────────────────────────────────────────────
    def _log(self, msg):
        ts = datetime.now().strftime("%H:%M:%S")
        self.log_text.configure(state=tk.NORMAL)
        self.log_text.insert(tk.END, f"[{ts}] {msg}\n")
        self.log_text.see(tk.END)
        self.log_text.configure(state=tk.DISABLED)

    # ── Queue polling ─────────────────────────────────────────────
    def _poll_queue(self):
        redraw_eye = False
        redraw_mse = False

        try:
            while True:
                event_type, data = self.gui_queue.get_nowait()

                if event_type == 'UPDATE':
                    tid = data['lane']
                    mse = data['mse']
                    syms = data['symbols']

                    if tid < len(self.lane_widgets):
                        w = self.lane_widgets[tid]
                        w['mse'].config(text=f"MSE: {mse:.4f}")
                        w['sym'].config(text=f"Sym: {syms}")
                        w['status'].config(text="● TRAINING", fg=GREEN)

                        with self.worker.lock:
                            main_tap = self.worker.rx_ffe[tid][RX_FFE_PRE]
                        w['ffe'].config(text=f"FFE: {main_tap:+.3f}")

                    redraw_mse = True
                    if tid == self.selected_lane.get():
                        redraw_eye = True

                    total = sum(self.worker.n_symbols[:self.n_tasks])
                    self.status_var.set(
                        f"Running — {total:,} total symbols processed")

                elif event_type == 'RESET':
                    tid = data['lane']
                    rc = data['count']
                    self._log(f"*** LANE {tid} RESET (#{rc}) ***")
                    if tid < len(self.lane_widgets):
                        w = self.lane_widgets[tid]
                        w['rst'].config(text=f"Rst: {rc}")
                        w['status'].config(text="● RESET", fg=AMBER)
                        # Flash effect: schedule revert after 1.5s
                        self.root.after(1500, lambda t=tid:
                            self.lane_widgets[t]['status'].config(
                                text="● TRAINING", fg=GREEN)
                            if self.running else None)
                    redraw_mse = True
                    redraw_eye = True

                elif event_type == 'LOG':
                    self._log(data['msg'])

                elif event_type == 'ERROR':
                    self._log(f"ERROR: {data['msg']}")
                    self._stop()

                elif event_type == 'STARTED':
                    self.status_var.set("Running...")
                    self._log("Bridge started — press 0-9 to reset lanes")

                elif event_type == 'STOPPED':
                    self._stop()
                    self._log("Bridge finished.")
                    # Final redraw
                    redraw_eye = True
                    redraw_mse = True

        except queue.Empty:
            pass

        if redraw_mse:
            self._redraw_mse()
        if redraw_eye:
            self._redraw_eye()

        self.root.after(200, self._poll_queue)

    # ── Plot updates ──────────────────────────────────────────────
    def _redraw_eye(self):
        if not self.worker:
            return
        with self.worker.lock:
            post_adc = self.worker.post_adc
            phase = self.worker.sample_phase
            lane = self.selected_lane.get()
            if lane >= self.n_tasks or post_adc is None:
                return
            eq_out = self.worker.eq_output[lane].copy()

        n_traces = min(300, N_BIT - 10)
        start_sym = TX_FFE_PRE + 5
        t_ui = np.linspace(-1, 1, 2 * OSF)

        for ax, signal, color, title in [
            (self.eye_ax_before, post_adc, "#4488cc", "Before Equalization"),
            (self.eye_ax_after, eq_out, LANE_COLORS[lane], "After Equalization"),
        ]:
            ax.clear()
            self._style_ax(ax, f"{title}  (Lane {lane})")
            ax.set_xlim(-1, 1)

            segments = []
            for i in range(start_sym, start_sym + n_traces):
                center = i * OSF + phase
                s = center - OSF
                e = center + OSF
                if s >= 0 and e < len(signal):
                    trace = signal[s:e]
                    if len(trace) == 2 * OSF:
                        seg = np.column_stack([t_ui, trace])
                        segments.append(seg)

            if segments:
                lc = LineCollection(segments, colors=color,
                                    linewidths=0.5, alpha=0.08)
                ax.add_collection(lc)
                # Auto-scale y
                all_y = np.concatenate([s[:, 1] for s in segments])
                ymin, ymax = all_y.min(), all_y.max()
                margin = (ymax - ymin) * 0.1 or 0.5
                ax.set_ylim(ymin - margin, ymax + margin)

            ax.axvline(0, color=RED, linestyle='--', alpha=0.4, linewidth=0.8)

        self.eye_fig.tight_layout(pad=1.5)
        self.eye_canvas.draw_idle()

    def _redraw_mse(self):
        if not self.worker:
            return

        self.mse_ax.clear()
        self._style_ax(self.mse_ax, "")
        self.mse_ax.set_xlabel("Symbol (×100)", color=TEXT_DIM, fontsize=9)
        self.mse_ax.set_ylabel("MSE", color=TEXT_DIM, fontsize=9)

        with self.worker.lock:
            for tid in range(self.n_tasks):
                hist = list(self.worker.mse_history[tid])
                if hist:
                    self.mse_ax.semilogy(
                        hist, linewidth=1.2,
                        color=LANE_COLORS[tid], alpha=0.85,
                        label=f"L{tid}")

        self.mse_ax.legend(fontsize=7, ncol=5, loc='upper right',
                           framealpha=0.3, labelcolor=TEXT)
        self.mse_fig.tight_layout(pad=1.5)
        self.mse_canvas.draw_idle()


# ═══════════════════════════════════════════════════════════════════════
# Entry point
# ═══════════════════════════════════════════════════════════════════════

if __name__ == '__main__':
    root = tk.Tk()
    app = SerDesGUI(root)
    root.mainloop()