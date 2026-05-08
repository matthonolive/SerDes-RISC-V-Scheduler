#!/usr/bin/env python3
"""
laptop_bridge.py  —  Laptop-side channel simulation + UART bridge to FPGA equalizer

Usage:
    # Normal FPGA mode (10 lanes)
    python laptop_bridge.py --port COM5 --channel channel_taps.txt --eye

    # Fewer lanes for testing
    python laptop_bridge.py --port COM5 --channel channel_taps.txt --tasks 3

    # Sim mode — software-only LMS, no FPGA needed
    python laptop_bridge.py --channel channel_taps.txt --sim --eye

  While running, press 0-9 to soft-reset the corresponding lane.

Dependencies:
    pip install pyserial numpy matplotlib
"""

import argparse
import struct
import time
import sys
import threading
import numpy as np

# ── Equalizer dimensions (must match FPGA main.c) ─────────────────────
RX_FFE_PRE  = 3
RX_FFE_POST = 10
RX_FFE_LEN  = RX_FFE_PRE + 1 + RX_FFE_POST   # 14
N_DFE       = 1

# ── Sync bytes ─────────────────────────────────────────────────────────
SYNC_TX = 0xAA   # laptop → FPGA
SYNC_RX = 0x55   # FPGA → laptop

# ── Command byte values (laptop → FPGA) ───────────────────────────────
CMD_NORMAL = 0x00
CMD_RESET  = 0x01

# ── Status byte values (FPGA → laptop) ────────────────────────────────
STATUS_OK    = 0x00
STATUS_RESET = 0x01

# ── SerDes simulation parameters ──────────────────────────────────────
OSF         = 16
N_BIT       = 256
ADC_BITS    = 5
TX_FFE_PRE  = 0
TX_FFE_POST = 0
TX_FFE_LEN  = TX_FFE_PRE + 1 + TX_FFE_POST

# ── LMS parameters (for --sim mode) ──────────────────────────────────
MU_FFE  = 0.01
MU_DFE  = 0.00


# ═══════════════════════════════════════════════════════════════════════
# Keyboard listener  (press 0-9 to reset a lane)
# ═══════════════════════════════════════════════════════════════════════

reset_pending = {}   # lane_id → True  (consumed by main loop)
_kb_stop = threading.Event()


def _keyboard_listener():
    """Background thread: read single keypresses, set reset flags."""
    try:
        # ── Windows ──
        import msvcrt
        while not _kb_stop.is_set():
            if msvcrt.kbhit():
                ch = msvcrt.getch().decode(errors='ignore')
                if ch in '0123456789':
                    lane = int(ch)
                    reset_pending[lane] = True
                    print(f"\n>>> Reset queued for lane {lane}  "
                          f"(will take effect on next packet)\n")
            _kb_stop.wait(0.05)
    except ImportError:
        # ── Unix / Linux / macOS ──
        import tty, termios, select
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            while not _kb_stop.is_set():
                rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
                if rlist:
                    ch = sys.stdin.read(1)
                    if ch in '0123456789':
                        lane = int(ch)
                        reset_pending[lane] = True
                        print(f"\n>>> Reset queued for lane {lane}  "
                              f"(will take effect on next packet)\n")
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def start_keyboard_listener():
    t = threading.Thread(target=_keyboard_listener, daemon=True)
    t.start()
    return t


def stop_keyboard_listener():
    _kb_stop.set()


# ═══════════════════════════════════════════════════════════════════════
# Channel simulation (laptop side)
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
    rng = np.random.default_rng(42)
    return rng.choice([-1.0, 1.0], size=n_bit)


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
    best_phase = 0
    best_metric = -1.0
    for phase in range(osf):
        samples = signal[phase::osf][:n_cdr]
        metric = np.mean(np.abs(samples))
        if metric > best_metric:
            best_metric = metric
            best_phase = phase
    return best_phase


def build_signal_chain(channel_file):
    h_fir = load_channel(channel_file)
    print(f"Loaded channel: {len(h_fir)} taps from {channel_file}")

    bits = generate_prbs(N_BIT)
    bits_osf = upsample(bits, OSF)

    tx_ffe = np.zeros(TX_FFE_LEN)
    tx_ffe[TX_FFE_PRE] = 1.0
    tx_signal = apply_tx_ffe(bits_osf, tx_ffe, OSF)

    post_channel = apply_channel_fir(tx_signal, h_fir)
    post_adc = adc_quantize(post_channel, ADC_BITS)

    sample_phase = simple_cdr(post_adc, OSF)
    print(f"CDR sample phase: {sample_phase}")

    return post_adc, bits, sample_phase


# ═══════════════════════════════════════════════════════════════════════
# Packet I/O  (with command byte)
# ═══════════════════════════════════════════════════════════════════════

def build_error_packet(task_id, cmd, error, rx_buf, d_hist):
    """Build: SYNC_TX | task_id(u8) | cmd(u8) | error(f32) | rx_buf | d_hist"""
    pkt = bytearray([SYNC_TX, task_id & 0xFF, cmd & 0xFF])
    pkt += struct.pack('<f', error)
    for v in rx_buf:
        pkt += struct.pack('<f', v)
    for v in d_hist:
        pkt += struct.pack('<f', v)
    return bytes(pkt)


def send_error_packet(ser, task_id, cmd, error, rx_buf, d_hist, debug=False):
    pkt = build_error_packet(task_id, cmd, error, rx_buf, d_hist)
    if debug:
        cmd_str = "RESET" if cmd == CMD_RESET else "NORM"
        print(f"  TX [lane {task_id} {cmd_str}] "
              f"({len(pkt)} bytes): {pkt.hex(' ')}")
    ser.write(pkt)
    ser.flush()


def recv_tap_packet(ser, debug=False):
    """Receive: SYNC_RX | task_id(u8) | status(u8) | floats…
    Returns (task_id, status, rx_ffe, dfe)."""

    n_floats = RX_FFE_LEN + N_DFE
    payload_len = n_floats * 4

    # ── Scan for sync byte ────────────────────────────────────
    scan_count = 0
    max_scan = payload_len * 4

    while scan_count < max_scan:
        b = ser.read(1)
        if len(b) == 0:
            raise TimeoutError("Timed out waiting for FPGA sync byte "
                               f"(scanned {scan_count} bytes)")
        scan_count += 1
        if debug:
            print(f"  RX scan [{scan_count:3d}]: 0x{b[0]:02X}"
                  f"{'  ← SYNC!' if b[0] == SYNC_RX else ''}")
        if b[0] == SYNC_RX:
            break
    else:
        raise TimeoutError(f"Never found SYNC_RX (0x{SYNC_RX:02X}) in "
                           f"{max_scan} bytes")

    # ── Read task ID byte ─────────────────────────────────────
    id_byte = ser.read(1)
    if len(id_byte) == 0:
        raise TimeoutError("Timed out reading task-ID byte")
    task_id = id_byte[0]

    # ── Read status byte ──────────────────────────────────────
    st_byte = ser.read(1)
    if len(st_byte) == 0:
        raise TimeoutError("Timed out reading status byte")
    status = st_byte[0]

    if debug:
        status_str = "RESET" if status == STATUS_RESET else "OK"
        print(f"  RX task_id: {task_id}  status: {status_str}")

    # ── Read float payload ────────────────────────────────────
    raw = ser.read(payload_len)
    if debug:
        print(f"  RX payload ({len(raw)}/{payload_len} bytes): "
              f"{raw[:20].hex(' ')}{'...' if len(raw) > 20 else ''}")
    if len(raw) < payload_len:
        raise TimeoutError(f"Short payload: got {len(raw)}/{payload_len} bytes")

    values = struct.unpack(f'<{n_floats}f', raw)
    rx_ffe = np.array(values[:RX_FFE_LEN])
    dfe    = np.array(values[RX_FFE_LEN:])
    return task_id, status, rx_ffe, dfe


# ═══════════════════════════════════════════════════════════════════════
# Listen mode
# ═══════════════════════════════════════════════════════════════════════

def run_listen(port, baud, duration=10):
    import serial
    ser = serial.Serial(port, baud, timeout=0.5)
    print(f"Listening on {port} @ {baud} for {duration}s  (Ctrl-C to stop)")
    print("─" * 60)

    t0 = time.time()
    total = 0
    try:
        while time.time() - t0 < duration:
            data = ser.read(ser.in_waiting or 1)
            if data:
                total += len(data)
                try:
                    text = data.decode('ascii')
                    if text.isprintable() or text.strip():
                        print(f"  TEXT ({len(data):3d}B): {text.rstrip()}")
                    else:
                        print(f"  HEX  ({len(data):3d}B): {data.hex(' ')}")
                except UnicodeDecodeError:
                    print(f"  HEX  ({len(data):3d}B): {data.hex(' ')}")
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
    print("─" * 60)
    print(f"Total received: {total} bytes in {time.time()-t0:.1f}s")


# ═══════════════════════════════════════════════════════════════════════
# Sim mode — software-only LMS (no FPGA)
# ═══════════════════════════════════════════════════════════════════════

def run_sim(channel_file, max_iters=None, verbose=True, show_eye=False):
    post_adc, bits, sample_phase = build_signal_chain(channel_file)
    print("Running software-only LMS (--sim mode)")

    rx_buf   = np.zeros(RX_FFE_LEN)
    d_hist   = np.zeros(N_DFE)
    rx_ffe   = np.zeros(RX_FFE_LEN)
    rx_ffe[RX_FFE_PRE] = 1.0
    dfe_taps = np.zeros(N_DFE)

    n_symbols = 0
    mse_acc = 0.0
    report_interval = 100
    mse_history = []
    eq_output = np.copy(post_adc).astype(float)

    total_samples = len(post_adc)
    start_idx = (TX_FFE_PRE + 1) * OSF

    for pt in range(start_idx, total_samples, 1):
        if pt + sample_phase >= total_samples:
            break
        sample = post_adc[pt + sample_phase]
        rx_buf[1:] = rx_buf[:-1]
        rx_buf[0] = sample
        y_ffe = np.dot(rx_ffe, rx_buf)
        y = y_ffe - np.dot(dfe_taps, d_hist)
        eq_output[pt + sample_phase] = y
        d_hat = 1.0 if y >= 0.0 else -1.0
        sym_idx = pt // 1
        desired = bits[sym_idx] if sym_idx < N_BIT else d_hat
        error = desired - y
        mse_acc += error * error
        n_symbols += 1
        rx_ffe  += MU_FFE * error * rx_buf
        dfe_taps -= MU_DFE * error * d_hist
        if N_DFE > 1:
            d_hist[1:] = d_hist[:-1]
        d_hist[0] = d_hat
        if verbose and n_symbols % report_interval == 0:
            mse = mse_acc / report_interval
            mse_history.append(mse)
            mse_acc = 0.0
            print(f"[{n_symbols:5d}] MSE={mse:.6f}  "
                  f"FFE_main={rx_ffe[RX_FFE_PRE]:+.4f}  "
                  f"DFE[0]={dfe_taps[0]:+.4f}")
        if max_iters and n_symbols >= max_iters:
            break

    print(f"\nDone: {n_symbols} symbols processed (software LMS)")
    print(f"Final RX FFE taps: {rx_ffe}")
    print(f"Final DFE taps:    {dfe_taps}")

    if show_eye and n_symbols > 100:
        plot_eye_diagrams(post_adc, eq_output, sample_phase, OSF, bits)
        if mse_history:
            plot_convergence(mse_history)
    return rx_ffe, dfe_taps


# ═══════════════════════════════════════════════════════════════════════
# FPGA bridge mode  (keyboard-triggered resets)
# ═══════════════════════════════════════════════════════════════════════

def drain_startup(ser, wait=1.0):
    ser.reset_input_buffer()
    time.sleep(wait)
    startup_text = b""
    while ser.in_waiting:
        startup_text += ser.read(ser.in_waiting)
        time.sleep(0.1)
    if startup_text:
        print(f"FPGA startup: {startup_text.decode(errors='replace').strip()}")
    else:
        print("(no startup text from FPGA)")


def reset_lane_state(rx_buf, d_hist, rx_ffe, dfe_taps, eq_output,
                     mse_acc, n_symbols, tid, post_adc):
    """Clear laptop-side state for a lane that was just reset."""
    rx_buf[tid][:] = 0.0
    d_hist[tid][:] = 0.0
    rx_ffe[tid][:] = 0.0
    rx_ffe[tid][RX_FFE_PRE] = 1.0
    dfe_taps[tid][:] = 0.0
    eq_output[tid] = np.copy(post_adc).astype(float)
    mse_acc[tid] = 0.0
    n_symbols[tid] = 0


def run_bridge(port, baud, channel_file, n_tasks=10, max_iters=None,
               verbose=True, show_eye=False, debug=False):
    import serial

    post_adc, bits, sample_phase = build_signal_chain(channel_file)

    print(f"Opening {port} @ {baud}...")
    ser = serial.Serial(port, baud, timeout=2.0)
    drain_startup(ser, wait=1.5)
    print(f"Serial open: {port} @ {baud}")

    tx_pkt_len = 1 + 1 + 1 + (1 + RX_FFE_LEN + N_DFE) * 4  # SYNC+ID+CMD+floats
    rx_pkt_len = 1 + 1 + 1 + (RX_FFE_LEN + N_DFE) * 4        # SYNC+ID+STATUS+floats
    print(f"Packet sizes — TX: {tx_pkt_len} bytes, "
          f"expected RX: {rx_pkt_len} bytes")
    print(f"Number of FPGA lane tasks: {n_tasks}")
    print(f"Starting equalization loop...")
    print(f"  >>> Press 0-9 to soft-reset a lane <<<")
    if debug:
        print("(debug mode: showing raw bytes)")
    print()

    # ── Start keyboard listener ──────────────────────────────
    start_keyboard_listener()

    # ── Per-lane state ────────────────────────────────────────
    rx_buf   = [np.zeros(RX_FFE_LEN) for _ in range(n_tasks)]
    d_hist   = [np.zeros(N_DFE)      for _ in range(n_tasks)]
    rx_ffe   = [np.zeros(RX_FFE_LEN) for _ in range(n_tasks)]
    dfe_taps = [np.zeros(N_DFE)      for _ in range(n_tasks)]
    for t in range(n_tasks):
        rx_ffe[t][RX_FFE_PRE] = 1.0

    n_symbols  = [0] * n_tasks
    mse_acc    = [0.0] * n_tasks
    report_interval = 100
    mse_history    = [[] for _ in range(n_tasks)]
    reset_counts   = [0] * n_tasks

    eq_output = [np.copy(post_adc).astype(float) for _ in range(n_tasks)]

    total_samples = len(post_adc)
    start_idx = (TX_FFE_PRE + 1) * OSF

    try:
      for pt in range(start_idx, total_samples, 1):
        if pt + sample_phase >= total_samples:
            break

        sample = post_adc[pt + sample_phase]

        for tid in range(n_tasks):
            # Shift sample into this lane's FFE buffer
            rx_buf[tid][1:] = rx_buf[tid][:-1]
            rx_buf[tid][0] = sample

            # Compute equalizer output with current FPGA taps
            y_ffe = np.dot(rx_ffe[tid], rx_buf[tid])
            y = y_ffe - np.dot(dfe_taps[tid], d_hist[tid])
            eq_output[tid][pt + sample_phase] = y

            d_hat = 1.0 if y >= 0.0 else -1.0

            sym_idx = pt // 1
            desired = bits[sym_idx] if sym_idx < N_BIT else d_hat

            error = desired - y
            mse_acc[tid] += error * error
            n_symbols[tid] += 1

            # ── Check if user pressed this lane's key ─────────
            cmd = CMD_NORMAL
            if reset_pending.pop(tid, False):
                cmd = CMD_RESET

            if debug:
                print(f"--- Lane {tid}  Symbol {n_symbols[tid]} ---")

            # ── Send to FPGA (with cmd byte) ──────────────────
            send_error_packet(ser, tid, cmd, float(error),
                              rx_buf[tid].tolist(), d_hist[tid].tolist(),
                              debug=debug)

            # ── Receive updated taps from FPGA ────────────────
            try:
                resp_id, status, new_ffe, new_dfe = \
                    recv_tap_packet(ser, debug=debug)
            except TimeoutError as e:
                print(f"\nTimeout at lane {tid}, "
                      f"symbol {n_symbols[tid]}: {e}")
                leftover = ser.read(ser.in_waiting) if ser.in_waiting \
                    else b""
                if leftover:
                    print(f"  Leftover ({len(leftover)} bytes): "
                          f"{leftover[:40].hex(' ')}")
                else:
                    print("  (serial buffer empty)")
                ser.close()
                stop_keyboard_listener()
                return

            # ── Handle reset confirmation from FPGA ───────────
            if status == STATUS_RESET:
                reset_counts[resp_id] += 1
                print(f"\n*** LANE {resp_id} RESET "
                      f"(reset #{reset_counts[resp_id]}) "
                      f"at symbol {n_symbols[resp_id]} ***\n")
                reset_lane_state(rx_buf, d_hist, rx_ffe, dfe_taps,
                                 eq_output, mse_acc, n_symbols,
                                 resp_id, post_adc)

            rx_ffe[resp_id]   = new_ffe
            dfe_taps[resp_id] = new_dfe

            if debug:
                print(f"  Got taps [lane {resp_id}]: "
                      f"FFE_main={new_ffe[RX_FFE_PRE]:+.6f}  "
                      f"DFE[0]={new_dfe[0]:+.6f}")

            # Update decision history
            if N_DFE > 1:
                d_hist[tid][1:] = d_hist[tid][:-1]
            d_hist[tid][0] = d_hat

            # ── Periodic reporting ────────────────────────────
            if verbose and n_symbols[tid] % report_interval == 0:
                mse = mse_acc[tid] / report_interval
                mse_history[tid].append(mse)
                mse_acc[tid] = 0.0
                main_tap = rx_ffe[tid][RX_FFE_PRE]
                print(f"[lane {tid}  {n_symbols[tid]:5d}] "
                      f"MSE={mse:.6f}  "
                      f"FFE_main={main_tap:+.4f}  "
                      f"DFE[0]={dfe_taps[tid][0]:+.4f}")

        if max_iters and n_symbols[0] >= max_iters:
            break

    except KeyboardInterrupt:
        print("\n\nStopped by Ctrl-C")
    finally:
        stop_keyboard_listener()

    # ── Final report ──────────────────────────────────────────
    for tid in range(n_tasks):
        print(f"\nLane {tid}: {n_symbols[tid]} symbols processed  "
              f"({reset_counts[tid]} resets)")
        print(f"  Final RX FFE taps: {rx_ffe[tid]}")
        print(f"  Final DFE taps:    {dfe_taps[tid]}")

    ser.close()

    if show_eye and n_symbols[0] > 100:
        plot_eye_diagrams(post_adc, eq_output[0], sample_phase, OSF, bits,
                          label="Lane 0")
        if any(len(h) > 0 for h in mse_history):
            plot_convergence_multi(mse_history)
    elif show_eye:
        print("Not enough symbols for eye diagram (need >100)")

    return rx_ffe, dfe_taps


# ═══════════════════════════════════════════════════════════════════════
# Plotting
# ═══════════════════════════════════════════════════════════════════════

def plot_eye_diagrams(post_adc, eq_output, sample_phase, osf, bits,
                      label=""):
    import matplotlib.pyplot as plt

    n_traces = min(500, len(bits) - 10)
    start_sym = TX_FFE_PRE + 5

    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    t_ui = np.linspace(-1, 1, 2 * osf)
    title_suffix = f"  ({label})" if label else ""

    ax = axes[0]
    for i in range(start_sym, start_sym + n_traces):
        center = i * osf + sample_phase
        start = center - osf
        end = center + osf
        if start >= 0 and end < len(post_adc):
            trace = post_adc[start:end]
            if len(trace) == 2 * osf:
                ax.plot(t_ui, trace, color='blue', alpha=0.03, linewidth=0.5)
    ax.set_title(f'Eye Diagram — Before Equalization{title_suffix}')
    ax.set_xlabel('Time (UI)')
    ax.set_ylabel('Amplitude')
    ax.set_xlim(-1, 1)
    ax.grid(True, alpha=0.3)
    ax.axvline(0, color='red', linestyle='--', alpha=0.5, label='Sample point')
    ax.legend()

    ax = axes[1]
    for i in range(start_sym, start_sym + min(n_traces, len(eq_output))):
        center = i * osf + sample_phase
        start = center - osf
        end = center + osf
        if start >= 0 and end < len(eq_output):
            trace = eq_output[start:end]
            if len(trace) == 2 * osf:
                ax.plot(t_ui, trace, color='green', alpha=0.03, linewidth=0.5)
    ax.set_title(f'Eye Diagram — After Equalization{title_suffix}')
    ax.set_xlabel('Time (UI)')
    ax.set_ylabel('Amplitude')
    ax.set_xlim(-1, 1)
    ax.grid(True, alpha=0.3)
    ax.axvline(0, color='red', linestyle='--', alpha=0.5, label='Sample point')
    ax.legend()

    plt.tight_layout()
    plt.savefig('eye_diagram.png', dpi=150)
    print("Eye diagram saved to eye_diagram.png")
    plt.show()


def plot_convergence_multi(mse_histories):
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(10, 5))
    colors = ['tab:blue', 'tab:orange', 'tab:green', 'tab:red',
              'tab:purple', 'tab:brown', 'tab:pink', 'tab:gray',
              'tab:olive', 'tab:cyan']
    for tid, hist in enumerate(mse_histories):
        if hist:
            ax.semilogy(hist, linewidth=1.5,
                        color=colors[tid % len(colors)],
                        label=f'Lane {tid}')
    ax.set_title('LMS Convergence — All Lanes')
    ax.set_xlabel('Symbol (×100)')
    ax.set_ylabel('MSE')
    ax.legend(ncol=2, fontsize='small')
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig('convergence.png', dpi=150)
    print("Convergence plot saved to convergence.png")
    plt.show()


def plot_convergence(mse_history):
    plot_convergence_multi([mse_history])


# ═══════════════════════════════════════════════════════════════════════
# CLI
# ═══════════════════════════════════════════════════════════════════════

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Laptop-side channel sim + UART bridge to FPGA equalizer')
    parser.add_argument('--port', default='COM3',
                        help='Serial port (default: COM3)')
    parser.add_argument('--baud', type=int, default=19200,
                        help='Baud rate (default: 19200)')
    parser.add_argument('--channel',
                        help='Path to channel taps file')
    parser.add_argument('--tasks', type=int, default=10,
                        help='Number of lane tasks on FPGA (default: 10)')
    parser.add_argument('--max-iters', type=int, default=None,
                        help='Max symbols to process')
    parser.add_argument('--quiet', action='store_true',
                        help='Suppress periodic MSE output')
    parser.add_argument('--eye', action='store_true',
                        help='Show eye diagrams after equalization')
    parser.add_argument('--debug', action='store_true',
                        help='Hex-dump all serial traffic')
    parser.add_argument('--listen', action='store_true',
                        help='Just listen to FPGA output (no sending)')
    parser.add_argument('--listen-time', type=int, default=10,
                        help='Seconds to listen in --listen mode')
    parser.add_argument('--sim', action='store_true',
                        help='Software-only LMS (no FPGA, no serial port)')

    args = parser.parse_args()

    if args.listen:
        run_listen(args.port, args.baud, duration=args.listen_time)
    elif args.sim:
        if not args.channel:
            parser.error("--sim requires --channel")
        run_sim(args.channel, max_iters=args.max_iters,
                verbose=not args.quiet, show_eye=args.eye)
    else:
        if not args.channel:
            parser.error("FPGA mode requires --channel")
        run_bridge(args.port, args.baud, args.channel,
                   n_tasks=args.tasks,
                   max_iters=args.max_iters, verbose=not args.quiet,
                   show_eye=args.eye, debug=args.debug)