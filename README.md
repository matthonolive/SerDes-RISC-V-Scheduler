# Multi-Lane SerDes Equalizer on NEORV32

A hardware/software co-design that runs a real-time, multi-lane LMS adaptive equalizer
on a [NEORV32](https://github.com/stnolting/neorv32) RISC-V softcore. The FPGA hosts a
small custom scheduler (cooperative + preemptive) that runs one task per SerDes lane,
each maintaining its own FFE/DFE taps. A Python laptop bridge simulates the analog
channel, ADC, and CDR, then drives the equalizer over UART symbol-by-symbol.

![Eye Diagram — Before and After Equalization](eye_diagram.png)

The eye is closed before equalization (left, post-channel + ADC) and clearly open
after the LMS-trained FFE settles (right). The FPGA's only job in the loop is the
per-lane LMS tap update; everything else — channel sim, slicer, error generation,
plotting — runs on the laptop.

---

## Architecture

```
┌────────────────────────────────────┐         UART         ┌────────────────────────────┐
│ Laptop (laptop_bridge.py)          │ ◄──── 19200 baud ──► │ FPGA  —  NEORV32 RV32IMC   │
│                                    │                      │                            │
│ • PRBS gen → TX FFE → channel FIR  │     packet protocol  │ • Custom scheduler         │
│ • 5-bit ADC quantize               │                      │ • 1 task per lane          │
│ • Simple max-|x| CDR               │                      │ • Per-lane FFE/DFE LMS     │
│ • Slicer + error computation       │                      │ • UART round-robin token   │
│ • Eye + MSE plotting               │                      │                            │
└────────────────────────────────────┘                      └────────────────────────────┘
```

The split is deliberate: the FPGA does the part that benefits from being on hardware
(parallel per-lane state, deterministic tap updates), and the laptop does the part
that's easier to iterate on (channel models, plotting, keyboard-driven resets).

---

## What's in the box

### FPGA-side (C + RISC-V assembly)

| File          | Role |
|---------------|------|
| `main.c`      | Lane task body, UART protocol, LMS update |
| `scheduler.c` | Round-robin scheduler, `create_task`, `yield`, `preempt_schedule` |
| `scheduler.h` | Task struct, public API |
| `context.S`   | `context_switch` — saves `ra` + `s0–s11` (52 bytes, ~30 instructions) |
| `timer.S`     | `trap_vector` + `timer_init` — caller-saved interrupt frame, CLINT-driven preemption |
| `start.S`     | Reset vector, BSS clear, CSR setup, `mtvec` install, calls `main` |
| `linker.ld`   | RAM at `0x80000000`, 256 KB, stack at top |
| `Makefile`    | NEORV32 build glue (`rv32imc_zicsr_zifencei`, 64 KB ROM / 32 KB RAM) |

### Laptop-side (Python)

| File              | Role |
|-------------------|------|
| `laptop_bridge.py`| Channel sim + UART bridge + sim mode + plotting |
| `serdes_gui.py`   | Tk dashboard: live eye, live MSE, per-lane status, click-to-reset |
| `channel_taps.txt`| Channel impulse response (~10k FIR taps) |

### Generated artifacts

| File            | Role |
|-----------------|------|
| `main.elf`      | Linked RV32IMC binary |
| `kernel.elf`    | NEORV32 bootloader/kernel image |
| `neorv32_exe.bin` | NEORV32-formatted executable for upload over UART |
| `convergence.png`, `eye_diagram.png` | Output plots from a training run |

---

## Scheduler design

There are two layers, both routed through the same `context_switch`:

**Cooperative (`yield`):** A task explicitly hands control to the scheduler. Only the
13 callee-saved registers (`ra`, `s0–s11`) need saving — 52 bytes, ~30 instructions.
This is what lane tasks call when they're waiting for their UART turn.

**Preemptive (timer interrupt → `preempt_schedule`):** The CLINT machine timer fires
every `TIMER_INTERVAL` cycles. `trap_vector` saves the 18-word caller-saved interrupt
frame (`ra`, `t0–t6`, `a0–a7`, `mepc`, `mstatus` = 72 bytes), then calls into C. The
trick: `preempt_schedule` invokes the *same* `context_switch`, which lays a cooperative
frame on top of the interrupt frame. So `tasks[i].sp` always points to a cooperative
frame — `context_switch` doesn't have to know whether a task was preempted or yielded.

```
Stack of a preempted task:           Stack of a yielded task:

  [task data …]                        [task data …]
  [interrupt frame: 72 B]              [cooperative frame: 52 B] ← tasks[i].sp
  [cooperative frame: 52 B] ← sp
```

**UART safety:** Tasks set `uart_busy = 1` while mid-packet. `preempt_schedule`
returns early if `uart_busy` is set, so a timer tick in the middle of sending a
float can't let another task interleave its bytes onto the wire. A round-robin
`uart_token` rotates UART access in lane order so the laptop can predict which
lane is responding.

---

## UART protocol

Endian: little-endian. Floats: IEEE-754 binary32. Defaults: `19200 8N1`.

### Laptop → FPGA (per symbol, per lane)

```
SYNC_TX (0xAA) │ task_id (u8) │ cmd (u8) │ error (f32)
              │ rx_buf[14] (f32 × 14)    │ d_hist[1] (f32)
```

`cmd = 0x00` runs a normal LMS update. `cmd = 0x01` resets the lane's taps
(and the FPGA echoes `STATUS_RESET` to confirm). 67 bytes total.

### FPGA → Laptop (response)

```
SYNC_RX (0x55) │ task_id (u8) │ status (u8)
              │ rx_ffe[14] (f32 × 14) │ dfe[1] (f32)
```

63 bytes total. `task_id` lets the laptop match the response back to the lane
that produced it.

---

## Build (FPGA side)

You need the [NEORV32](https://github.com/stnolting/neorv32) tree checked out and
a RISC-V GCC toolchain (`riscv32-unknown-elf-gcc` with `rv32imc_zicsr_zifencei`).

```bash
# from the project directory
export NEORV32_HOME=/path/to/neorv32
make exe         # builds main.elf + neorv32_exe.bin
make clean_all all
```

The `Makefile` already pins:
- `MARCH = rv32imc_zicsr_zifencei`
- `EFFORT = -Os`
- 64 KB instruction ROM, 32 KB data RAM (linker `--defsym`)

Upload `neorv32_exe.bin` through the NEORV32 bootloader UART, or flash directly
into IMEM, depending on your bitstream.

---

## Build (Laptop side)

```bash
pip install pyserial numpy matplotlib
```

That's it. `serdes_gui.py` additionally uses `tkinter`, which ships with most
Python distributions.

---

## Running

### Sim mode — no FPGA, no serial port

The fastest way to sanity-check the LMS loop and channel model end-to-end:

```bash
python laptop_bridge.py --channel channel_taps.txt --sim --eye
```

This runs the equalizer entirely in Python and produces `eye_diagram.png` and
`convergence.png` at the end.

### FPGA bridge — full hardware loop

```bash
# 10 lanes (the default; matches MAX_TASKS in scheduler.h)
python laptop_bridge.py --port COM5 --channel channel_taps.txt --eye

# fewer lanes for debugging
python laptop_bridge.py --port COM5 --channel channel_taps.txt --tasks 3

# hex-dump every byte on the wire
python laptop_bridge.py --port COM5 --channel channel_taps.txt --tasks 3 --debug
```

While the loop is running, press `0`–`9` to soft-reset the corresponding lane.
The FPGA acknowledges with a `STATUS_RESET` byte and the laptop wipes its mirror
of that lane's state, so you can watch a lane re-converge from scratch in real
time.

### Listen mode — pure passive sniff

```bash
python laptop_bridge.py --port COM5 --listen --listen-time 5
```

Useful for catching the FPGA's startup banner (`"10-lane SerDes equalizer  —  RR scheduler"`)
and confirming the link is alive before committing to a full run.

### GUI dashboard

```bash
python serdes_gui.py
```

Live eye diagrams, live MSE convergence, per-lane status indicators, and
click-to-reset per lane. Same protocol as `laptop_bridge.py` underneath.

---

## Results

![LMS Convergence — All Lanes](convergence.png)

MSE drops from ~1.0 to ~0.05–0.1 within ~500 symbols and stays in that band.
The remaining floor is the residual ISI plus 5-bit ADC quantization noise —
expected with a single FFE post-cursor count of 10 and `MU_FFE = 0.01`. The
DFE is wired up but disabled (`MU_DFE = 0.00`) by default; flip it on in both
`main.c` and `laptop_bridge.py` to experiment.

The "before" eye is fully closed (left half of the eye-diagram figure above),
and the "after" eye opens cleanly to ±1.0 with crisp crossings — a textbook
result for an LMS-trained linear equalizer on this kind of low-pass-ish channel.

---

## Tunables

All of these have to match on both sides (FPGA and laptop) or the protocol
breaks:

| Constant     | Default | Where |
|--------------|---------|-------|
| `RX_FFE_PRE` | 3       | `main.c`, `laptop_bridge.py`, `serdes_gui.py` |
| `RX_FFE_POST`| 10      | same |
| `N_DFE`      | 1       | same |
| `MU_FFE`     | 0.01    | same |
| `MU_DFE`     | 0.00    | same |
| `NUM_LANES`  | 3 (FPGA) / 10 (laptop default) | `main.c`, `--tasks` flag |
| `MAX_TASKS`  | 10      | `scheduler.h` |
| `STACK_SIZE` | 256 words (1 KB) | `scheduler.h` |
| Baud         | 19200   | `main.c` (`neorv32_uart_setup`), `--baud` flag |

> Note: `main.c` currently sets `NUM_LANES = 3`. Bump it to 10 (and confirm it's
> ≤ `MAX_TASKS`) for a full ten-lane run — and pass `--tasks 10` to the laptop
> bridge to match.

---

## Layout

```
.
├── main.c              # FPGA: lane task + UART protocol
├── scheduler.c         # FPGA: round-robin scheduler
├── scheduler.h
├── context.S           # FPGA: cooperative context switch
├── timer.S             # FPGA: trap vector + CLINT timer
├── start.S             # FPGA: reset / boot
├── linker.ld           # FPGA: memory map
├── Makefile            # FPGA: NEORV32 build wrapper
│
├── laptop_bridge.py    # Laptop: channel sim + UART bridge
├── serdes_gui.py       # Laptop: Tk live dashboard
├── channel_taps.txt    # Channel impulse response
│
├── main.elf            # Build artifact
├── kernel.elf          # NEORV32 bootloader image
├── neorv32_exe.bin     # NEORV32 upload image
├── convergence.png     # Sample run output
└── eye_diagram.png     # Sample run output
```

---

## Troubleshooting

**"Timed out waiting for FPGA sync byte"** — the laptop scanned a few packet-lengths
of bytes without seeing `0x55`. Check baud, check that the FPGA actually printed its
startup banner (`--listen` mode), and confirm `--tasks` matches `NUM_LANES` in `main.c`.

**Lane stuck, never responds** — UART token didn't rotate to it. Usually means a
previous lane took an exception or got stuck without releasing `uart_busy`. Reset
the FPGA.

**MSE flatlines at 1.0** — `rx_ffe[RX_FFE_PRE]` got zeroed somewhere (probably a
runaway tap). Press the lane's number key to reset, or reduce `MU_FFE`.

**Bytes desynced after a long run** — the receiver's sync-byte scan should
re-align within one packet, but if not, restart the bridge. The protocol has no
explicit re-sync sequence beyond the sync byte itself.
