# Multi-Lane SerDes Equalizer on NEORV32

A multi-lane adaptive equalizer for a serial link, partitioned across a laptop
and a [NEORV32](https://github.com/stnolting/neorv32) RV32IMC softcore. The
laptop simulates the analog channel, ADC, and CDR; the FPGA runs a small custom
scheduler that hosts one cooperative task per lane, each performing an LMS tap
update on its own FFE/DFE state. The two sides communicate symbol-by-symbol
over UART.

## Architecture

The system is split along the line where each side has a comparative advantage.
The laptop owns the parts that benefit from rapid iteration — channel models,
slicing, error generation, plotting. The FPGA owns the parts that benefit from
parallel, deterministic state — per-lane tap memory and the LMS update itself.

```
Laptop (Python)                              FPGA (NEORV32, RV32IMC)
─────────────────                            ────────────────────────
 PRBS → TX FFE → channel FIR     UART        scheduler (RR, preemptive)
 5-bit ADC → CDR                 ◄────►      task per lane
 slicer → error                              FFE / DFE LMS update
```

## Scheduler

Two scheduling layers share a single `context_switch` routine:

- **Cooperative** (`yield`). Saves the 13 callee-saved registers
  (`ra`, `s0–s11`, 52 B). Used when a task is waiting for its UART turn.
- **Preemptive** (CLINT machine timer → `trap_vector` → `preempt_schedule`).
  The trap handler saves an 18-word caller-saved interrupt frame, then calls
  `preempt_schedule`, which invokes the same `context_switch`. The cooperative
  frame ends up on top of the interrupt frame, so `tasks[i].sp` always points
  to a cooperative frame regardless of how the task was suspended.

```
Preempted task stack:               Yielded task stack:
  [task data …]                       [task data …]
  [interrupt frame, 72 B]             [cooperative frame, 52 B] ← tasks[i].sp
  [cooperative frame, 52 B] ← sp
```

UART access is serialized by a round-robin token (`uart_token`) and a busy flag
(`uart_busy`). `preempt_schedule` returns early when `uart_busy` is set, which
prevents a timer tick from interleaving bytes from two tasks onto the wire
mid-packet.

## UART protocol

Little-endian, IEEE-754 binary32 floats, `19200 8N1`.

| Direction      | Frame                                                                          | Bytes |
|----------------|--------------------------------------------------------------------------------|-------|
| Laptop → FPGA  | `0xAA │ task_id │ cmd │ error(f32) │ rx_buf[14](f32) │ d_hist[1](f32)`         | 67    |
| FPGA → Laptop  | `0x55 │ task_id │ status │ rx_ffe[14](f32) │ dfe[1](f32)`                      | 63    |

`cmd = 0x01` resets the lane's taps and is acknowledged with `status = 0x01`.
`task_id` echoes back so the laptop can match responses to lanes.

## Files

| File               | Role                                                           |
|--------------------|----------------------------------------------------------------|
| `main.c`           | Lane task body, UART protocol, LMS update                      |
| `scheduler.{c,h}`  | Round-robin scheduler API                                      |
| `context.S`        | `context_switch`, ~30 instructions                             |
| `timer.S`          | `trap_vector`, `timer_init`, CLINT-driven preemption           |
| `start.S`          | Reset vector, BSS clear, CSR setup                             |
| `linker.ld`        | RAM at `0x80000000`, 256 KB, stack at top                      |
| `Makefile`         | NEORV32 build (`rv32imc_zicsr_zifencei`, 64 KB ROM, 32 KB RAM) |
| `laptop_bridge.py` | Channel sim, UART bridge, `--sim` mode, plotting               |
| `serdes_gui.py`    | Tk dashboard: live eye, MSE, per-lane reset                    |
| `channel_taps.txt` | Channel impulse response                                       |

## Build

FPGA side requires the [NEORV32](https://github.com/stnolting/neorv32) tree and
a RISC-V GCC toolchain.

```bash
export NEORV32_HOME=/path/to/neorv32
make exe
```

Laptop side:

```bash
pip install pyserial numpy matplotlib
```

## Running

```bash
# Software-only LMS (no FPGA, no serial port)
python laptop_bridge.py --channel channel_taps.txt --sim --eye

# Hardware loop, N lanes
python laptop_bridge.py --port COM5 --channel channel_taps.txt --tasks 10

# Live dashboard
python serdes_gui.py
```

While the bridge is running, pressing `0`–`9` issues a soft reset to the
corresponding lane; the FPGA acknowledges with `STATUS_RESET` and the laptop
clears its mirror of that lane's state.

## Parameters

The constants below must agree on both sides of the link.

| Constant      | Default | Defined in                       |
|---------------|---------|----------------------------------|
| `RX_FFE_PRE`  | 3       | `main.c`, `laptop_bridge.py`     |
| `RX_FFE_POST` | 10      | same                             |
| `N_DFE`       | 1       | same                             |
| `MU_FFE`      | 0.01    | same                             |
| `MU_DFE`      | 0.00    | same                             |
| `NUM_LANES`   | 3       | `main.c` (compile-time)          |
| `MAX_TASKS`   | 10      | `scheduler.h`                    |
| `STACK_SIZE`  | 256 W   | `scheduler.h`                    |
| Baud          | 19200   | `main.c`, `--baud` flag          |

The DFE is implemented but disabled by default (`MU_DFE = 0`); enable it on
both sides to experiment. `NUM_LANES` in `main.c` and the laptop's `--tasks`
flag must match, and both must be ≤ `MAX_TASKS`.

## Results

On the included channel, MSE drops from ~1.0 to ~0.05–0.1 within roughly 500
symbols and remains in that band. The residual floor reflects the combined
effect of uncompensated ISI and 5-bit ADC quantization. The eye, fully closed
at the slicer input, opens cleanly to ±1.0 after the FFE settles.
