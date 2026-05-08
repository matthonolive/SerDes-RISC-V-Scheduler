# ──────────────────────────────────────────────────────────────
# timer.S  –  timer init + interrupt handler (preemptive layer)
#
# The scheduling logic (uart_busy check, pick_next, context_switch
# call) is now inlined here — no C call to preempt_schedule().
# This eliminates the compiler-generated prologue that redundantly
# saved callee-saved registers before context_switch saved them
# all again.
#
# Stack layout for a preempted task:
#
#   [task stack data …]
#   [interrupt frame: ra,t0-t6,a0-a7,mepc,mstatus = 80 bytes]
#   [cooperative frame: ra,s0-s11 = 52 bytes]  ← tasks[i].sp
#
# Stack layout for a cooperatively-yielded task:
#
#   [task stack data …]
#   [cooperative frame: ra,s0-s11 = 52 bytes]  ← tasks[i].sp
# ──────────────────────────────────────────────────────────────

.global trap_vector
.global timer_init

# CLINT addresses (QEMU virt)
.equ CLINT_BASE,     0x02000000
.equ MTIMECMP_LO,    CLINT_BASE + 0x4000
.equ MTIMECMP_HI,    CLINT_BASE + 0x4004
.equ MTIME_LO,       CLINT_BASE + 0xBFF8
.equ TIMER_INTERVAL, 90000000         # ~1 s at 10 MHz

# sizeof(task_t) = 8
.equ TASK_STRIDE, 8

# ── Interrupt-frame offsets (caller-saved + CSRs) ─────────────
# 18 words = 72 bytes, padded to 80 for 16-byte alignment
.equ IF_RA,       0
.equ IF_T0,       4
.equ IF_T1,       8
.equ IF_T2,      12
.equ IF_T3,      16
.equ IF_T4,      20
.equ IF_T5,      24
.equ IF_T6,      28
.equ IF_A0,      32
.equ IF_A1,      36
.equ IF_A2,      40
.equ IF_A3,      44
.equ IF_A4,      48
.equ IF_A5,      52
.equ IF_A6,      56
.equ IF_A7,      60
.equ IF_MEPC,    64
.equ IF_MSTATUS, 68
.equ IF_SIZE,    80


# ──────────────────────────────────────────────────────────────
# timer_init  –  arm the first timer deadline
# ──────────────────────────────────────────────────────────────
timer_init:
    li   t0, MTIME_LO
    lw   t1, 0(t0)
    li   t2, TIMER_INTERVAL
    add  t1, t1, t2

    li   t0, MTIMECMP_HI
    sw   zero, 0(t0)            # hi = 0 first (avoids spurious)
    li   t0, MTIMECMP_LO
    sw   t1, 0(t0)
    ret


# ──────────────────────────────────────────────────────────────
# trap_vector  –  machine-mode trap entry
# ──────────────────────────────────────────────────────────────
.align 4
trap_vector:
    # ── save interrupt frame (caller-saved only) ──────────────
    addi sp, sp, -IF_SIZE

    sw   ra,  IF_RA(sp)
    sw   t0,  IF_T0(sp)
    sw   t1,  IF_T1(sp)
    sw   t2,  IF_T2(sp)
    sw   t3,  IF_T3(sp)
    sw   t4,  IF_T4(sp)
    sw   t5,  IF_T5(sp)
    sw   t6,  IF_T6(sp)
    sw   a0,  IF_A0(sp)
    sw   a1,  IF_A1(sp)
    sw   a2,  IF_A2(sp)
    sw   a3,  IF_A3(sp)
    sw   a4,  IF_A4(sp)
    sw   a5,  IF_A5(sp)
    sw   a6,  IF_A6(sp)
    sw   a7,  IF_A7(sp)

    csrr t0, mepc
    sw   t0, IF_MEPC(sp)
    csrr t0, mstatus
    sw   t0, IF_MSTATUS(sp)

    # ── identify cause ────────────────────────────────────────
    csrr t0, mcause

    li   t1, 0x80000007        # machine timer interrupt
    bne  t0, t1, .irq_restore  # unknown — return without switching

    # ── acknowledge: mtimecmp = mtime + INTERVAL ──────────────
    li   t0, MTIME_LO
    lw   t1, 0(t0)
    li   t2, TIMER_INTERVAL
    add  t1, t1, t2
    li   t0, MTIMECMP_LO
    sw   t1, 0(t0)

    # ── skip switch if UART is mid-packet ─────────────────────
    la   t0, uart_busy
    lw   t0, 0(t0)
    bnez t0, .irq_restore

    # ── pick next task (round-robin) ──────────────────────────
    la   t0, current_task
    lw   t1, 0(t0)            # t1 = prev
    la   t2, num_tasks
    lw   t2, 0(t2)            # t2 = num_tasks
    addi t3, t1, 1
    blt  t3, t2, 1f
    li   t3, 0                # wrap
1:
    beq  t1, t3, .irq_restore # same task — nothing to do

    sw   t3, 0(t0)            # current_task = next

    # ── context_switch(&tasks[prev].sp, tasks[next].sp) ───────
    la   t0, tasks
    slli t4, t1, 3             # prev * TASK_STRIDE
    add  a0, t0, t4            # a0 = &tasks[prev].sp
    slli t5, t3, 3             # next * TASK_STRIDE
    add  t5, t0, t5
    lw   a1, 0(t5)            # a1 = tasks[next].sp
    call context_switch
    # returns here when this task is switched back in;
    # falls through to .irq_restore to unwind the interrupt frame

.irq_restore:
    # ── restore interrupt frame ───────────────────────────────
    lw   t0, IF_MSTATUS(sp)
    csrw mstatus, t0
    lw   t0, IF_MEPC(sp)
    csrw mepc, t0

    lw   ra,  IF_RA(sp)
    lw   t0,  IF_T0(sp)
    lw   t1,  IF_T1(sp)
    lw   t2,  IF_T2(sp)
    lw   t3,  IF_T3(sp)
    lw   t4,  IF_T4(sp)
    lw   t5,  IF_T5(sp)
    lw   t6,  IF_T6(sp)
    lw   a0,  IF_A0(sp)
    lw   a1,  IF_A1(sp)
    lw   a2,  IF_A2(sp)
    lw   a3,  IF_A3(sp)
    lw   a4,  IF_A4(sp)
    lw   a5,  IF_A5(sp)
    lw   a6,  IF_A6(sp)
    lw   a7,  IF_A7(sp)

    addi sp, sp, IF_SIZE
    mret