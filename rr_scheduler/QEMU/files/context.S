# ──────────────────────────────────────────────────────────────
# context_switch  –  cooperative (callee-saved registers only)
#
#   void context_switch(uint32_t **old_sp, uint32_t *new_sp);
#       a0 = &old_sp    a1 = new_sp
#
# Saves:  ra, s0-s11  =  13 registers  =  52 bytes
# Still used by: schedule() for first launch, and trap_vector
# for preemptive switches.
# ──────────────────────────────────────────────────────────────

.global context_switch
.global task_trampoline
.global yield

# sizeof(task_t) = 8  (uint32_t *sp @ 0, uint8_t active @ 4, +3 pad)
.equ TASK_STRIDE, 8

context_switch:
    addi sp, sp, -52
    sw   ra,   0(sp)
    sw   s0,   4(sp)
    sw   s1,   8(sp)
    sw   s2,  12(sp)
    sw   s3,  16(sp)
    sw   s4,  20(sp)
    sw   s5,  24(sp)
    sw   s6,  28(sp)
    sw   s7,  32(sp)
    sw   s8,  36(sp)
    sw   s9,  40(sp)
    sw   s10, 44(sp)
    sw   s11, 48(sp)

    sw   sp, 0(a0)              # *old_sp = sp

    mv   sp, a1                 # sp = new_sp

    lw   ra,   0(sp)
    lw   s0,   4(sp)
    lw   s1,   8(sp)
    lw   s2,  12(sp)
    lw   s3,  16(sp)
    lw   s4,  20(sp)
    lw   s5,  24(sp)
    lw   s6,  28(sp)
    lw   s7,  32(sp)
    lw   s8,  36(sp)
    lw   s9,  40(sp)
    lw   s10, 44(sp)
    lw   s11, 48(sp)

    addi sp, sp, 52
    ret


# ──────────────────────────────────────────────────────────────
# task_trampoline  –  first-ever entry into a new task
# ──────────────────────────────────────────────────────────────

task_trampoline:
    mv   ra, s1                 # ra = task_exit
    jr   s0                     # jump to task entry


# ──────────────────────────────────────────────────────────────
# yield  –  cooperative context switch  (replaces the C version)
#
# The C yield() had the compiler save callee-saved regs in its
# prologue, then context_switch saved them ALL again — double
# work.  This version inlines the save/restore so every register
# is touched exactly once.
#
# Uses only t-regs for scratch, so no caller state is clobbered
# before the frame is saved.
# ──────────────────────────────────────────────────────────────

yield:
    # ── disable interrupts during switch ──────────────────────
    csrci mstatus, 8           # clear MIE (bit 3)

    # ── pick next task (round-robin) ──────────────────────────
    la   t0, current_task
    lw   t1, 0(t0)            # t1 = prev
    la   t2, num_tasks
    lw   t2, 0(t2)            # t2 = num_tasks
    addi t3, t1, 1            # t3 = candidate next
    blt  t3, t2, 1f
    li   t3, 0                # wrap
1:
    beq  t1, t3, .yield_done  # only one runnable task — skip

    sw   t3, 0(t0)            # current_task = next

    # ── save cooperative frame (13 regs, 52 bytes) ────────────
    addi sp, sp, -52
    sw   ra,   0(sp)
    sw   s0,   4(sp)
    sw   s1,   8(sp)
    sw   s2,  12(sp)
    sw   s3,  16(sp)
    sw   s4,  20(sp)
    sw   s5,  24(sp)
    sw   s6,  28(sp)
    sw   s7,  32(sp)
    sw   s8,  36(sp)
    sw   s9,  40(sp)
    sw   s10, 44(sp)
    sw   s11, 48(sp)

    # ── swap stack pointers ───────────────────────────────────
    # t1 = prev index, t3 = next index  (still live, only sw above)
    la   t0, tasks
    slli t4, t1, 3             # prev * TASK_STRIDE
    add  t4, t0, t4
    sw   sp, 0(t4)             # tasks[prev].sp = sp

    slli t5, t3, 3             # next * TASK_STRIDE
    add  t5, t0, t5
    lw   sp, 0(t5)             # sp = tasks[next].sp

    # ── restore cooperative frame ─────────────────────────────
    lw   ra,   0(sp)
    lw   s0,   4(sp)
    lw   s1,   8(sp)
    lw   s2,  12(sp)
    lw   s3,  16(sp)
    lw   s4,  20(sp)
    lw   s5,  24(sp)
    lw   s6,  28(sp)
    lw   s7,  32(sp)
    lw   s8,  36(sp)
    lw   s9,  40(sp)
    lw   s10, 44(sp)
    lw   s11, 48(sp)
    addi sp, sp, 52

.yield_done:
    csrsi mstatus, 8           # re-enable MIE
    ret