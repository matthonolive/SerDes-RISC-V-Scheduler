.section .text.init
.global _start

_start:
    la sp, _stack_top           # initialize stack pointer

    # Enable FPU: set mstatus.FS = 01 (Initial)
    li t0, (1 << 13)
    csrs mstatus, t0

    # Zero the BSS section
    la t0, _bss_start
    la t1, _bss_end
1:  bge t0, t1, 2f
    sw zero, 0(t0)
    addi t0, t0, 4
    j 1b
2:

    # Set up trap vector (aligned, direct mode)
    la t0, trap_vector
    csrw mtvec, t0

    # Initialize the timer
    call timer_init

    # Enable machine timer interrupt (mie.MTIE = bit 7)
    li t0, (1 << 7)
    csrs mie, t0

    # Enable global interrupts (mstatus.MIE = bit 3)
    li t0, (1 << 3)
    csrs mstatus, t0

    call main

3:  wfi
    j 3b