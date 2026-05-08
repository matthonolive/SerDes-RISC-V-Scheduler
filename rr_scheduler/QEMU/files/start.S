.section .text.init
.global _start

_start:
    la   sp, _stack_top

    # Zero BSS
    la   t0, _bss_start
    la   t1, _bss_end
1:  bge  t0, t1, 2f
    sw   zero, 0(t0)
    addi t0, t0, 4
    j    1b
2:
    # Set trap vector (main2.c also does this, but belt-and-suspenders)
    la   t0, trap_vector
    csrw mtvec, t0

    call main

3:  wfi
    j    3b
