#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <stdint.h>

#define MAX_TASKS   10
#define STACK_SIZE  256      /* words (1 KB per task stack) */

typedef struct {
    uint32_t *sp;           /* saved stack pointer (cooperative frame) */
    uint8_t   active;       /* 1 = runnable                           */
} task_t;

/* Core API */
void scheduler_init(void);
int  create_task(void (*func)(void));
void yield(void);           /* cooperative: context_switch, 13 regs   */
void schedule(void);        /* launch first task (called once)        */

/* preempt_schedule() is no longer a C function — the scheduling
 * logic is inlined directly into trap_vector in timer.S to
 * avoid the C function prologue/epilogue overhead.                   */

#endif