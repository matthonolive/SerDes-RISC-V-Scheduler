#include <stdint.h>
#include "neorv32.h"
#include "scheduler.h"

/* scheduler.c references this */
volatile int uart_busy = 0;

/* These are defined in scheduler.c */
extern task_t   tasks[MAX_TASKS];
extern uint32_t stacks[MAX_TASKS][STACK_SIZE];
extern int      current_task;
extern int      num_tasks;

/* Assembly scheduler internals */
extern void trap_vector(void);

/* Constants based on context.S / timer.S */
#define TWO_KB_BYTES             2048u
#define STACK_CANARY             0xA5A5A5A5u
#define STACK_TEST_YIELDS        100u
#define COOP_FRAME_BYTES         52u   /* ra + s0-s11 = 13 regs */
#define IRQ_FRAME_BYTES          80u   /* caller-saved IRQ frame */
#define PREEMPT_FRAME_BYTES      (COOP_FRAME_BYTES + IRQ_FRAME_BYTES)

static uint32_t stack_used_snapshot[MAX_TASKS];

/* ───────────────────────────────────────────────────────────── */
/* UART helpers                                                  */
/* ───────────────────────────────────────────────────────────── */

static void print(const char *s) {
    while (*s) {
        if (*s == '\n')
            neorv32_uart_putc(NEORV32_UART0, '\r');
        neorv32_uart_putc(NEORV32_UART0, *s++);
    }
}

static void print_u32(uint32_t v) {
    char buf[11];
    int i = 0;

    if (v == 0) {
        neorv32_uart_putc(NEORV32_UART0, '0');
        return;
    }

    while (v) {
        buf[i++] = '0' + (v % 10);
        v /= 10;
    }

    while (i--)
        neorv32_uart_putc(NEORV32_UART0, buf[i]);
}

static void print_bytes(uint32_t bytes) {
    print_u32(bytes);
    print(" B");
}

static void print_pass_fail(uint32_t value, uint32_t limit) {
    if (value <= limit)
        print("  [PASS]\n");
    else
        print("  [FAIL]\n");
}

/* ───────────────────────────────────────────────────────────── */
/* Stack high-water helpers                                      */
/* ───────────────────────────────────────────────────────────── */

static void fill_stack_canaries(void) {
    for (int t = 0; t < MAX_TASKS; t++) {
        for (int i = 0; i < STACK_SIZE; i++) {
            stacks[t][i] = STACK_CANARY;
        }
    }
}

/*
 * Stacks grow downward from stacks[id][STACK_SIZE].
 * The lowest addresses remain canary if unused.
 */
static uint32_t stack_used_bytes(int task_id) {
    uint32_t unused_words = 0;

    while ((unused_words < STACK_SIZE) &&
           (stacks[task_id][unused_words] == STACK_CANARY)) {
        unused_words++;
    }

    return (STACK_SIZE - unused_words) * sizeof(uint32_t);
}

static void snapshot_stack_usage(void) {
    for (int t = 0; t < MAX_TASKS; t++) {
        stack_used_snapshot[t] = stack_used_bytes(t);
    }
}

/* ───────────────────────────────────────────────────────────── */
/* Static RAM report                                             */
/* ───────────────────────────────────────────────────────────── */

static void print_static_memory_report(void) {
    uint32_t task_table_bytes = (uint32_t)sizeof(tasks);
    uint32_t stack_bytes      = (uint32_t)sizeof(stacks);
    uint32_t globals_bytes    = (uint32_t)(sizeof(current_task) + sizeof(num_tasks));

    uint32_t core_scheduler_ram = task_table_bytes + globals_bytes;
    uint32_t total_with_stacks  = core_scheduler_ram + stack_bytes;

    print("\n=== Scheduler memory-footprint report ===\n\n");

    print("[Configuration]\n");
    print("    MAX_TASKS              : "); print_u32(MAX_TASKS); print("\n");
    print("    STACK_SIZE             : "); print_u32(STACK_SIZE);
    print(" words = "); print_u32(STACK_SIZE * sizeof(uint32_t)); print(" B/task\n");
    print("    sizeof(task_t)         : "); print_u32((uint32_t)sizeof(task_t)); print(" B\n\n");

    print("[Static RAM estimate]\n");
    print("    task table             : "); print_bytes(task_table_bytes); print("\n");
    print("    scheduler globals      : "); print_bytes(globals_bytes); print("\n");
    print("    core scheduler RAM     : "); print_bytes(core_scheduler_ram);
    print_pass_fail(core_scheduler_ram, TWO_KB_BYTES);

    print("    reserved task stacks   : "); print_bytes(stack_bytes); print("\n");
    print("    total incl. stacks     : "); print_bytes(total_with_stacks);
    print_pass_fail(total_with_stacks, TWO_KB_BYTES);

    print("\n[Frame overhead]\n");
    print("    cooperative frame      : "); print_bytes(COOP_FRAME_BYTES); print("\n");
    print("    interrupt frame        : "); print_bytes(IRQ_FRAME_BYTES); print("\n");
    print("    preempted-task frame   : "); print_bytes(PREEMPT_FRAME_BYTES); print("\n");

    print("\nInterpretation:\n");
    print("    If task stacks are excluded, compare core scheduler RAM to 2 KB.\n");
    print("    If task stacks are included, compare total incl. stacks to 2 KB.\n\n");
}

/* ───────────────────────────────────────────────────────────── */
/* Runtime stack test tasks                                      */
/* ───────────────────────────────────────────────────────────── */

static volatile uint32_t worker_counter = 0;

void worker_task(void) {
    for (uint32_t i = 0; i < STACK_TEST_YIELDS; i++) {
        worker_counter++;
        yield();
    }

    while (1) {
        yield();
    }
}

void driver_task(void) {
    /*
     * Exercise cooperative switching before taking the snapshot.
     * No printing before snapshot, so the printed stack report does not
     * affect the captured worker-task stack use.
     */
    for (uint32_t i = 0; i < STACK_TEST_YIELDS; i++) {
        yield();
    }

    snapshot_stack_usage();

    print("[Runtime stack high-water estimate]\n");
    print("    Note: task 0 includes measurement/printing overhead.\n");

    uint32_t sum_used = 0;
    uint32_t max_used = 0;

    for (int t = 0; t < num_tasks; t++) {
        uint32_t used = stack_used_snapshot[t];
        sum_used += used;
        if (used > max_used)
            max_used = used;

        print("    task ");
        print_u32((uint32_t)t);
        print(" stack used        : ");
        print_bytes(used);
        print(" / ");
        print_bytes(STACK_SIZE * sizeof(uint32_t));
        print("\n");
    }

    print("    total used stack      : "); print_bytes(sum_used); print("\n");
    print("    max used stack/task   : "); print_bytes(max_used); print("\n");

    print("\nDone.\n");

    while (1) {
        yield();
    }
}

/* ───────────────────────────────────────────────────────────── */

int main(void) {
    neorv32_uart_setup(NEORV32_UART0, 19200, 0);

    /*
     * Use the trap vector from timer.S, but disable timer interrupts
     * for this memory benchmark. We only need cooperative yield().
     */
    __asm__ volatile("csrw mtvec, %0" :: "r"((uint32_t)trap_vector & ~0x3));
    __asm__ volatile("csrc mie,     %0" :: "r"(1u << 7)); /* disable MTIE */
    __asm__ volatile("csrc mstatus, %0" :: "r"(1u << 3)); /* disable MIE  */

    print_static_memory_report();

    scheduler_init();

    /*
     * Important: fill canaries before create_task(), because create_task()
     * writes the initial task frame onto each task stack.
     */
    fill_stack_canaries();

    create_task(driver_task);
    create_task(worker_task);

    schedule();

    while (1) {
        __asm__ volatile("wfi");
    }
}