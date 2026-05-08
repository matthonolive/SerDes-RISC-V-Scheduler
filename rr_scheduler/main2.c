#include <stdint.h>
#include "neorv32.h"
#include "scheduler.h"

/* scheduler.c references this */
volatile int uart_busy = 0;

/* ── UART text helpers ─────────────────────────────────────── */
static void print(const char *s) {
    while (*s) {
        if (*s == '\n')
            neorv32_uart_putc(NEORV32_UART0, '\r');
        neorv32_uart_putc(NEORV32_UART0, *s++);
    }
}

static void print_u32(uint32_t v) {
    char buf[11];
    int  i = 0;
    if (v == 0) { neorv32_uart_putc(NEORV32_UART0, '0'); return; }
    while (v) { buf[i++] = '0' + (v % 10); v /= 10; }
    while (i--) neorv32_uart_putc(NEORV32_UART0, buf[i]);
}

/* ── cycle counter ─────────────────────────────────────────── */
static inline uint64_t read_mcycle(void) {
    uint32_t lo, hi, hi2;
    do {
        __asm__ volatile("csrr %0, mcycleh" : "=r"(hi));
        __asm__ volatile("csrr %0, mcycle"  : "=r"(lo));
        __asm__ volatile("csrr %0, mcycleh" : "=r"(hi2));
    } while (hi != hi2);
    return ((uint64_t)hi << 32) | lo;
}

/* ── CLINT registers — must match timer.S ──────────────────── */
#define CLINT_BASE   0xFFF40000
#define MTIME_LO     (*(volatile uint32_t *)(CLINT_BASE + 0xBFF8))
#define MTIME_HI     (*(volatile uint32_t *)(CLINT_BASE + 0xBFFC))
#define MTIMECMP_LO  (*(volatile uint32_t *)(CLINT_BASE + 0x4000))
#define MTIMECMP_HI  (*(volatile uint32_t *)(CLINT_BASE + 0x4004))

/* Short interval for the benchmark probe.  timer.S will
 * reload with its own 90 M-cycle interval after the first
 * interrupt, so we only get ~1 preemption per probe run.       */
#define BENCH_TIMER_INTERVAL  10000

/* ── benchmark parameters ──────────────────────────────────── */
#define COOP_ITERS       100
#define CS_ITERS         100
#define PREEMPT_SAMPLES  200000
#define GAP_THRESHOLD    200

/* ── dummy task for create_task benchmark ──────────────────── */
static void dummy_task(void) { while (1) yield(); }

/* ── shared flags ──────────────────────────────────────────── */
static volatile int partner_done  = 0;
static volatile int bounce_mode   = 0;

/* ── scheduler internals for bare context_switch test ──────── */
extern task_t   tasks[];
extern int      current_task;
extern int      num_tasks;
extern void     context_switch(uint32_t **old_sp, uint32_t *new_sp);

/* ═══════════════════════════════════════════════════════════════
 * Partner task
 *
 * Phase 1: cooperative yield partner
 * Phase 2: bare bounce (raw context_switch back to driver)
 * Phase 3: park for preemptive test
 * ═══════════════════════════════════════════════════════════════ */
void partner_task(void) {
    /* Phase 1 */
    for (int i = 0; i < COOP_ITERS; i++)
        yield();
    partner_done = 1;

    /* Wait for bounce signal */
    while (!bounce_mode)
        yield();

    /* Phase 2: bare bounce — no yield, no pick_next */
    for (int i = 0; i < CS_ITERS; i++) {
        current_task = 0;
        context_switch(&tasks[1].sp, tasks[0].sp);
    }

    /* Phase 3: park */
    bounce_mode = 0;
    while (1) yield();
}

/* ═══════════════════════════════════════════════════════════════
 * Driver task — runs all three phases, prints results.
 * ═══════════════════════════════════════════════════════════════ */
void driver_task(void) {

    print("\n=== Scheduler cycle-count benchmark ===\n\n");

    /* ─────────────────────────────────────────────────────────
     * Phase 0: Baselines — mcycle read cost & memory latency
     * ───────────────────────────────────────────────────────── */
    #define MEM_ITERS 100

    /* 0a: how long does read_mcycle itself take? */
    uint64_t cal0 = read_mcycle();
    for (int i = 0; i < MEM_ITERS; i++) {
        read_mcycle();
    }
    uint64_t cal1 = read_mcycle();
    uint32_t mcycle_cost = (uint32_t)(cal1 - cal0) / MEM_ITERS;

    /* 0b: single sw + lw pair latency (volatile forces both) */
    volatile uint32_t scratch = 0;
    uint64_t mem0 = read_mcycle();
    for (int i = 0; i < MEM_ITERS; i++) {
        scratch = 42;
        (void)scratch;
    }
    uint64_t mem1 = read_mcycle();
    uint32_t mem_cost = (uint32_t)(mem1 - mem0) / MEM_ITERS;

    /* 0c: back-to-back mcycle gap (= one probe loop iteration) */
    uint32_t gap_min = UINT32_MAX;
    uint32_t gap_max = 0;
    uint64_t gprev = read_mcycle();
    for (int i = 0; i < MEM_ITERS; i++) {
        uint64_t gnow = read_mcycle();
        uint32_t g = (uint32_t)(gnow - gprev);
        if (g < gap_min) gap_min = g;
        if (g > gap_max) gap_max = g;
        gprev = gnow;
    }

    print("[0] Baselines\n");
    print("    read_mcycle overhead : "); print_u32(mcycle_cost);
    print(" cycles\n");
    print("    sw+lw pair          : "); print_u32(mem_cost);
    print(" cycles  (memory latency indicator)\n");
    print("    back-to-back gap    : "); print_u32(gap_min);
    print("-"); print_u32(gap_max);
    print(" cycles  (probe loop floor)\n\n");

    /* ─────────────────────────────────────────────────────────
     * Phase 1: Cooperative yield() round-trip
     * ───────────────────────────────────────────────────────── */
    uint32_t coop_total = 0;
    uint32_t coop_min   = UINT32_MAX;
    uint32_t coop_max   = 0;

    for (int i = 0; i < COOP_ITERS; i++) {
        uint64_t t0 = read_mcycle();
        yield();
        uint64_t t1 = read_mcycle();

        uint32_t dt = (uint32_t)(t1 - t0);
        coop_total += dt;
        if (dt < coop_min) coop_min = dt;
        if (dt > coop_max) coop_max = dt;
    }

    while (!partner_done)
        yield();

    print("[1] Cooperative yield() round-trip  (");
    print_u32(COOP_ITERS);
    print(" samples, 2 tasks)\n");
    print("    min : "); print_u32(coop_min);  print(" cycles\n");
    print("    max : "); print_u32(coop_max);  print(" cycles\n");
    print("    avg : "); print_u32(coop_total / COOP_ITERS);
    print(" cycles  (divide by 2 for one-way yield cost)\n\n");

    /* ─────────────────────────────────────────────────────────
     * Phase 2: Bare context_switch
     * ───────────────────────────────────────────────────────── */
    uint32_t cs_total = 0;
    uint32_t cs_min   = UINT32_MAX;
    uint32_t cs_max   = 0;

    bounce_mode = 1;
    yield();      /* let partner see the flag */

    for (int i = 0; i < CS_ITERS; i++) {
        uint64_t t0 = read_mcycle();
        current_task = 1;
        context_switch(&tasks[0].sp, tasks[1].sp);
        uint64_t t1 = read_mcycle();

        uint32_t dt = (uint32_t)(t1 - t0);
        cs_total += dt;
        if (dt < cs_min) cs_min = dt;
        if (dt > cs_max) cs_max = dt;
    }

    while (bounce_mode)
        yield();

    print("[2] Bare context_switch  (");
    print_u32(CS_ITERS);
    print(" samples)\n");
    print("    min : "); print_u32(cs_min);  print(" cycles\n");
    print("    max : "); print_u32(cs_max);  print(" cycles\n");
    print("    avg : "); print_u32(cs_total / CS_ITERS);
    print(" cycles  (divide by 2 for one-way switch cost)\n\n");

    /* ─────────────────────────────────────────────────────────
     * Phase 3: Preemptive switch probe
     *
     * Arm the CLINT timer with a short interval, enable
     * interrupts, then probe.  timer.S will reload mtimecmp
     * with its own 90 M-cycle interval after the first IRQ,
     * so expect only ~1 preemption per run.
     * ───────────────────────────────────────────────────────── */
    print("[3] Preemptive switch probe  (");
    print_u32(PREEMPT_SAMPLES);
    print(" loop iters, gap threshold=");
    print_u32(GAP_THRESHOLD);
    print(")\n");

    /* Read current mtime */
    uint32_t thi, tlo, thi2;
    do {
        thi  = MTIME_HI;
        tlo  = MTIME_LO;
        thi2 = MTIME_HI;
    } while (thi != thi2);

    /* Arm mtimecmp with a short interval */
    uint64_t cmp_val = ((uint64_t)thi << 32 | tlo) + BENCH_TIMER_INTERVAL;
    MTIMECMP_LO = 0xFFFFFFFFu;          /* prevent spurious match */
    MTIMECMP_HI = (uint32_t)(cmp_val >> 32);
    MTIMECMP_LO = (uint32_t)cmp_val;

    /* Enable timer interrupt */
    __asm__ volatile("csrs mie,     %0" :: "r"(1u << 7));   /* MTIE */
    __asm__ volatile("csrs mstatus, %0" :: "r"(1u << 3));   /* MIE  */

    uint32_t pre_max   = 0;
    uint32_t pre_min   = UINT32_MAX;
    uint64_t pre_total = 0;
    uint32_t pre_count = 0;

    uint64_t prev = read_mcycle();
    for (int i = 0; i < PREEMPT_SAMPLES; i++) {
        uint64_t now = read_mcycle();
        uint32_t gap = (uint32_t)(now - prev);
        if (gap > GAP_THRESHOLD) {
            pre_count++;
            pre_total += gap;
            if (gap > pre_max) pre_max = gap;
            if (gap < pre_min) pre_min = gap;
        }
        prev = now;
    }

    /* Disable timer interrupt */
    __asm__ volatile("csrc mie, %0" :: "r"(1u << 7));

    if (pre_count == 0) {
        print("    No preemptions detected.\n");
        print("    (timer.S interval is 90M cycles — probe window\n");
        print("     may be too short.  Try increasing PREEMPT_SAMPLES.)\n");
    } else {
        print("    preemptions seen : "); print_u32(pre_count);  print("\n");
        print("    min gap          : "); print_u32(pre_min);    print(" cycles\n");
        print("    max gap          : "); print_u32(pre_max);    print(" cycles\n");
        print("    avg gap          : ");
        print_u32((uint32_t)(pre_total / pre_count));
        print(" cycles  (ISR entry/exit + 2 context switches)\n");
    }

    /* ─────────────────────────────────────────────────────────
     * Phase 4: Task creation cost
     *
     * Measure create_task() by creating dummy tasks into the
     * remaining slots.  Reset num_tasks between rounds to
     * reuse the slots (those tasks never run).
     * ───────────────────────────────────────────────────────── */
    #define CT_ROUNDS     5
    #define CT_PER_ROUND  (MAX_TASKS - 2)   /* slots 2..9 */

    uint32_t ct_total = 0;
    uint32_t ct_min   = UINT32_MAX;
    uint32_t ct_max   = 0;
    uint32_t ct_count = 0;

    for (int r = 0; r < CT_ROUNDS; r++) {
        num_tasks = 2;                      /* reset to just driver+partner */
        for (int j = 0; j < CT_PER_ROUND; j++) {
            uint64_t t0 = read_mcycle();
            create_task(dummy_task);
            uint64_t t1 = read_mcycle();

            uint32_t dt = (uint32_t)(t1 - t0);
            ct_total += dt;
            ct_count++;
            if (dt < ct_min) ct_min = dt;
            if (dt > ct_max) ct_max = dt;
        }
    }
    num_tasks = 2;                          /* leave clean for yield() */

    print("\n[4] Task creation  (");
    print_u32(ct_count);
    print(" samples)\n");
    print("    min : "); print_u32(ct_min);  print(" cycles\n");
    print("    max : "); print_u32(ct_max);  print(" cycles\n");
    print("    avg : "); print_u32(ct_total / ct_count);
    print(" cycles\n");

    print("\nDone.\n");
    while (1) yield();
}

/* ═══════════════════════════════════════════════════════════════ */
int main(void) {
    neorv32_uart_setup(NEORV32_UART0, 19200, 0);

    extern void trap_vector(void);
    __asm__ volatile("csrw mtvec, %0" :: "r"((uint32_t)trap_vector & ~0x3));

    /* Timer is NOT enabled here — phase 3 arms it when ready */

    scheduler_init();

    create_task(driver_task);
    create_task(partner_task);

    schedule();
    while (1);
}