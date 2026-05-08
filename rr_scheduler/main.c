#include <stdint.h>
#include "neorv32.h"
#include "scheduler.h"

/*
 * Updated SerDes equalizer main.c
 *
 * This version keeps the same UART packet protocol expected by serdes_gui.py,
 * but uses the updated scheduler API:
 *   - yield() is the assembly implementation in context.S
 *   - schedule() launches the first task through context_switch()
 *   - timer.S can preemptively context-switch, while respecting uart_busy
 */

/* ── UART text helpers ─────────────────────────────────────── */
static void print(const char *s) {
    while (*s) {
        if (*s == '\n')
            neorv32_uart_putc(NEORV32_UART0, '\r');
        neorv32_uart_putc(NEORV32_UART0, *s++);
    }
}

/* ── UART byte helpers ─────────────────────────────────────── */
static void uart_putc(uint8_t c) {
    neorv32_uart_putc(NEORV32_UART0, (char)c);
}

static uint8_t uart_getc(void) {
    return (uint8_t)neorv32_uart_getc(NEORV32_UART0);
}

/* ── float ↔ bytes, little-endian byte order ───────────────── */
typedef union {
    float   f;
    uint8_t b[4];
} f2b_t;

static float recv_float(void) {
    f2b_t u;
    for (int i = 0; i < 4; i++)
        u.b[i] = uart_getc();
    return u.f;
}

static void send_float(float v) {
    f2b_t u;
    u.f = v;
    for (int i = 0; i < 4; i++)
        uart_putc(u.b[i]);
}

/* ── Equalizer constants: must match serdes_gui.py ─────────── */
#define RX_FFE_PRE   3
#define RX_FFE_POST  10
#define RX_FFE_LEN   (RX_FFE_PRE + 1 + RX_FFE_POST)
#define N_DFE        1
#define MU_FFE       0.01f
#define MU_DFE       0.00f

/* GUI → FPGA packet begins with 0xAA; FPGA → GUI packet begins with 0x55. */
#define SYNC_RX      0xAA
#define SYNC_TX      0x55

#define CMD_NORMAL   0x00
#define CMD_RESET    0x01

#define STATUS_OK    0x00
#define STATUS_RESET 0x01

/* serdes_gui.py defaults to 10 lanes. If you change this, change GUI Lanes too. */
#define NUM_LANES    10

#if NUM_LANES > MAX_TASKS
#error "NUM_LANES cannot exceed scheduler MAX_TASKS"
#endif

/* ── Per-lane equalizer state ───────────────────────────────── */
static float rx_ffe[NUM_LANES][RX_FFE_LEN];
static float dfe[NUM_LANES][N_DFE];

/*
 * UART serialization token. Only the lane whose ID equals uart_token may
 * receive/process/respond to the GUI packet.
 */
static volatile int uart_token = 0;

/*
 * timer.S reads this symbol directly. Keep it global and exactly named.
 * When non-zero, the timer interrupt returns without context-switching, so a
 * UART packet cannot be split between lane tasks.
 */
volatile int uart_busy = 0;

/* ── Helper: initialize/reset one lane's taps ───────────────── */
static void init_lane_taps(int id) {
    for (int k = 0; k < RX_FFE_LEN; k++)
        rx_ffe[id][k] = 0.0f;

    rx_ffe[id][RX_FFE_PRE] = 1.0f;

    for (int k = 0; k < N_DFE; k++)
        dfe[id][k] = 0.0f;
}

/* ── Optional preemptive timer setup for timer.S ────────────── */
static void enable_scheduler_timer(void) {
    extern void trap_vector(void);
    extern void timer_init(void);

    /* Direct-mode trap vector. */
    __asm__ volatile("csrw mtvec, %0" :: "r"((uint32_t)trap_vector & ~0x3u));

    /* Arm timer.S's first deadline, then enable MTIE and global MIE. */
    timer_init();
    __asm__ volatile("csrs mie,     %0" :: "r"(1u << 7)); /* mie.MTIE */
    __asm__ volatile("csrs mstatus, %0" :: "r"(1u << 3)); /* mstatus.MIE */
}

/* ═══════════════════════════════════════════════════════════════
 * Lane task body
 *
 * Protocol per iteration:
 *   GUI  → FPGA : 0xAA | lane_id | cmd | error(f32) | rx_buf | d_hist
 *   FPGA → GUI  : 0x55 | lane_id | status | rx_ffe | dfe
 * ═══════════════════════════════════════════════════════════════ */
static void lane_task_common(int my_id) {
    init_lane_taps(my_id);

    float rx_buf[RX_FFE_LEN];
    float d_hist[N_DFE];

    while (1) {
        /* Wait cooperatively until the GUI is expected to send our lane. */
        while (uart_token != my_id)
            yield();

        /* Prevent timer.S from switching tasks in the middle of a packet. */
        uart_busy = 1;

        /* Receive request packet. Resync until the GUI→FPGA sync byte. */
        uint8_t sync;
        do {
            sync = uart_getc();
        } while (sync != SYNC_RX);

        uint8_t pkt_id = uart_getc();
        uint8_t cmd    = uart_getc();

        float error = recv_float();

        for (int k = 0; k < RX_FFE_LEN; k++)
            rx_buf[k] = recv_float();

        for (int k = 0; k < N_DFE; k++)
            d_hist[k] = recv_float();

        /*
         * The GUI should send lanes in token order. If a stale/misaligned
         * packet ever appears, ignore its ID for state updates and preserve
         * token-driven behavior. The response still reports my_id.
         */
        (void)pkt_id;

        uint8_t status = STATUS_OK;

        if (cmd == CMD_RESET) {
            init_lane_taps(my_id);
            status = STATUS_RESET;
        }

        /* Same LMS update behavior as the original main.c. */
        for (int k = 0; k < RX_FFE_LEN; k++)
            rx_ffe[my_id][k] += MU_FFE * error * rx_buf[k];

        for (int k = 0; k < N_DFE; k++)
            dfe[my_id][k] -= MU_DFE * error * d_hist[k];

        /* Send response packet. */
        uart_putc(SYNC_TX);
        uart_putc((uint8_t)my_id);
        uart_putc(status);

        for (int k = 0; k < RX_FFE_LEN; k++)
            send_float(rx_ffe[my_id][k]);

        for (int k = 0; k < N_DFE; k++)
            send_float(dfe[my_id][k]);

        /* Pass UART ownership to the next lane, then allow preemption again. */
        uart_token = (my_id + 1) % NUM_LANES;
        __asm__ volatile("" ::: "memory");
        uart_busy = 0;

        yield();
    }
}

/* create_task() takes void (*)(void), so provide one fixed wrapper per lane. */
#define DEFINE_LANE_TASK(ID) \
    static void lane_task_##ID(void) { lane_task_common(ID); }

DEFINE_LANE_TASK(0)
DEFINE_LANE_TASK(1)
DEFINE_LANE_TASK(2)
DEFINE_LANE_TASK(3)
DEFINE_LANE_TASK(4)
DEFINE_LANE_TASK(5)
DEFINE_LANE_TASK(6)
DEFINE_LANE_TASK(7)
DEFINE_LANE_TASK(8)
DEFINE_LANE_TASK(9)

static void (*const lane_entry[10])(void) = {
    lane_task_0,
    lane_task_1,
    lane_task_2,
    lane_task_3,
    lane_task_4,
    lane_task_5,
    lane_task_6,
    lane_task_7,
    lane_task_8,
    lane_task_9
};

/* ═══════════════════════════════════════════════════════════════ */
int main(void) {
    neorv32_uart_setup(NEORV32_UART0, 19200, 0);

    print("SerDes equalizer - updated RR scheduler\n");

    scheduler_init();

    uart_token = 0;

    /*
     * Hold off timer-driven context switches until after the first task has
     * entered packet processing. This avoids switching while still on main's
     * startup stack if the timer is enabled before schedule().
     */
    uart_busy = 0;

    for (int i = 0; i < NUM_LANES; i++) {
        if (create_task(lane_entry[i]) < 0) {
            print("ERROR: create_task failed\n");
            while (1) { }
        }
    }

    /*
     * Use timer.S's updated preemptive path. For purely cooperative testing,
     * comment this out; the lane tasks will still rotate using yield().
     */
    //enable_scheduler_timer();

    schedule();

    while (1) { }
}