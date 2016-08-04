/*
 * QEMU PowerMac PMU device support
 *
 * Copyright (c) 2016 Benjamin Herrenschmidt, IBM Corp.
 *
 * Based on the CUDA device by:
 *
 * Copyright (c) 2004-2007 Fabrice Bellard
 * Copyright (c) 2007 Jocelyn Mayer
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#include "qemu/osdep.h"
#include "hw/hw.h"
#include "hw/ppc/mac.h"
#include "hw/input/adb.h"
#include "qemu/timer.h"
#include "sysemu/sysemu.h"
#include "qemu/cutils.h"
#include "qemu/log.h"
#include "pmu.h"

/* XXX: implement all timer modes */

#undef DEBUG_PMU
#undef DEBUG_PMU_ALL_MMIO
#undef DEBUG_PMU_PROTOCOL
#undef DEBUG_VIA

/* debug PMU packets */
#define DEBUG_PMU_PACKET

#ifdef DEBUG_PMU
#define PMU_DPRINTF(fmt, ...)                                  \
    do { printf("PMU: " fmt , ## __VA_ARGS__); } while (0)
#else
#define PMU_DPRINTF(fmt, ...)
#endif

#define REG_B		0		/* B-side data */
#define REG_A		1		/* A-side data */
#define REG_DIRB        2	        /* B-side direction (1=output) */
#define REG_DIRA        3	        /* A-side direction (1=output) */
#define REG_T1CL        4		/* Timer 1 ctr/latch (low 8 bits) */
#define REG_T1CH	5		/* Timer 1 counter (high 8 bits) */
#define REG_T1LL	6		/* Timer 1 latch (low 8 bits) */
#define REG_T1LH	7		/* Timer 1 latch (high 8 bits) */
#define REG_T2CL	8		/* Timer 2 ctr/latch (low 8 bits) */
#define REG_T2CH	9		/* Timer 2 counter (high 8 bits) */
#define REG_SR          10		/* Shift register */
#define REG_ACR		11	/* Auxiliary control register */
#define REG_PCR		12	/* Peripheral control register */
#define REG_IFR		13	/* Interrupt flag register */
#define REG_IER		14	/* Interrupt enable register */
#define REG_ANH		15	/* A-side data, no handshake */

/* Bits in B data register: all active low */
#define TACK		0x08		/* Transfer request (input) */
#define TREQ		0x10		/* Transfer acknowledge (output) */

/* Bits in ACR */
#define SR_CTRL		0x1c		/* Shift register control bits */
#define SR_EXT		0x0c		/* Shift on external clock */
#define SR_OUT		0x10		/* Shift out if 1 */

/* Bits in IFR and IER */
#define IER_SET		0x80		/* set bits in IER */
#define IER_CLR		0		/* clear bits in IER */
#define CA2_INT	        0x01
#define CA1_INT 	0x02
#define SR_INT		0x04		/* Shift register full/empty */
#define CB2_INT	        0x08
#define CB1_INT 	0x10
#define T1_INT          0x40            /* Timer 1 interrupt */
#define T2_INT          0x20            /* Timer 2 interrupt */

/* Bits in ACR */
#define T1MODE          0xc0            /* Timer 1 mode */
#define T1MODE_CONT     0x40            /*  continuous interrupts */

/* PMU returns time_t's offset from Jan 1, 1904, not 1970 */
#define RTC_OFFSET                      2082844800

/*
 * This table indicates for each PMU opcode:
 * - the number of data bytes to be sent with the command, or -1
 *   if a length byte should be sent,
 * - the number of response bytes which the PMU will return, or
 *   -1 if it will send a length byte.
 */
static const int8_t pmu_data_len[256][2] = {
/*	   0	   1	   2	   3	   4	   5	   6	   7  */
/*00*/	{-1, 0},{-1, 0},{-1, 0},{-1, 0},{-1, 0},{-1, 0},{-1, 0},{-1, 0},
/*08*/	{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
/*10*/	{ 1, 0},{ 1, 0},{-1, 0},{-1, 0},{-1, 0},{-1, 0},{-1, 0},{-1, 0},
/*18*/	{ 0, 1},{ 0, 1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{ 0, 0},
/*20*/	{-1, 0},{ 0, 0},{ 2, 0},{ 1, 0},{ 1, 0},{-1, 0},{-1, 0},{-1, 0},
/*28*/	{ 0,-1},{ 0,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{ 0,-1},
/*30*/	{ 4, 0},{20, 0},{-1, 0},{ 3, 0},{-1, 0},{-1, 0},{-1, 0},{-1, 0},
/*38*/	{ 0, 4},{ 0,20},{ 2,-1},{ 2, 1},{ 3,-1},{-1,-1},{-1,-1},{ 4, 0},
/*40*/	{ 1, 0},{ 1, 0},{-1, 0},{-1, 0},{-1, 0},{-1, 0},{-1, 0},{-1, 0},
/*48*/	{ 0, 1},{ 0, 1},{-1,-1},{ 1, 0},{ 1, 0},{-1,-1},{-1,-1},{-1,-1},
/*50*/	{ 1, 0},{ 0, 0},{ 2, 0},{ 2, 0},{-1, 0},{ 1, 0},{ 3, 0},{ 1, 0},
/*58*/	{ 0, 1},{ 1, 0},{ 0, 2},{ 0, 2},{ 0,-1},{-1,-1},{-1,-1},{-1,-1},
/*60*/	{ 2, 0},{-1, 0},{-1, 0},{-1, 0},{-1, 0},{-1, 0},{-1, 0},{-1, 0},
/*68*/	{ 0, 3},{ 0, 3},{ 0, 2},{ 0, 8},{ 0,-1},{ 0,-1},{-1,-1},{-1,-1},
/*70*/	{ 1, 0},{ 1, 0},{ 1, 0},{-1, 0},{-1, 0},{-1, 0},{-1, 0},{-1, 0},
/*78*/	{ 0,-1},{ 0,-1},{-1,-1},{-1,-1},{-1,-1},{ 5, 1},{ 4, 1},{ 4, 1},
/*80*/	{ 4, 0},{-1, 0},{ 0, 0},{-1, 0},{-1, 0},{-1, 0},{-1, 0},{-1, 0},
/*88*/	{ 0, 5},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
/*90*/	{ 1, 0},{ 2, 0},{-1, 0},{-1, 0},{-1, 0},{-1, 0},{-1, 0},{-1, 0},
/*98*/	{ 0, 1},{ 0, 1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
/*a0*/	{ 2, 0},{ 2, 0},{ 2, 0},{ 4, 0},{-1, 0},{ 0, 0},{-1, 0},{-1, 0},
/*a8*/	{ 1, 1},{ 1, 0},{ 3, 0},{ 2, 0},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
/*b0*/	{-1, 0},{-1, 0},{-1, 0},{-1, 0},{-1, 0},{-1, 0},{-1, 0},{-1, 0},
/*b8*/	{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
/*c0*/	{-1, 0},{-1, 0},{-1, 0},{-1, 0},{-1, 0},{-1, 0},{-1, 0},{-1, 0},
/*c8*/	{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
/*d0*/	{ 0, 0},{-1, 0},{-1, 0},{-1, 0},{-1, 0},{-1, 0},{-1, 0},{-1, 0},
/*d8*/	{ 1, 1},{ 1, 1},{-1,-1},{-1,-1},{ 0, 1},{ 0,-1},{-1,-1},{-1,-1},
/*e0*/	{-1, 0},{ 4, 0},{ 0, 1},{-1, 0},{-1, 0},{ 4, 0},{-1, 0},{-1, 0},
/*e8*/	{ 3,-1},{-1,-1},{ 0, 1},{-1,-1},{ 0,-1},{-1,-1},{-1,-1},{ 0, 0},
/*f0*/	{-1, 0},{-1, 0},{-1, 0},{-1, 0},{-1, 0},{-1, 0},{-1, 0},{-1, 0},
/*f8*/	{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
};

/* Command protocol state machine */
typedef enum {
    pmu_state_idle, /* Waiting for command */
    pmu_state_cmd,  /* Receiving command */
    pmu_state_rsp,  /* Responding to command */
} PMUCmdState;


#define TYPE_VIA_PMU "via-pmu"
#define VIA_PMU(obj) OBJECT_CHECK(PMUState, (obj), TYPE_VIA_PMU)

/* XXX FIXME */
//#define VIA_TIMER_FREQ (4700000 / 6)
#define VIA_TIMER_FREQ (4700000 / 1)

/**
 * VIATimer:
 * @counter_value: counter value at load time
 */
typedef struct VIATimer {
    int index;
    uint16_t latch;
    uint16_t counter_value;
    int64_t load_time;
    int64_t next_irq_time;
    uint64_t frequency;
    QEMUTimer *timer;
} VIATimer;

/**
 * PMUState:
 * @b: B-side data
 * @a: A-side data
 * @dirb: B-side direction (1=output)
 * @dira: A-side direction (1=output)
 * @sr: Shift register
 * @acr: Auxiliary control register
 * @pcr: Peripheral control register
 * @ifr: Interrupt flag register
 * @ier: Interrupt enable register
 * @anh: A-side data, no handshake
 * @last_b: last value of B register
 * @last_acr: last value of ACR register
 */
typedef struct PMUState {
    /*< private >*/
    SysBusDevice parent_obj;
    /*< public >*/

    /* --- VIA state, to move into a different object ! --- */
    MemoryRegion mem;
    uint8_t b;
    uint8_t a;
    uint8_t dirb;
    uint8_t dira;
    uint8_t sr;
    uint8_t acr;
    uint8_t pcr;
    uint8_t ifr;
    uint8_t ier;
    uint8_t anh;
    VIATimer timers[2];
    uint64_t frequency;
    qemu_irq via_irq;

    /* MacOS 9 is racy and requires a delay upon setting the SR_INT bit */
    QEMUTimer *sr_delay_timer;

    /* --- PMU state --- */

    /* PMU state */
    ADBBusState adb_bus;
    uint32_t tick_offset;
    uint8_t last_b;
    uint8_t intbits;
    uint8_t intmask;

    PMUCmdState cmd_state;
    uint8_t cmd;
    int cmdlen;
    int rsplen;
    uint8_t cmd_buf_pos;
    uint8_t cmd_buf[128];
    uint8_t cmd_rsp_pos;
    uint8_t cmd_rsp_sz;
    uint8_t cmd_rsp[128];

    uint16_t adb_poll_mask;
    uint8_t autopoll_rate_ms;
    uint8_t autopoll;
    QEMUTimer *adb_poll_timer;

    /* XXX HACK */
    void *macio;
} PMUState;

static void via_timer_update(PMUState *s, VIATimer *ti, int64_t current_time);

static void via_update_irq(PMUState *s)
{
    if (s->ifr & s->ier & (SR_INT | T1_INT | T2_INT)) {
        qemu_irq_raise(s->via_irq);
    } else {
        qemu_irq_lower(s->via_irq);
    }
}

static uint64_t get_tb(uint64_t time, uint64_t freq)
{
    return muldiv64(time, freq, NANOSECONDS_PER_SECOND);
}

static unsigned int get_counter(VIATimer *ti)
{
    int64_t d;
    unsigned int counter;
    uint64_t tb_diff;
    uint64_t current_time = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    /* Reverse of the tb calculation algorithm that Mac OS X uses on bootup. */
    tb_diff = get_tb(current_time, ti->frequency) - ti->load_time;
    d = (tb_diff * 0xBF401675E5DULL) / (ti->frequency << 24);

    if (ti->index == 0) {
        /* the timer goes down from latch to -1 (period of latch + 2) */
        if (d <= (ti->counter_value + 1)) {
            counter = (ti->counter_value - d) & 0xffff;
        } else {
            counter = (d - (ti->counter_value + 1)) % (ti->latch + 2);
            counter = (ti->latch - counter) & 0xffff;
        }
    } else {
        counter = (ti->counter_value - d) & 0xffff;
    }
    return counter;
}

static void set_counter(PMUState *s, VIATimer *ti, unsigned int val)
{
#ifdef DEBUG_VIA
    PMU_DPRINTF("T%d.counter=%d\n", 1 + ti->index, val);
#endif
    ti->load_time = get_tb(qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL),
                           s->frequency);
    ti->counter_value = val;
    via_timer_update(s, ti, ti->load_time);
}

static int64_t get_next_irq_time(VIATimer *s, int64_t current_time)
{
    int64_t d, next_time;
    unsigned int counter;

    /* current counter value */
    d = muldiv64(current_time - s->load_time,
                 VIA_TIMER_FREQ, NANOSECONDS_PER_SECOND);
    /* the timer goes down from latch to -1 (period of latch + 2) */
    if (d <= (s->counter_value + 1)) {
        counter = (s->counter_value - d) & 0xffff;
    } else {
        counter = (d - (s->counter_value + 1)) % (s->latch + 2);
        counter = (s->latch - counter) & 0xffff;
    }

    /* Note: we consider the irq is raised on 0 */
    if (counter == 0xffff) {
        next_time = d + s->latch + 1;
    } else if (counter == 0) {
        next_time = d + s->latch + 2;
    } else {
        next_time = d + counter;
    }
#ifdef DEBUG_VIA
    PMU_DPRINTF("latch=%d counter=%" PRId64 " delta_next=%" PRId64 "\n",
                s->latch, d, next_time - d);
#endif
    next_time = muldiv64(next_time, NANOSECONDS_PER_SECOND, VIA_TIMER_FREQ) +
        s->load_time;
    if (next_time <= current_time)
        next_time = current_time + 1;
    return next_time;
}

static void via_timer_update(PMUState *s, VIATimer *ti,
                             int64_t current_time)
{
    if (!ti->timer)
        return;
    if (ti->index == 0 && (s->acr & T1MODE) != T1MODE_CONT) {
        timer_del(ti->timer);
    } else {
        ti->next_irq_time = get_next_irq_time(ti, current_time);
        timer_mod(ti->timer, ti->next_irq_time);
    }
}

static void via_timer1(void *opaque)
{
    PMUState *s = opaque;
    VIATimer *ti = &s->timers[0];

    via_timer_update(s, ti, ti->next_irq_time);
    s->ifr |= T1_INT;
    via_update_irq(s);
}

static void via_timer2(void *opaque)
{
    PMUState *s = opaque;
    VIATimer *ti = &s->timers[1];

    via_timer_update(s, ti, ti->next_irq_time);
    s->ifr |= T2_INT;
    via_update_irq(s);
}

static void via_set_sr_int(void *opaque)
{
    PMUState *s = opaque;

#ifdef DEBUG_VIA
    PMU_DPRINTF("VIA: %s:%d\n", __func__, __LINE__);
#endif
    s->ifr |= SR_INT;
    via_update_irq(s);
}

static void via_delay_set_sr_int(PMUState *s)
{
    int64_t expire;

    if (s->dirb == 0xff) {
        /* Not in Mac OS, fire the IRQ directly */
        via_set_sr_int(s);
        return;
    }

#ifdef DEBUG_VIA
    PMU_DPRINTF("PMU: %s:%d\n", __func__, __LINE__);
#endif
    expire = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 300 * SCALE_US;
    timer_mod(s->sr_delay_timer, expire);
}

static void pmu_adb_poll(void *opaque)
{
#if 0
    PMUState *s = opaque;
    uint8_t obuf[ADB_MAX_OUT_LEN + 2];
    int olen;

    olen = adb_poll(&s->adb_bus, obuf + 2, s->adb_poll_mask);
    if (olen > 0) {
        obuf[0] = ADB_PACKET;
        obuf[1] = 0x40; /* polled data */
        cuda_send_packet_to_host(s, obuf, olen + 2);
    }
    timer_mod(s->adb_poll_timer,
                   qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                   (NANOSECONDS_PER_SECOND / (1000 / s->autopoll_rate_ms)));
#endif
}

static void pmu_update_extirq(PMUState *s)
{
        //. XXX
    MacIOSetGPIO(s->macio, 9, false);
}

static void pmu_cmd_int_ack(PMUState *s,
                            const uint8_t *in_data, uint8_t in_len,
                            uint8_t *out_data, uint8_t *out_len)
{
    if (in_len != 0) {
        PMU_DPRINTF("INT_ACK command, invalid len: %d want: 0\n", in_len);
        return;
    }
    out_data[0] = s->intbits;
    s->intbits = 0;
    pmu_update_extirq(s);
    *out_len = 1;
}

static void pmu_cmd_set_int_mask(PMUState *s,
                                 const uint8_t *in_data, uint8_t in_len,
                                 uint8_t *out_data, uint8_t *out_len)
{
    if (in_len != 1) {
        PMU_DPRINTF("SET_INT_MASK command, invalid len: %d want: 1\n", in_len);
        return;
    }
    s->intmask = in_data[0];
    PMU_DPRINTF("Setting PMU int mask to 0x%02x\n", s->intmask);
    pmu_update_extirq(s);
}

static void pmu_cmd_adb(PMUState *s,
                        const uint8_t *in_data, uint8_t in_len,
                        uint8_t *out_data, uint8_t *out_len)
{
    if (in_len != 0) {
        PMU_DPRINTF("ADB POLL OFF command, invalid len: %d want: 0\n", in_len);
        return;
    }

    if (s->autopoll) {
            timer_del(s->adb_poll_timer);
            s->autopoll = false;
    }
#if 0
    if (autopoll) {
            timer_mod(s->adb_poll_timer,
                      qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                      (NANOSECONDS_PER_SECOND / (1000 / s->autopoll_rate_ms)));
    }
#endif
}

static void pmu_cmd_adb_poll_off(PMUState *s,
                                 const uint8_t *in_data, uint8_t in_len,
                                 uint8_t *out_data, uint8_t *out_len)
{
    if (in_len != 0) {
        PMU_DPRINTF("ADB POLL OFF command, invalid len: %d want: 0\n", in_len);
        return;
    }

    if (s->autopoll) {
            timer_del(s->adb_poll_timer);
            s->autopoll = false;
    }
#if 0
    if (autopoll) {
            timer_mod(s->adb_poll_timer,
                      qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                      (NANOSECONDS_PER_SECOND / (1000 / s->autopoll_rate_ms)));
    }
#endif
}

static void pmu_cmd_shutdown(PMUState *s,
                             const uint8_t *in_data, uint8_t in_len,
                             uint8_t *out_data, uint8_t *out_len)
{
    if (in_len != 4) {
        PMU_DPRINTF("SHUTDOWN command, invalid len: %d want: 4\n", in_len);
        return;
    }
    *out_len = 1;
    out_data[0] = 0;
    if (in_data[0] != 'M' || in_data[1] != 'A' ||
        in_data[2] != 'T' || in_data[3] != 'T') {
        PMU_DPRINTF("SHUTDOWN command, Bad MATT signature\n");
        return;
    }
    qemu_system_shutdown_request();
}

static void pmu_cmd_reset(PMUState *s,
                          const uint8_t *in_data, uint8_t in_len,
                          uint8_t *out_data, uint8_t *out_len)
{
    if (in_len != 0) {
        PMU_DPRINTF("RESET command, invalid len: %d want: 0\n", in_len);
        return;
    }

    qemu_system_reset_request();
}

static void pmu_cmd_get_rtc(PMUState *s,
                            const uint8_t *in_data, uint8_t in_len,
                            uint8_t *out_data, uint8_t *out_len)
{
    uint32_t ti;

    if (in_len != 0) {
        PMU_DPRINTF("GET_RTC command, invalid len: %d want: 0\n", in_len);
        return;
    }

    ti = s->tick_offset + (qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)
                           / NANOSECONDS_PER_SECOND);
    out_data[0] = ti >> 24;
    out_data[1] = ti >> 16;
    out_data[2] = ti >> 8;
    out_data[3] = ti;
    *out_len = 4;
}

static void pmu_cmd_set_rtc(PMUState *s,
                            const uint8_t *in_data, uint8_t in_len,
                            uint8_t *out_data, uint8_t *out_len)
{
    uint32_t ti;

    if (in_len != 4) {
        PMU_DPRINTF("SET_RTC command, invalid len: %d want: 4\n", in_len);
        return;
    }

    ti = (((uint32_t)in_data[0]) << 24) + (((uint32_t)in_data[1]) << 16)
         + (((uint32_t)in_data[2]) << 8) + in_data[3];
    s->tick_offset = ti - (qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)
                           / NANOSECONDS_PER_SECOND);
}

static void pmu_cmd_system_ready(PMUState *s,
                                 const uint8_t *in_data, uint8_t in_len,
                                 uint8_t *out_data, uint8_t *out_len)
{
        /* Do nothing */
}

static void pmu_cmd_get_version(PMUState *s,
                                const uint8_t *in_data, uint8_t in_len,
                                uint8_t *out_data, uint8_t *out_len)
{
        *out_len = 1;
        *out_data = 1; /* ??? Check what Apple does */
}

static void pmu_cmd_power_events(PMUState *s,
                                 const uint8_t *in_data, uint8_t in_len,
                                 uint8_t *out_data, uint8_t *out_len)
{
    if (in_len < 1) {
        PMU_DPRINTF("POWER EVENTS command, invalid len %d, want at least 1\n",
                    in_len);
        return;
    }
    switch(in_data[0]) {
    /* Dummies for now */
    case PMU_PWR_GET_POWERUP_EVENTS:
        *out_len = 2;
        out_data[0] = 0;
        out_data[1] = 0;
        break;
    case PMU_PWR_SET_POWERUP_EVENTS:
    case PMU_PWR_CLR_POWERUP_EVENTS:
        break;
    case PMU_PWR_GET_WAKEUP_EVENTS:
        *out_len = 2;
        out_data[0] = 0;
        out_data[1] = 0;
        break;
    case PMU_PWR_SET_WAKEUP_EVENTS:
    case PMU_PWR_CLR_WAKEUP_EVENTS:
        break;
    default:
        PMU_DPRINTF("POWER EVENTS unknown subcommand 0x%02x\n",
                    in_data[0]);
    }
}

/* description of commands */
typedef struct PMUCmdHandler {
    uint8_t command;
    const char *name;
    void (*handler)(PMUState *s,
                    const uint8_t *in_args, uint8_t in_len,
                    uint8_t *out_args, uint8_t *out_len);
} PMUCmdHandler;

static const PMUCmdHandler PMUCmdHandlers[] = {
    { PMU_INT_ACK, "INT ACK", pmu_cmd_int_ack },
    { PMU_SET_INTR_MASK, "SET INT MASK", pmu_cmd_set_int_mask },
    { PMU_ADB_CMD, "ADB COMMAND", pmu_cmd_adb },
    { PMU_ADB_POLL_OFF, "ADB POLL OFF", pmu_cmd_adb_poll_off },
    { PMU_RESET, "REBOOT", pmu_cmd_reset },
    { PMU_SHUTDOWN, "SHUTDOWN", pmu_cmd_shutdown },
    { PMU_READ_RTC, "GET RTC", pmu_cmd_get_rtc },
    { PMU_SET_RTC, "SET RTC", pmu_cmd_set_rtc },
    { PMU_SYSTEM_READY, "SYSTEM READY", pmu_cmd_system_ready },
    { PMU_GET_VERSION, "GET VERSION", pmu_cmd_get_version },
    { PMU_POWER_EVENTS, "POWER EVENTS", pmu_cmd_power_events },
    // .../...
};

static void pmu_dispatch_cmd(PMUState *s)
{
    unsigned int i;

    /* No response by default */
    s->cmd_rsp_sz = 0;

    for (i = 0; i < ARRAY_SIZE(PMUCmdHandlers); i++) {
        const PMUCmdHandler *desc = &PMUCmdHandlers[i];
        if (desc->command != s->cmd) {
            continue;
        }
        PMU_DPRINTF("handling command %s\n", desc->name);
        desc->handler(s, s->cmd_buf, s->cmd_buf_pos,
                      s->cmd_rsp, &s->cmd_rsp_sz);
        if (s->rsplen != -1 && s->rsplen != s->cmd_rsp_sz) {
            PMU_DPRINTF("Qemu internal cmd resp mismatch !\n");
        } else {
#ifdef DEBUG_PMU_PROTOCOL
            PMU_DPRINTF("sending %d resp bytes\n", s->cmd_rsp_sz);
#endif
        }
        return;
    }
    PMU_DPRINTF("Unknown PMU command %02x !\n", s->cmd);

    /* Manufacture fake response with 0's */
    if (s->rsplen == -1) {
            s->cmd_rsp_sz = 0;
    } else {
            s->cmd_rsp_sz = s->rsplen;
            memset(s->cmd_rsp, 0, s->rsplen);
    }
}

static void pmu_update(PMUState *s)
{
    /* Only react to changes in reg B */
    if (s->b == s->last_b) {
        return;
    }
    s->last_b = s->b;

    /* Check the TREQ / TACK state */
    switch (s->b & (TREQ | TACK)) {
    case TREQ:
        /* This is an ack release, handle it and bail out */
        s->b |= TACK;
        s->last_b = s->b;
#ifdef DEBUG_PMU_PROTOCOL
        PMU_DPRINTF("handshake: TREQ high, setting TACK\n");
#endif
        return;
    case TACK:
        /* This is a valid request, handle below */
        break;
    case TREQ | TACK:
        /* This is an idle state */
        return;
    default:
        /* Invalid state, log and ignore */
        PMU_DPRINTF("protocol error ! portB=0x%02x\n", s->b);
        return;
    }

    /* If we wanted to handle commands asynchronously, this is where
     * we would delay the clearing of TACK until we are ready to send
     * the response
     */

    /* We have a request, handshake TACK so we don't stay in
     * an invalid state. If we were concurrent with the OS we
     * should only do this after we grabbed the SR but that isn't
     * a problem here.
     */
#ifdef DEBUG_PMU_PROTOCOL
    PMU_DPRINTF("TREQ cleared, clearing TACK, state: %d\n", s->cmd_state);
#endif
    s->b &= ~TACK;
    s->last_b = s->b;

    /* Act according to state */
    switch(s->cmd_state) {
    case pmu_state_idle:
        if (!(s->acr & SR_OUT)) {
            PMU_DPRINTF("protocol error ! state idle, ACR reading\n");
            break;
        }
        s->cmd = s->sr;
        via_delay_set_sr_int(s);
        s->cmdlen = pmu_data_len[s->cmd][0];
        s->rsplen = pmu_data_len[s->cmd][1];
        s->cmd_buf_pos = 0;
        s->cmd_rsp_pos = 0;
        s->cmd_state = pmu_state_cmd;
#ifdef DEBUG_PMU_PROTOCOL
        PMU_DPRINTF("Got command byte %02x, clen=%d,rlen=%d\n",
                    s->cmd, s->cmdlen, s->rsplen);
#endif
        break;
    case pmu_state_cmd:
        if (!(s->acr & SR_OUT)) {
            PMU_DPRINTF("protocol error ! state cmd, ACR reading\n");
            break;
        }
        if (s->cmdlen == -1) {
#ifdef DEBUG_PMU_PROTOCOL
            PMU_DPRINTF("got cmd length byte: %d\n", s->sr);
#endif
            s->cmdlen = s->sr;
            if (s->cmdlen > sizeof(s->cmd_buf)) {
                PMU_DPRINTF("command too big (%d bytes)!\n", s->cmdlen);
            }
        } else if (s->cmd_buf_pos < sizeof(s->cmd_buf)) {
            s->cmd_buf[s->cmd_buf_pos++] = s->sr;
        }
        via_delay_set_sr_int(s);
        break;
    case pmu_state_rsp:
        if (s->acr & SR_OUT) {
            PMU_DPRINTF("protocol error ! state resp, ACR writing\n");
            break;
        }
        if (s->rsplen == -1) {
#ifdef DEBUG_PMU_PROTOCOL
            PMU_DPRINTF(" sending length byte: %d\n", s->cmd_rsp_sz);
#endif
            s->sr = s->cmd_rsp_sz;
            s->rsplen = s->cmd_rsp_sz;
        } else if (s->cmd_rsp_pos < s->cmd_rsp_sz) {
#ifdef DEBUG_PMU_PROTOCOL
            PMU_DPRINTF(" sending byte: %d/%d\n",
                        s->cmd_rsp_pos, s->rsplen);
#endif
            s->sr = s->cmd_rsp[s->cmd_rsp_pos++];
        }
        via_delay_set_sr_int(s);
        break;
    }

    /* Check for state completion */
    if (s->cmd_state == pmu_state_cmd && s->cmdlen == s->cmd_buf_pos) {
#ifdef DEBUG_PMU_PROTOCOL
        PMU_DPRINTF("Command reception complete, dispatching...\n");
#endif
        pmu_dispatch_cmd(s);
        s->cmd_state = pmu_state_rsp;
    }
    if (s->cmd_state == pmu_state_rsp && s->rsplen == s->cmd_rsp_pos) {
#ifdef DEBUG_PMU_PROTOCOL
        PMU_DPRINTF("Response send complete. IER=%02x\n", s->ier);
#endif
        s->cmd_state = pmu_state_idle;
    }
}

static uint64_t pmu_readb(void *opaque, hwaddr addr, unsigned size)
{
    PMUState *s = opaque;
    uint32_t val;

    addr = (addr >> 9) & 0xf;
    switch(addr) {
    case REG_B:
        val = s->b;
        if ((s->pcr & 0xe0) == 0x20 || (s->pcr & 0xe0) == 0x60) {
            s->ifr &= ~CB2_INT;
        }
        s->ifr &= ~CB1_INT;
        via_update_irq(s);
        break;
    case REG_A:
        val = s->a;
        if ((s->pcr & 0x0e) == 0x02 || (s->pcr & 0x02) == 0x06) {
            s->ifr &= ~CA2_INT;
        }
        s->ifr &= ~CA1_INT;
        via_update_irq(s);
        break;
    case REG_DIRB:
        val = s->dirb;
        break;
    case REG_DIRA:
        val = s->dira;
        break;
    case REG_T1CL:
        val = get_counter(&s->timers[0]) & 0xff;
        s->ifr &= ~T1_INT;
        via_update_irq(s);
        break;
    case REG_T1CH:
        val = get_counter(&s->timers[0]) >> 8;
        via_update_irq(s);
        break;
    case REG_T1LL:
        val = s->timers[0].latch & 0xff;
        break;
    case REG_T1LH:
        /* XXX: check this */
        val = (s->timers[0].latch >> 8) & 0xff;
        break;
    case REG_T2CL:
        val = get_counter(&s->timers[1]) & 0xff;
        s->ifr &= ~T2_INT;
        via_update_irq(s);
        break;
    case REG_T2CH:
        val = get_counter(&s->timers[1]) >> 8;
        break;
    case REG_SR:
        val = s->sr;
        s->ifr &= ~SR_INT;
        via_update_irq(s);
        break;
    case REG_ACR:
        val = s->acr;
        break;
    case REG_PCR:
        val = s->pcr;
        break;
    case REG_IFR:
        val = s->ifr;
        if (s->ifr & s->ier) {
            val |= 0x80;
        }
        break;
    case REG_IER:
        val = s->ier | 0x80;
        break;
    default:
    case REG_ANH:
        val = s->anh;
        break;
    }
#ifdef DEBUG_PMU_ALL_MMIO
    if (addr != REG_IFR || val != 0) {
        PMU_DPRINTF("read: reg=0x%x val=%02x\n", (int)addr, (int)val);
    }
#endif
    return val;
}

static void pmu_writeb(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    PMUState *s = opaque;

    addr = (addr >> 9) & 0xf;
#ifdef DEBUG_PMU_ALL_MMIO
    PMU_DPRINTF("write: reg=0x%x val=%02x\n", (int)addr, (int)val);
#endif
    switch(addr) {
    case REG_B:
        s->b = (val & s->dirb) | (s->b & ~s->dirb);
        if ((s->pcr & 0xe0) == 0x20 || (s->pcr & 0xe0) == 0x60) {
            s->ifr &= ~CB2_INT;
        }
        s->ifr &= ~CB1_INT;
        via_update_irq(s);
        pmu_update(s);
        break;
    case REG_A:
        s->a = (val & s->dira) | (s->b & ~s->dira);
        if ((s->pcr & 0x0e) == 0x02 || (s->pcr & 0x0e) == 0x06) {
            s->ifr &= ~CA2_INT;
        }
        s->ifr &= ~CA1_INT;
        via_update_irq(s);
        break;
    case REG_DIRB:
        s->dirb = val;
        break;
    case REG_DIRA:
        s->dira = val;
        break;
    case REG_T1CL:
        s->timers[0].latch = (s->timers[0].latch & 0xff00) | val;
        via_timer_update(s, &s->timers[0],
                         qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
        break;
    case REG_T1CH:
        s->timers[0].latch = (s->timers[0].latch & 0xff) | (val << 8);
        s->ifr &= ~T1_INT;
        set_counter(s, &s->timers[0], s->timers[0].latch);
        break;
    case REG_T1LL:
        s->timers[0].latch = (s->timers[0].latch & 0xff00) | val;
        via_timer_update(s, &s->timers[0],
                         qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
        break;
    case REG_T1LH:
        s->timers[0].latch = (s->timers[0].latch & 0xff) | (val << 8);
        s->ifr &= ~T1_INT;
        via_timer_update(s, &s->timers[0],
                         qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
        break;
    case REG_T2CL:
        s->timers[1].latch = (s->timers[1].latch & 0xff00) | val;
        break;
    case REG_T2CH:
        /* To ensure T2 generates an interrupt on zero crossing with the
           common timer code, write the value directly from the latch to
           the counter */
        s->timers[1].latch = (s->timers[1].latch & 0xff) | (val << 8);
        s->ifr &= ~T2_INT;
        set_counter(s, &s->timers[1], s->timers[1].latch);
        break;
    case REG_SR:
        s->sr = val;
        s->ifr &= ~SR_INT;
        break;
    case REG_ACR:
        s->acr = val;
        via_timer_update(s, &s->timers[0],
                         qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
        break;
    case REG_PCR:
        s->pcr = val;
        break;
    case REG_IFR:
        /* reset bits */
        s->ifr &= ~val;
        via_update_irq(s);
        break;
    case REG_IER:
        if (val & IER_SET) {
            /* set bits */
            s->ier |= val & 0x7f;
        } else {
            /* reset bits */
            s->ier &= ~val;
        }
        via_update_irq(s);
        break;
    default:
    case REG_ANH:
        s->anh = val;
        break;
    }
}


static const MemoryRegionOps pmu_mm_ops = {
    .read = pmu_readb,
    .write = pmu_writeb,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 1,
    },
};

#if 0
static bool cuda_timer_exist(void *opaque, int version_id)
{
    VIATimer *s = opaque;

    return s->timer != NULL;
}

static const VMStateDescription vmstate_cuda_timer = {
    .name = "cuda_timer",
    .version_id = 0,
    .minimum_version_id = 0,
    .fields = (VMStateField[]) {
        VMSTATE_UINT16(latch, VIATimer),
        VMSTATE_UINT16(counter_value, VIATimer),
        VMSTATE_INT64(load_time, VIATimer),
        VMSTATE_INT64(next_irq_time, VIATimer),
        VMSTATE_TIMER_PTR_TEST(timer, VIATimer, cuda_timer_exist),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_cuda = {
    .name = "cuda",
    .version_id = 4,
    .minimum_version_id = 4,
    .fields = (VMStateField[]) {
        VMSTATE_UINT8(a, PMUState),
        VMSTATE_UINT8(b, PMUState),
        VMSTATE_UINT8(last_b, PMUState),
        VMSTATE_UINT8(dira, PMUState),
        VMSTATE_UINT8(dirb, PMUState),
        VMSTATE_UINT8(sr, PMUState),
        VMSTATE_UINT8(acr, PMUState),
        VMSTATE_UINT8(last_acr, PMUState),
        VMSTATE_UINT8(pcr, PMUState),
        VMSTATE_UINT8(ifr, PMUState),
        VMSTATE_UINT8(ier, PMUState),
        VMSTATE_UINT8(anh, PMUState),
        VMSTATE_INT32(data_in_size, PMUState),
        VMSTATE_INT32(data_in_index, PMUState),
        VMSTATE_INT32(data_out_index, PMUState),
        VMSTATE_UINT8(autopoll, PMUState),
        VMSTATE_UINT8(autopoll_rate_ms, PMUState),
        VMSTATE_UINT16(adb_poll_mask, PMUState),
        VMSTATE_BUFFER(data_in, PMUState),
        VMSTATE_BUFFER(data_out, PMUState),
        VMSTATE_UINT32(tick_offset, PMUState),
        VMSTATE_STRUCT_ARRAY(timers, PMUState, 2, 1,
                             vmstate_cuda_timer, VIATimer),
        VMSTATE_TIMER_PTR(adb_poll_timer, PMUState),
        VMSTATE_TIMER_PTR(sr_delay_timer, PMUState),
        VMSTATE_END_OF_LIST()
    }
};
#endif

static void pmu_reset(DeviceState *dev)
{
    PMUState *s = VIA_PMU(dev);

    s->b = s->last_b = TACK | TREQ;
    s->a = 0;
    s->dirb = 0xff;
    s->dira = 0;
    s->sr = 0;
    s->acr = 0;
    s->pcr = 0;
    s->ifr = 0;
    s->ier = 0;
    s->anh = 0;

    s->timers[0].latch = 0xffff;
    set_counter(s, &s->timers[0], 0xffff);
    s->timers[1].latch = 0xffff;

    s->sr_delay_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, via_set_sr_int, s);

    s->cmd_state = pmu_state_idle;
    s->autopoll = 0;
}

static void pmu_realizefn(DeviceState *dev, Error **errp)
{
    PMUState *s = VIA_PMU(dev);
    struct tm tm;

    printf("PMU: realize, macio=%p\n", s->macio);

    s->timers[0].timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, via_timer1, s);
    s->timers[0].frequency = s->frequency;
    s->timers[1].timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, via_timer2, s);
    s->timers[1].frequency = (SCALE_US * 6000) / 4700;

    qemu_get_timedate(&tm, 0);
    s->tick_offset = (uint32_t)mktimegm(&tm) + RTC_OFFSET;

    s->adb_poll_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, pmu_adb_poll, s);
    s->autopoll_rate_ms = 20;
    s->adb_poll_mask = 0xffff;
}

static void pmu_initfn(Object *obj)
{
    SysBusDevice *d = SYS_BUS_DEVICE(obj);
    PMUState *s = VIA_PMU(obj);
    int i;

    memory_region_init_io(&s->mem, OBJECT(s),
                          &pmu_mm_ops, s, "via-pmu", 0x2000);
    sysbus_init_mmio(d, &s->mem);
    sysbus_init_irq(d, &s->via_irq);

    for (i = 0; i < ARRAY_SIZE(s->timers); i++) {
        s->timers[i].index = i;
    }

    PMU_DPRINTF("PMU: Creating ADB bus\n");
    qbus_create_inplace(&s->adb_bus, sizeof(s->adb_bus), TYPE_ADB_BUS,
                        DEVICE(obj), "adb.0");
}

static Property pmu_properties[] = {
    // XXX Add a "has ADB" property
    DEFINE_PROP_UINT64("frequency", PMUState, frequency, 0),
    DEFINE_PROP_PTR("macio", PMUState, macio),
    DEFINE_PROP_END_OF_LIST()
};

static void pmu_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = pmu_realizefn;
    dc->reset = pmu_reset;
//    dc->vmsd = &vmstate_cuda;
    dc->props = pmu_properties;
    set_bit(DEVICE_CATEGORY_BRIDGE, dc->categories);
}

static const TypeInfo pmu_type_info = {
    .name = TYPE_VIA_PMU,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(PMUState),
    .instance_init = pmu_initfn,
    .class_init = pmu_class_init,
};

static void pmu_register_types(void)
{
    type_register_static(&pmu_type_info);
}

type_init(pmu_register_types)
