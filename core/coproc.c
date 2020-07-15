#include "coproc.h"
#include "debug/debug.h"
#include "defines.h"
#include "emu.h"
#include "interrupt.h"
#include "schedule.h"

#include <string.h>

#define COPROC_MIN_REPEAT (48000000 / 10000)
#define COPROC_MAX_REPEAT (48000000 / 10)

coproc_state_t coproc;

static bool coproc_rxfull(void) {
    return (uint8_t)(coproc.rxwr + 1) == coproc.rxrd;
}

static bool coproc_rxempty(void) {
    return coproc.rxrd == coproc.rxwr;
}

static bool coproc_txfull(void) {
    return (uint8_t)(coproc.txwr + 1) == coproc.txrd;
}

static bool coproc_txempty(void) {
    return coproc.txrd == coproc.txwr;
}

static void coproc_update(void) {
    coproc.sts = coproc_txfull() << 4 |
        !coproc_rxempty() << 2;
    intrpt_set(INT_USART, coproc.msk & coproc.sts);
}

static void coproc_event(enum sched_item_id id) {
    if (!coproc_txempty() &&
        arm_usart_send(coproc.arm,
                       coproc.txbuf[coproc.txrd])) {
        coproc.txrd++;
        coproc.repeat = COPROC_MIN_REPEAT;
    }
    if (!coproc_rxfull() &&
        arm_usart_recv(coproc.arm,
                       &coproc.rxbuf[coproc.rxwr])) {
        coproc.rxwr++;
        coproc.repeat = COPROC_MIN_REPEAT;
    }
    coproc_update();
    sched_repeat(id, coproc.repeat);
    coproc.repeat <<= 1;
    if (likely(coproc.repeat > COPROC_MAX_REPEAT)) {
        coproc.repeat = COPROC_MAX_REPEAT;
    } else if (unlikely(coproc.repeat < COPROC_MIN_REPEAT)) {
        coproc.repeat = COPROC_MIN_REPEAT;
    }
}

/* Read from the coprocessor range of ports */
static uint8_t coproc_read(uint16_t addr, bool peek) {
    uint8_t val = 0;
    if (addr == 0x00) {
        if (!coproc_rxempty()) {
            val = coproc.rxbuf[coproc.rxrd++];
            coproc_update();
        }
    } else if (addr == 0x04) {
        val = coproc.msk;
    } else if (addr == 0x08) {
        val = coproc.sts;
    } else if (addr == 0x14) {
        val = !coproc_rxempty() << 0;
    } else if (addr == 0x18) {
    } else if (!peek) {
        printf("%04X -> %02X\n", addr, val);
    }
    return val;
}

/* Write to the coprocessor range of ports */
static void coproc_write(uint16_t addr, uint8_t val, bool poke) {
    if (addr == 0x00) {
        if (!coproc_txfull()) {
            coproc.txbuf[coproc.txwr++] = val;
            coproc_update();
        }
    } else if (addr == 0x04) {
        coproc.msk = val;
        intrpt_set(INT_USART, coproc.msk & coproc.sts);
    } else if (addr == 0x08) {
        coproc.sts &= ~val;
        intrpt_set(INT_USART, coproc.msk & coproc.sts);
    } else if (addr == 0x1C) {
        arm_reset(coproc.arm); // extremely arbitrary
    } else if (!poke) {
        printf("%04X <- %02X\n", addr, val);
    }
}

static const eZ80portrange_t pcoproc = {
    .read  = coproc_read,
    .write = coproc_write
};

eZ80portrange_t init_coproc(void) {
    gui_console_printf("[CEmu] Initialized Coprocessor Interface...\n");
    return pcoproc;
}

void coproc_reset(void) {
    gui_console_printf("[CEmu] Reset Coprocessor Interface...\n");
    arm_destroy(coproc.arm);
    memset(&coproc, 0, sizeof(coproc));
    coproc.arm = arm_create();
    sched.items[SCHED_COPROC].callback.event = coproc_event;
    sched.items[SCHED_COPROC].clock = CLOCK_48M;
    sched_set(SCHED_COPROC, 0);
}

bool coproc_load(const char *path) {
    return arm_load(coproc.arm, path);
}

void coproc_select(bool low) {
    arm_spi_sel(coproc.arm, low);
}

uint8_t coproc_transfer(uint8_t dir, uint32_t *value, uint8_t *bits) {
    bool send = dir & 1 << 1, recv = dir & 1 << 0;
    if (send && *bits < 8) {
        send = recv = false;
    }
    *value = send || recv ? arm_spi_xfer(coproc.arm, send ? *value : 0) : 0;
    *bits = recv << 3;
    return send << 3;
}
