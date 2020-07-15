#ifndef COPROC_H
#define COPROC_H

#include "arm/arm.h"
#include "port.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct coproc_state {
    arm_t *arm;
    uint32_t repeat;
    uint8_t txrd, txwr, txbuf[256];
    uint8_t rxrd, rxwr, rxbuf[256];
    uint8_t msk, sts;
} coproc_state_t;

#ifdef __cplusplus
extern "C" {
#endif

extern coproc_state_t state;

eZ80portrange_t init_coproc(void);
void coproc_reset(void);
bool coproc_load(const char *path);
void coproc_select(bool low);
uint8_t coproc_transfer(uint8_t dir, uint32_t *value, uint8_t *bits);

#ifdef __cplusplus
}
#endif

#endif
