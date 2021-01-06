#ifndef GATE_H
#define GATE_H

#include "luos.h"

#define NB_DXL 8

#ifdef LEFT_ARM
static uint8_t DXL_IDS[NB_DXL] = {20, 21, 22, 23, 24, 25, 26, 27};
#else
static uint8_t DXL_IDS[NB_DXL] = {10, 11, 12, 13, 14, 15, 16, 17};
#endif

#define NB_FAN 3


void gate_init(void);
void gate_loop(void);
#endif /* GATE_H */
