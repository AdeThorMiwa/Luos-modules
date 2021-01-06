#ifndef GATE_H
#define GATE_H

#include "luos.h"

#define NB_DXL 8
extern uint8_t DXL_IDS[NB_DXL];

#define NB_FAN 0

void assert(uint8_t assertion);
void gate_init(void);
void gate_loop(void);
#endif /* GATE_H */
