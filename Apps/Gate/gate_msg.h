#ifndef _GATE_MSG_H
#define _GATE_MSG_H

#include "stdint.h"

typedef enum {
    P4_DYNAMIXEL = 10,
    P4_FORCE = 20,
} p4_module_t;

typedef enum {
    P4_DXL_TORQUE_ENABLE = 24,
    P4_DXL_GOAL_POSITION = 30,
    P4_DXL_MOVING_SPEED = 32,
    P4_DXL_TORQUE_LIMIT = 34,
    P4_DXL_PRESENT_POSITION = 36,
    P4_DXL_PRESENT_TEMPERATURE = 43,
} dxl_value_t;

typedef enum {
    P4_DXL_NO_ERROR = 0,
} dxl_error_t;

uint8_t module_id_for_dxl_id(uint8_t dxl_id);

void make_present_pos_msg(char *buff, uint8_t buff_size, float *positions, uint8_t nb_dxl);
void make_present_temp_msg(char *buff, uint8_t buff_size, float *temperatures, uint8_t nb_dxl);

#endif