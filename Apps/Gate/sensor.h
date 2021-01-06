#ifndef _SENSOR_H
#define _SENSOR_H

#include "luos.h"

void collect_data(module_t *src_module, uint8_t dst_module_sensor_id);

uint8_t get_next_dxl(uint8_t current);

#endif /* _SENSOR_H */