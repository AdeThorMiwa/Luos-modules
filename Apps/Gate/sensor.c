#include "sensor.h"

void collect_data(module_t *src_module, uint8_t dst_module_sensor_id) 
{
    msg_t ask_pub_msg;
    ask_pub_msg.header.target_mode = ID;
    ask_pub_msg.header.target = dst_module_sensor_id;
    ask_pub_msg.header.cmd = ASK_PUB_CMD;
    ask_pub_msg.header.size = 0;

    luos_send(src_module, &ask_pub_msg);
}

uint8_t get_next_dxl(uint8_t current)
{
    for (uint8_t i = current + 1; i <= get_last_module(); i++)
    {
        if (type_from_id(i) == DYNAMIXEL_MOD)
        {
            return i;
        }
    }

    if (current != 0)
    {
        return get_next_dxl(0);
    }
    
    return 0;
}