#include "gate_msg.h"
#include "gate.h"

uint8_t module_id_for_dxl_id(uint8_t dxl_id)
{
    for (int i=0; i < NB_DXL; i++)
    {
        if (DXL_IDS[i] == dxl_id)
        {
            return i;
        }
    }
    assert(0);
    return 255;
}

void make_present_pos_msg(char *buff, uint8_t buff_size, float *positions, uint8_t nb_dxl) 
{
    memset(buff, '\0', buff_size);

    buff[0] = P4_DYNAMIXEL;
    buff[1] = P4_DXL_PRESENT_POSITION;
    buff[2] = nb_dxl;

    for (int i=0; i < nb_dxl; i++)
    {
        int j = 3 + i * (2 + sizeof(float));

        buff[j] = DXL_IDS[i];
        buff[j + 1] = P4_DXL_NO_ERROR;
        memcpy(&buff[j + 2], &positions[i], sizeof(float));
    }
}

void make_present_temp_msg(char *buff, uint8_t buff_size, float *temperatures, uint8_t nb_dxl) 
{
    memset(buff, '\0', buff_size);

    buff[0] = P4_DYNAMIXEL;
    buff[1] = P4_DXL_PRESENT_TEMPERATURE;
    buff[2] = nb_dxl;

    for (int i=0; i < nb_dxl; i++)
    {
        int j = 3 + i * (2 + sizeof(float));

        buff[j] = DXL_IDS[i];
        buff[j + 1] = P4_DXL_NO_ERROR;
        memcpy(&buff[j + 2], &temperatures[i], sizeof(float));
    }
}

