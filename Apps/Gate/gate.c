#include "main.h"
#include "gate.h"
#include "sensor.h"
#include "gate_msg.h"
#include "luos_board.h"

#ifdef LEFT_ARM
uint8_t DXL_IDS[NB_DXL] = {20, 21, 22, 23, 24, 25, 26, 27};
#else
uint8_t DXL_IDS[NB_DXL] = {10, 11, 12, 13, 14, 15, 16, 17};
#endif

#define SENSOR_POLL_PERIOD 1 // (in ms)
#define SERIAL_PRINT_POS_PERIOD 8 // (in ms)
#define SERIAL_PRINT_TEMP_PERIOD 1000 // (in ms)

#define STRINGIFY(s) STRINGIFY1(s)
#define STRINGIFY1(s) #s

#define RECV_BUFF_SIZE 64
#define RECV_RING_BUFFER_SIZE 2
volatile uint8_t recv_buff[RECV_RING_BUFFER_SIZE][RECV_BUFF_SIZE] = {0};
static volatile uint8_t nb_recv_msg = 0;
static volatile uint8_t recv_buff_read_index = 0;
static volatile uint8_t recv_buff_write_index = 0;

#define SEND_BUFF_SIZE 64
uint8_t send_buff[SEND_BUFF_SIZE] = {0};

float present_positions[NB_DXL] = {0.0};
float present_temperatures[NB_DXL] = {0.0};

#define NB_MODULES (NB_DXL + NB_FAN + 1)


module_t *my_module;


void assert(uint8_t assertion)
{
    if (assertion == 0)
    {
        status_led(1);
        while (1) ;
    }
}

int serial_write(uint8_t *data, int len)
{
    // TODO: Should be done as interrupt?
    for (unsigned short i = 0; i < len; i++)
    {
        while (!LL_USART_IsActiveFlag_TXE(USART3))
            ;
        LL_USART_TransmitData8(USART3, *(data + i));
    }
    return 0;
}

void USART3_4_IRQHandler(void)
{
    // check if we receive an IDLE on usart3
    if (LL_USART_IsActiveFlag_IDLE(USART3))
    {        
        LL_USART_ClearFlag_IDLE(USART3);

        // reset DMA
        __disable_irq();

        nb_recv_msg++;
        recv_buff_write_index++;

        if (recv_buff_write_index == RECV_RING_BUFFER_SIZE)
        {
            recv_buff_write_index = 0;
        }

        assert(nb_recv_msg < RECV_RING_BUFFER_SIZE);

        LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);
        LL_DMA_SetM2MDstAddress(DMA1, LL_DMA_CHANNEL_3, (uint32_t)recv_buff[recv_buff_write_index]);
        LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, RECV_BUFF_SIZE);
        LL_DMA_SetM2MSrcAddress(DMA1, LL_DMA_CHANNEL_3, (uint32_t)&USART3->RDR);
        LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_3, (uint32_t)recv_buff[recv_buff_write_index]);
        LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3);
        LL_USART_EnableDMAReq_RX(USART3);

        __enable_irq();
    }
}

void msg_cb(module_t *module, msg_t *msg)
{
    uint16_t sensor_id = msg->header.source;

    if (type_from_id(sensor_id) == DYNAMIXEL_MOD) 
    {
        uint16_t dxl_id = sensor_id - 2;
        assert (dxl_id < NB_DXL);

        switch (msg->header.cmd)
        {
        case ANGULAR_POSITION:
            if (msg->header.size == sizeof(float))
            {
                memcpy((float *)&present_positions[dxl_id], msg->data, msg->header.size);
            }
            break;
        
        case TEMPERATURE:
            if (msg->header.size == sizeof(float))
            {
                memcpy((float *)&present_temperatures[dxl_id], msg->data, msg->header.size);
            }
            break;

        case GRAVITY_VECTOR:
            if (msg->header.size == sizeof(int))
            {
                int nb_dxl_pos;
                memcpy(&nb_dxl_pos, msg->data, msg->header.size);

                sprintf((char *)send_buff, "%d ----", nb_dxl_pos);
                serial_write(send_buff, SEND_BUFF_SIZE); 
            }
            break;

        default:
            return;
        }
    }
}

void send_dxl_messages(uint8_t *src_msg, uint8_t data_size, module_register_t cmd)
{
    uint8_t nb_dxl = src_msg[2];
    assert(nb_dxl <= NB_DXL);

    for (int i=0; i < nb_dxl; i++)
    {
        uint8_t *data = src_msg + 3 + i * (1 + data_size);

        uint8_t dst_id = module_id_for_dxl_id(data[0]);
        assert(dst_id < NB_DXL);

        msg_t dxl_msg;
        dxl_msg.header.target_mode = IDACK;
        dxl_msg.header.target = dst_id + 2;                
        dxl_msg.header.cmd = cmd;

        dxl_msg.header.size = data_size;
        for (int j=0; j<data_size; j++)
        {
            dxl_msg.data[j] = data[j + 1];
        }
        luos_send(my_module, &dxl_msg);
    }
}

void handle_message(uint8_t *msg)
{
    if (msg[0] == P4_DYNAMIXEL)
    {
        if (msg[1] == P4_DXL_TORQUE_ENABLE)
        {
            send_dxl_messages(msg, sizeof(uint8_t), COMPLIANT);
        }
        else if (msg[1] == P4_DXL_GOAL_POSITION)
        {
            send_dxl_messages(msg, sizeof(float), ANGULAR_POSITION);
        }
        else if (msg[1] == P4_DXL_MOVING_SPEED)
        {
            send_dxl_messages(msg, sizeof(float), ANGULAR_SPEED);
        }
        else if (msg[2] == P4_DXL_TORQUE_LIMIT)
        {
            send_dxl_messages(msg, sizeof(float), RATIO);
        }
    }
}

void handle_inbound_messages()
{
    while (nb_recv_msg > 0)
    {
        handle_message((uint8_t *)recv_buff[recv_buff_read_index]);

        recv_buff_read_index++;

        if (recv_buff_read_index == RECV_RING_BUFFER_SIZE)
        {
            recv_buff_read_index = 0;
        }

        __disable_irq();
        nb_recv_msg--;
        __enable_irq();
    }
}

void gate_init(void)
{
    LL_USART_ClearFlag_IDLE(USART3);
    LL_USART_EnableIT_IDLE(USART3);
    NVIC_DisableIRQ(DMA1_Channel2_3_IRQn);
    LL_DMA_DisableIT_TC(DMA1, LL_DMA_CHANNEL_3);
    LL_DMA_DisableIT_HT(DMA1, LL_DMA_CHANNEL_3);
    LL_DMA_DisableIT_TE(DMA1, LL_DMA_CHANNEL_3);
    LL_DMA_SetM2MDstAddress(DMA1, LL_DMA_CHANNEL_3, (uint32_t)recv_buff[recv_buff_write_index]);
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, RECV_BUFF_SIZE);
    LL_DMA_SetM2MSrcAddress(DMA1, LL_DMA_CHANNEL_3, (uint32_t)&USART3->RDR);
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3);
    LL_USART_EnableDMAReq_RX(USART3);

    #ifdef ORBITA
    my_module = luos_module_create(msg_cb, GATE_MOD, "r_head", STRINGIFY(VERSION));
    #else
    my_module = luos_module_create(msg_cb, GATE_MOD, "gate", STRINGIFY(VERSION));
    #endif
}

void gate_loop(void)
{
    static volatile uint8_t detection_done = 0;
    static uint8_t poll_dxl_id = 0;

    static uint32_t last_sensor_poll = 0;
    static uint32_t last_serial_print_pos = 0;
    static uint32_t last_serial_print_temp = 0;

    if (!detection_done)
    {
        while (1)
        {
            HAL_Delay(5000);
            detect_modules(my_module);

            if (get_last_module() != NB_MODULES)
            {
                sprintf((char *)recv_buff, "modules founds: %d       ", get_last_module());
                serial_write((uint8_t *)recv_buff, RECV_BUFF_SIZE);
            }
            else
            {
                break;
            }
        }
        detection_done = 1;
    }

    if (detection_done)
    {
        uint32_t t = HAL_GetTick();

        handle_inbound_messages();

        if ((t - last_sensor_poll) >= SENSOR_POLL_PERIOD) 
        {
            poll_dxl_id = get_next_dxl(poll_dxl_id);
            if (poll_dxl_id != 0) 
            {
                // collect_data(my_module, poll_dxl_id);
                last_sensor_poll = HAL_GetTick();
            }
        }

        if ((t - last_serial_print_pos) >= SERIAL_PRINT_POS_PERIOD)
        {
            // make_present_pos_msg((char *)send_buff, SEND_BUFF_SIZE, (float *)present_positions, NB_DXL);
            // serial_write(send_buff, SEND_BUFF_SIZE); 
            last_serial_print_pos = HAL_GetTick();
        }

        if ((t - last_serial_print_temp) >= SERIAL_PRINT_TEMP_PERIOD)
        {
            // make_present_temp_msg((char *)send_buff, SEND_BUFF_SIZE, (float *)present_temperatures, NB_DXL);
            // serial_write(send_buff, SEND_BUFF_SIZE); 
            last_serial_print_temp = HAL_GetTick();
        }
    }
}