#include "can_app.h"



/* ========================= CAN ID 定义 ========================= */



#define CAN_ID_BROADCAST_START       0x000U

#define CAN_ID_BROADCAST_STOP        0x001U

#define CAN_ID_BROADCAST_CLR_SYNC    0x003U



#define CAN_ID_MOTOR1_BASE           0x200U//指定的ID号

#define CAN_ID_M1_WRITE_SPEED        (CAN_ID_MOTOR1_BASE + 0x00U)   /* 0x200 */

#define CAN_ID_M1_WRITE_POSITION     (CAN_ID_MOTOR1_BASE + 0x01U)   /* 0x201 合并绝对/增量位置 */

#define CAN_ID_M1_READ_STATUS        (CAN_ID_MOTOR1_BASE + 0x02U)   /* 0x202 */

#define CAN_ID_M1_POS_MAX_SPEED      (CAN_ID_MOTOR1_BASE + 0x03U)   /* 0x203 */

#define CAN_ID_M1_STATUS_MAIN        (CAN_ID_MOTOR1_BASE + 0x04U)   /* 0x204 */

#define CAN_ID_M1_STATUS_CURRENT     (CAN_ID_MOTOR1_BASE + 0x05U)   /* 0x205 */

#define CAN_ID_M1_CLR_LOCAL_SYNC     (CAN_ID_MOTOR1_BASE + 0x06U)   /* 0x206 */

#define CAN_ID_M1_STATUS_SYNC        (CAN_ID_MOTOR1_BASE + 0x07U)   /* 0x207 */

/* 0x208 已废弃，增量式位置已合并至 0x201 (byte4 bit1 = 1) */



/* 驱动使能与停车动作已经移动到 YiFOC 控制层。 */

#ifndef CAN_APP_PI

#define CAN_APP_PI                   3.14159265358979323846f

#endif



#define CAN_APP_2PI                  (2.0f * CAN_APP_PI)



/* ========================= 接收环形缓冲区 ========================= */

#define CAN_RX_BUF_SIZE 16



typedef struct {

    uint16_t std_id;

    uint8_t  dlc;

    uint8_t  data[8];

} CAN_RxFrame_t;



static CAN_RxFrame_t s_can_rx_buf[CAN_RX_BUF_SIZE];

static volatile uint8_t s_rx_head = 0;

static uint8_t s_rx_tail = 0;



/* ========================= 同步缓存 ========================= */



typedef enum {

    SYNC_CMD_NONE = 0,

    SYNC_CMD_SPEED,

    SYNC_CMD_POSITION

} SyncCmdType_t;



typedef struct {

    SyncCmdType_t type;

    float         speed_rad_s;

    int32_t       position_count;

    uint8_t       position_is_inc;   /* 0=绝对式 1=增量式 */

} SyncCache_t;



static SyncCache_t s_sync_cache = { SYNC_CMD_NONE, 0.0f, 0, 0 };



/* ========================= PID限幅宏 ========================= */



#ifndef CAN_APP_SET_ANGLE_PID_LIMIT

#define CAN_APP_SET_ANGLE_PID_LIMIT(v) do { pid_motor_angle.voltage_limit = (v); } while (0)

#endif



/* ========================= 外部变量 / 函数 ========================= */

extern CAN_HandleTypeDef hcan;

extern float as5600_angle;
extern float as5600_speed;
extern DQCurrent_s current;
extern MotionControlType controller;

/* ========================= 模块内部状态 ========================= */

static volatile float s_position_max_speed_rad_s = 0.0f;

/* ========================= 字节序转换 ========================= */



static uint32_t can_get_u32_le(const uint8_t *p)

{

    return ((uint32_t)p[0])       |

           ((uint32_t)p[1] << 8)  |

           ((uint32_t)p[2] << 16) |

           ((uint32_t)p[3] << 24);

}



static int32_t can_get_i32_le(const uint8_t *p)

{

    return (int32_t)can_get_u32_le(p);

}



static float can_get_f32_le(const uint8_t *p)

{

    union {

        uint32_t u32;

        float f32;

    } v;



    v.u32 = can_get_u32_le(p);

    return v.f32;

}



static void can_put_u16_le(uint8_t *p, uint16_t v)

{

    p[0] = (uint8_t)(v & 0xFFU);

    p[1] = (uint8_t)((v >> 8) & 0xFFU);

}



static void can_put_i16_le(uint8_t *p, int16_t v)

{

    can_put_u16_le(p, (uint16_t)v);

}



static void can_put_u32_le(uint8_t *p, uint32_t v)

{

    p[0] = (uint8_t)(v & 0xFFU);

    p[1] = (uint8_t)((v >> 8) & 0xFFU);

    p[2] = (uint8_t)((v >> 16) & 0xFFU);

    p[3] = (uint8_t)((v >> 24) & 0xFFU);

}



static void can_put_i32_le(uint8_t *p, int32_t v)

{

    can_put_u32_le(p, (uint32_t)v);

}



static void can_put_f32_le(uint8_t *p, float v)

{

    union {

        uint32_t u32;

        float f32;

    } u;



    u.f32 = v;

    can_put_u32_le(p, u.u32);

}



/* ========================= 位置单位转换 ========================= */



static float can_count_to_rad(int32_t count)

{

    return ((float)count) * CAN_APP_2PI / CAN_APP_ENCODER_CPR;

}



static int32_t can_rad_to_count(float rad)

{

    float count_f = rad * CAN_APP_ENCODER_CPR / CAN_APP_2PI;



    if (count_f > 2147483647.0f) {

        return 2147483647;

    }



    if (count_f < -2147483648.0f) {

        return (int32_t)0x80000000;

    }



    return (int32_t)lroundf(count_f);

}



static int16_t can_float_to_i16_sat(float x)

{

    if (x > 32767.0f) {

        return 32767;

    }



    if (x < -32768.0f) {

        return -32768;

    }



    return (int16_t)lroundf(x);

}



/* ========================= 同步缓存操作 ========================= */



static void can_app_clear_sync_cache(void)

{

    s_sync_cache.type = SYNC_CMD_NONE;

    s_sync_cache.speed_rad_s = 0.0f;

    s_sync_cache.position_count = 0;

    s_sync_cache.position_is_inc = 0;

}



static void can_app_cache_speed(float speed_rad_s)

{

    s_sync_cache.type = SYNC_CMD_SPEED;

    s_sync_cache.speed_rad_s = speed_rad_s;

}



static void can_app_cache_position(int32_t position_count, uint8_t is_inc)

{

    s_sync_cache.type = SYNC_CMD_POSITION;

    s_sync_cache.position_count = position_count;

    s_sync_cache.position_is_inc = is_inc;

}



static uint8_t can_app_has_sync_cache(void)

{

    return (s_sync_cache.type != SYNC_CMD_NONE) ? 1U : 0U;

}



static void can_app_exec_sync_cache(void)
{
    if (s_sync_cache.type == SYNC_CMD_SPEED)
    {
        Motor_Control_SetVelocity(s_sync_cache.speed_rad_s);
    }
    else if (s_sync_cache.type == SYNC_CMD_POSITION)
    {
        float position_rad;

        if (s_sync_cache.position_is_inc)
        {
            position_rad = as5600_angle +
                           can_count_to_rad(s_sync_cache.position_count);
        }
        else
        {
            position_rad = can_count_to_rad(s_sync_cache.position_count);
        }

        Motor_Control_SetPosition(position_rad);
    }

    s_sync_cache.type = SYNC_CMD_NONE;
}

/* ========================= 电机启停 ========================= */

static void can_app_start_motor(void)
{
    if (can_app_has_sync_cache())
    {
        can_app_exec_sync_cache();
    }
    else
    {
        Motor_Control_Start();
    }
}

static void can_app_stop_motor(void)
{
    /*
     * 0x001 保留为紧急停止：立即关闭驱动使能，自由滑行。
     * 正常停车请发送速度0，控制层会执行积分清零和主动制动。
     */
    Motor_Control_RequestStop(MOTOR_STOP_COAST);
    can_app_clear_sync_cache();
}

/* ========================= CAN 发送函数 ========================= */



HAL_StatusTypeDef CAN_App_SendStd(uint16_t std_id, const uint8_t *data, uint8_t len)

{

    CAN_TxHeaderTypeDef tx_header;

    uint32_t tx_mailbox;

    uint8_t tx_data[8] = {0};



    if (len > 8U) {

        return HAL_ERROR;

    }



    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0U) {

        return HAL_BUSY;

    }



    if ((data != NULL) && (len > 0U)) {

        memcpy(tx_data, data, len);

    }



    tx_header.StdId = std_id & 0x7FFU;

    tx_header.ExtId = 0;

    tx_header.IDE = CAN_ID_STD;

    tx_header.RTR = CAN_RTR_DATA;

    tx_header.DLC = len;

    tx_header.TransmitGlobalTime = DISABLE;



    return HAL_CAN_AddTxMessage(&hcan, &tx_header, tx_data, &tx_mailbox);

}



HAL_StatusTypeDef CAN_App_SendStatus(void)

{

    uint8_t data_main[8];

    uint8_t data_current[2];



    int32_t position_count = can_rad_to_count(as5600_angle);

    float velocity_rad_s = as5600_speed;

    int16_t current_ma = can_float_to_i16_sat(current.q * CAN_APP_CURRENT_SCALE);



    can_put_i32_le(&data_main[0], position_count);

    can_put_f32_le(&data_main[4], velocity_rad_s);



    can_put_i16_le(&data_current[0], current_ma);



    HAL_StatusTypeDef ret = CAN_App_SendStd(CAN_ID_M1_STATUS_MAIN, data_main, 8U);



    if (ret != HAL_OK) {

        return ret;

    }



    return CAN_App_SendStd(CAN_ID_M1_STATUS_CURRENT, data_current, 2U);

}



HAL_StatusTypeDef CAN_App_SendSyncStatus(void)

{

    uint8_t data = 0;



    if (can_app_has_sync_cache()) {

        data |= 0x01U;

    }



    if (Motor_Control_IsRunning()) {

        if (controller == Type_velocity) {

            data |= 0x02U;

        } else if (controller == Type_angle) {

            data |= 0x04U;

        }

    }



    return CAN_App_SendStd(CAN_ID_M1_STATUS_SYNC, &data, 1U);

}



/* ========================= CAN 帧处理 ========================= */



static void can_app_process_frame(uint16_t std_id, const uint8_t *data, uint8_t dlc)

{

    my_printf(&huart1, "RX ID: 0x%03X, DLC: %d, Raw Data: ", std_id, dlc);

    for (uint8_t i = 0; i < dlc; i++) {

        my_printf(&huart1, "%02X ", data[i]);

    }

    my_printf(&huart1, "\r\n");



    switch (std_id)

    {

        case CAN_ID_BROADCAST_START:

        {

            if (dlc == 0U) {

                can_app_start_motor();

                my_printf(&huart1, ">> Cmd: Global Sync/Start\r\n");

            }

        } break;



        case CAN_ID_BROADCAST_STOP:

        {

            if (dlc == 0U) {

                can_app_stop_motor();

                my_printf(&huart1, ">> Cmd: Global Stop\r\n");

            }

        } break;



        case CAN_ID_BROADCAST_CLR_SYNC:

        {

            if (dlc == 0U) {

                can_app_clear_sync_cache();

                my_printf(&huart1, ">> Cmd: Global Clear Sync Cache\r\n");

            }

        } break;



        case CAN_ID_M1_WRITE_SPEED:

        {

            if (dlc >= 4U) {

                float speed_rad_s = can_get_f32_le(&data[0]);

                uint8_t sync_flag = (dlc >= 5U) ? (data[4] & 0x01U) : 0U;



                if (sync_flag) {

                    can_app_cache_speed(speed_rad_s);

                    my_printf(&huart1, ">> Cmd: Speed Mode (SYNC CACHED), Target = %.2f rad/s\r\n", speed_rad_s);

                } else {

                    Motor_Control_SetVelocity(speed_rad_s);

                    if (fabsf(speed_rad_s) <= 0.02f) {
                        my_printf(&huart1, ">> Cmd: Speed 0 -> Active Brake\r\n");
                    } else {
                        my_printf(&huart1, ">> Cmd: Speed Mode (NOW), Target = %.2f rad/s\r\n", speed_rad_s);
                    }

                }

            }

        } break;



        /* 0x201: 位置模式 (绝对式/增量式合并)

         * byte4 bit0 = 同步标志, bit1 = 位置类型 (0=绝对 1=增量) */

        case CAN_ID_M1_WRITE_POSITION:

        {

            if (dlc >= 4U) {

                int32_t position_count = can_get_i32_le(&data[0]);

                uint8_t flags = (dlc >= 5U) ? data[4] : 0U;

                uint8_t sync_flag = flags & 0x01U;

                uint8_t pos_type  = (flags >> 1) & 0x01U;



                if (sync_flag) {

                    can_app_cache_position(position_count, pos_type);

                    my_printf(&huart1, ">> Cmd: Pos(%s, SYNC CACHED), Count: %d\r\n",

                              pos_type ? "INC" : "ABS", position_count);

                } else {

                    float position_rad;

                    if (pos_type) {

                        position_rad = as5600_angle + can_count_to_rad(position_count);

                    } else {

                        position_rad = can_count_to_rad(position_count);

                    }

                    Motor_Control_SetPosition(position_rad);

                    my_printf(&huart1, ">> Cmd: Pos(%s, NOW), Target = %.2f rad, Count: %d\r\n",

                              pos_type ? "INC" : "ABS", position_rad, position_count);

                }

            }

        } break;



        case CAN_ID_M1_READ_STATUS:

        {

            if (dlc == 0U) {

                (void)CAN_App_SendStatus();

                (void)CAN_App_SendSyncStatus();

                my_printf(&huart1, ">> Cmd: Read Status\r\n");

            }

        } break;



        case CAN_ID_M1_POS_MAX_SPEED:

        {

            if (dlc >= 4U) {

                float max_speed_rad_s = can_get_f32_le(&data[0]);

                s_position_max_speed_rad_s = max_speed_rad_s;

                CAN_APP_SET_ANGLE_PID_LIMIT(max_speed_rad_s);

                my_printf(&huart1, ">> Cmd: Pos Max Speed = %.2f rad/s\r\n", max_speed_rad_s);

            }

        } break;



        case CAN_ID_M1_CLR_LOCAL_SYNC:

        {

            if (dlc == 0U) {

                can_app_clear_sync_cache();

                my_printf(&huart1, ">> Cmd: Clear Local Sync Cache\r\n");

            }

        } break;



        case CAN_ID_M1_STATUS_SYNC:

        {

            my_printf(&huart1, ">> Cmd: Ignored STATUS_SYNC frame\r\n");

        } break;



        default:

        {

            my_printf(&huart1, ">> Cmd: UNKNOWN ID!\r\n");

        } break;

    }

}



/* ========================= CAN 接收中断 ========================= */



void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *phcan)
{

    CAN_RxHeaderTypeDef rx_header;

    uint8_t rx_data[8];



    if (phcan->Instance != CAN1) return;



    if (HAL_CAN_GetRxMessage(phcan, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK) {

        if ((rx_header.IDE == CAN_ID_STD) && (rx_header.RTR == CAN_RTR_DATA)) {

            uint8_t next_head = (s_rx_head + 1) % CAN_RX_BUF_SIZE;



            if (next_head != s_rx_tail) {

                s_can_rx_buf[s_rx_head].std_id = rx_header.StdId;

                s_can_rx_buf[s_rx_head].dlc = rx_header.DLC;

                memcpy(s_can_rx_buf[s_rx_head].data, rx_data, rx_header.DLC);

                s_rx_head = next_head;

            }

        }

    }

}



void CAN_App_ProcessRxTask(void)

{

    while (s_rx_tail != s_rx_head) {

        can_app_process_frame(s_can_rx_buf[s_rx_tail].std_id,

                              s_can_rx_buf[s_rx_tail].data,

                              s_can_rx_buf[s_rx_tail].dlc);

        s_rx_tail = (s_rx_tail + 1) % CAN_RX_BUF_SIZE;

    }

}



/* ========================= CAN 初始化 ========================= */



HAL_StatusTypeDef CAN_App_Init(void)

{

    CAN_FilterTypeDef filter;



    memset(&filter, 0, sizeof(filter));



    filter.FilterBank = 0;

    filter.FilterMode = CAN_FILTERMODE_IDMASK;

    filter.FilterScale = CAN_FILTERSCALE_32BIT;



    filter.FilterIdHigh = 0x0000;

    filter.FilterIdLow = 0x0000;

    filter.FilterMaskIdHigh = 0x0000;

    filter.FilterMaskIdLow = 0x0000;



    filter.FilterFIFOAssignment = CAN_RX_FIFO0;

    filter.FilterActivation = ENABLE;



#if defined(CAN_FILTER_FIFO0)

    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;

#endif



#if defined(CAN2)

    filter.SlaveStartFilterBank = 14;

#endif

    if (HAL_CAN_ConfigFilter(&hcan, &filter) != HAL_OK) {
        return HAL_ERROR;

    }



    if (HAL_CAN_Start(&hcan) != HAL_OK) {

        return HAL_ERROR;

    }



    if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {

        return HAL_ERROR;

    }



    /*
     * 电机初始模式由 init_all() 设置。
     * CAN初始化只建立通信，不在这里改变速度/位置控制模式。
     */
    can_app_clear_sync_cache();



    return HAL_OK;

}



uint8_t CAN_App_IsRunning(void)
{

    return Motor_Control_IsRunning();

}



float CAN_App_GetPositionMaxSpeed(void)
{

    return s_position_max_speed_rad_s;

}

