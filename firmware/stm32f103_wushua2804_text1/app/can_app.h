#ifndef __can_app_H__

#define __can_app_H__



#ifdef __cplusplus

extern "C" {

#endif



#include "bsp_system.h"

/*

 * can_app.h

 *

 * CAN通信协议接口：

 *   广播命令:

 *     0x000  全局同步执行/启动 (DLC=0)

 *     0x001  全局停止电机 (DLC=0)

 *     0x003  全局清除同步缓存 (DLC=0)

 *   电机1控制:

 *     0x200  速度模式控制 (DLC=5, float32 rad/s + bit0 同步标志)

 *     0x201  位置模式控制 (DLC=5, int32 编码器计数值 + bit0 同步标志 + bit1 位置类型标志)

 *     0x202  请求状态反馈 (DLC=0)

 *     0x203  位置模式限速设置 (DLC=4, float32 rad/s)

 *     0x206  清除本地同步缓存 (DLC=0)

 *     0x208  (已合并至0x201)

 *   电机1状态:

 *     0x204  主状态帧 (DLC=8, int32位置 + float32速度)

 *     0x205  电流状态帧 (DLC=2, int16 mA)

 *     0x207  同步状态帧 (DLC=1, bit0缓存状态 + bit1-2运行模式)

 */



#ifndef CAN_APP_ENCODER_CPR

#define CAN_APP_ENCODER_CPR          4096.0f

#endif



#ifndef CAN_APP_CURRENT_SCALE

#define CAN_APP_CURRENT_SCALE        1000.0f

#endif



HAL_StatusTypeDef CAN_App_Init(void);

HAL_StatusTypeDef CAN_App_SendStd(uint16_t std_id, const uint8_t *data, uint8_t len);

HAL_StatusTypeDef CAN_App_SendStatus(void);

HAL_StatusTypeDef CAN_App_SendSyncStatus(void);



uint8_t CAN_App_IsRunning(void);

float CAN_App_GetPositionMaxSpeed(void);

void CAN_App_ProcessRxTask(void);



#ifdef __cplusplus

}

#endif



#endif

