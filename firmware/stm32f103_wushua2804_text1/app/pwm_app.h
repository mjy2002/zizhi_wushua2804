#ifndef __pwm_app_H
#define __pwm_app_H

#include "bsp_system.h"

// 定义通道映射，方便修改
#define MOTOR_TIM htim2

#define MOTOR_CHANNEL_U           TIM_CHANNEL_1
#define MOTOR_CHANNEL_V           TIM_CHANNEL_2
#define MOTOR_CHANNEL_W           TIM_CHANNEL_3

void PWM_Init_All(void);
void Set_MOTOR_Compare(uint32_t Channel, uint32_t CompareValue);

#endif

