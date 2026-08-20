/*

 * @Author: yihazui6 1319743179@qq.com

 * @Date: 2026-05-11 10:43:23

 * @LastEditors: yihazui6 1319743179@qq.com

 * @LastEditTime: 2026-05-11 11:27:00

 * @FilePath: \stm32f103_wushua2804_text1\app\bsp_system.h

 * @Description: ÕâÊÇÄ¬ÈÏÉèÖÃ,ÇëÉèÖÃ`customMade`, ´ò¿ªkoroFileHeader²é¿´ÅäÖÃ ½øÐÐÉèÖÃ: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE

 */

#ifndef __bsp_system_H

#define __bsp_system_H



//Í·ï¿½Ä¼ï¿½ï¿½ï¿½ï¿½ï¿½



#include "main.h"

#include "string.h"

#include "stdio.h"

#include "stdarg.h"

#include <math.h> // ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ë£¬ï¿½ï¿½ï¿½ï¿½ pow ï¿½ï¿½ï¿½ï¿½



#include "scheduler.h"

#include "usart.h"

#include "uart_app.h"

#include "oled_app.h"

#include "oled.h"

#include "key_app.h"

#include "led_app.h"

#include "as5600_app.h"

#include "low_lvbo_app.h"

#include "dwt_time_app.h"

#include "pwm_app.h"

#include "pid.h"

#include "pid_tune.h"

#include "YiFOC.h"

#include "FOC_angle.h"

#include "adc_app.h"

#include "foc_current_math.h"

#include "can_app.h"

#include "foc_flash.h"



extern UART_HandleTypeDef huart1;

extern TIM_HandleTypeDef htim2;

extern I2C_HandleTypeDef hi2c1;

extern I2C_HandleTypeDef hi2c2;

extern TIM_HandleTypeDef htim4;

extern ADC_HandleTypeDef hadc1;



extern uint8_t uart1_rx_buff[512];

extern uint8_t uart2_rx_buff[512];

extern uint8_t uart3_rx_buff[512];

extern uint16_t AD_Value[2];



extern uint8_t moshi;



extern float as5600_speed;

extern int Dir1;
extern int pp1;
extern float zero_electric_Angle1;
extern float voltage_power_supply1;   // motor1ï¿½ï¿½ï¿½ï¿½

extern float target;

extern float current_sp;

extern DQCurrent_s current;

extern Dir_axis voltage;





#endif



