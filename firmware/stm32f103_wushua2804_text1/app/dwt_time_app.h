#ifndef __dwt_time_app_H
#define __dwt_time_app_H

#include "bsp_system.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 包含 STM32F4 系列的 HAL 库头文件 */

/*
*********************************************************************************************************
* 函数声明
*********************************************************************************************************
*/

/**
 * @brief  初始化 DWT 时间戳计数器
 * @note   在使用延时函数前，必须先调用此函数
 * @retval HAL_OK: 初始化成功
 */
HAL_StatusTypeDef DWT_Timer_Init(void);

/**
 * @brief  读取当前 DWT 计数器值 (CPU Cycles)
 * @retval 当前 CYCCNT 寄存器的值
 */
uint32_t DWT_Get_CNT(void);

/**
 * @brief  获取系统时钟频率
 * @retval 时钟频率 (Hz)，例如 168000000
 */
uint32_t DWT_Get_System_Clock_Freq(void);

/**
 * @brief  获取 1us 所需的 CPU 计数值
 * @retval 计数值 (例如 168MHz 下为 168)
 */
uint32_t DWT_Get_UsCNT(void);

/**
 * @brief  微秒级延时 (阻塞式)
 * @param  us: 需要延迟的微秒数
 */
void DWT_Delay_us(uint32_t us);

/**
 * @brief  获取当前微秒时间戳
 * @note   需要在 DWT 溢出前再次调用 (对于 F407 168Mhz 约为 25秒) 以保证累积时间准确
 * @retval 从系统启动或初始化开始累计的微秒数
 */
uint32_t DWT_Get_Microsecond(void);

/*
*********************************************************************************************************
* Arduino 风格兼容函数
*********************************************************************************************************
*/

/**
 * @brief  毫秒延时 (封装 HAL_Delay)
 * @param  ms: 毫秒数
 */
void _delay(unsigned long ms);

/**
 * @brief  获取微秒时间戳 (封装 DWT_Get_Microsecond)
 * @retval 微秒数
 */
unsigned long _micros(void);

#ifdef __cplusplus
}
#endif


#endif
