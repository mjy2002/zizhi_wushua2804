#ifndef __PID_TUNE_H
#define __PID_TUNE_H

#include "bsp_system.h"

/*
 * PID_TUNE_MODE: 选择当前串口调参目标
 *   1 = 角度环 (pid_motor_angle)
 *   2 = 速度环 (pid_motor_velocity)
 * 后续可扩展: 3 = 电流环 ...
 */
#define PID_TUNE_MODE   2

/*
 * PID_Tune_Parse
 * 解析 "P,I,D,target" 格式的字符串，更新对应环路的 PID 参数和目标值。
 * 返回值: 1=解析并更新成功, 0=解析失败
 */
uint8_t PID_Tune_Parse(char *cmd);

#endif
