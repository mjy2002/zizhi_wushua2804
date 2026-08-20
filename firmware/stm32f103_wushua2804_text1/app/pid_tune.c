#include "pid_tune.h"
#include <stdlib.h>
#include <string.h>

/*
 * PID_Tune_Parse: 解析串口收到的 "P,I,D,target" 字符串
 *
 * 解析规则:
 *   - 逗号分隔 4 个浮点数
 *   - 支持数字前后有空格
 *   - 支持末尾 \r\n
 *   - 必须精确解析到 4 个 float 才允许更新
 *   - 解析失败时不动原参数
 *
 * 根据 PID_TUNE_MODE 决定更新哪个环路:
 *   1 = pid_motor_angle  + target
 *   2 = pid_motor_velocity + target (速度环目标)
 *
 * 安全性: 先解析到临时变量，确认全部合法后再关闭中断一次性更新。
 */
uint8_t PID_Tune_Parse(char *cmd)
{
    float p, i, d, tgt;
    char *endptr;

    /* ---- 去除前导空白 ---- */
    while (*cmd == ' ' || *cmd == '\t') cmd++;

    /* ---- 解析 P ---- */
    p = strtof(cmd, &endptr);
    if (endptr == cmd) goto PARSE_FAIL;
    cmd = endptr;
    while (*cmd == ' ' || *cmd == '\t') cmd++;
    if (*cmd != ',') goto PARSE_FAIL;
    cmd++; /* skip ',' */

    /* ---- 解析 I ---- */
    i = strtof(cmd, &endptr);
    if (endptr == cmd) goto PARSE_FAIL;
    cmd = endptr;
    while (*cmd == ' ' || *cmd == '\t') cmd++;
    if (*cmd != ',') goto PARSE_FAIL;
    cmd++; /* skip ',' */

    /* ---- 解析 D ---- */
    d = strtof(cmd, &endptr);
    if (endptr == cmd) goto PARSE_FAIL;
    cmd = endptr;
    while (*cmd == ' ' || *cmd == '\t') cmd++;
    if (*cmd != ',') goto PARSE_FAIL;
    cmd++; /* skip ',' */

    /* ---- 解析 target ---- */
    tgt = strtof(cmd, &endptr);
    if (endptr == cmd) goto PARSE_FAIL;

    /* ---- 4 个参数全部解析成功，关中断后一次性更新 ---- */
    __disable_irq();

#if (PID_TUNE_MODE == 1)
    pid_motor_angle.P = p;
    pid_motor_angle.I = i;
    pid_motor_angle.D = d;
    target = tgt;
#elif (PID_TUNE_MODE == 2)
    pid_motor_velocity.P = p;
    pid_motor_velocity.I = i;
    pid_motor_velocity.D = d;
    target = tgt;
#endif

    __enable_irq();

    /* ---- 打印更新后的参数 ---- */
#if (PID_TUNE_MODE == 1)
    my_printf(&huart1,
              "[PID_ANGL] P=%.4f I=%.4f D=%.4f target=%.4f\r\n",
              pid_motor_angle.P,
              pid_motor_angle.I,
              pid_motor_angle.D,
              target);
#elif (PID_TUNE_MODE == 2)
    my_printf(&huart1,
              "[PID_VEL]  P=%.4f I=%.4f D=%.4f target=%.4f\r\n",
              pid_motor_velocity.P,
              pid_motor_velocity.I,
              pid_motor_velocity.D,
              target);
#endif

    return 1;

PARSE_FAIL:
    my_printf(&huart1,
              "[PID_TUNE] Parse ERR! Format: P,I,D,target\r\n");
    return 0;
}
