#ifndef __YIFOC_H
#define __YIFOC_H

#include <stdint.h>

typedef struct
{
    float d;
    float q;
} DQCurrent_s;

typedef struct
{
    float d;
    float q;
} Dir_axis;

#include "bsp_system.h"

/*============================================================
 * 低速速度模式总开关
 *============================================================*/
#define LOW_SPEED_RAMP_ENABLE    0

#if ((LOW_SPEED_RAMP_ENABLE != 0) && (LOW_SPEED_RAMP_ENABLE != 1))
#error "LOW_SPEED_RAMP_ENABLE must be 0 or 1"
#endif

#define COGGING_TABLE_SIZE 128

/*============================================================
 * 零电角度校准模式选择
 * 1：每次开机自动校准
 * 2：按键校准并保存到Flash
 *============================================================*/
#define CALIBRATION_MODE    2

#if ((CALIBRATION_MODE != 1) && (CALIBRATION_MODE != 2))
#error "CALIBRATION_MODE must be 1 or 2"
#endif

typedef enum
{
    MOTOR_STATE_UNCALIBRATED = 0,
    MOTOR_STATE_CALIBRATING,
    MOTOR_STATE_READY
} MotorState_e;

/*============================================================
 * 统一停车模式
 *============================================================*/
typedef enum
{
    MOTOR_STOP_NONE = 0,          /* 正常运行 */
    MOTOR_STOP_ACTIVE_BRAKE,      /* 主动制动，接近零速后输出归零 */
    MOTOR_STOP_COAST              /* 紧急滑行停止，立即关闭驱动使能 */
} MotorStopMode_e;

extern volatile MotorState_e motor_state;
extern volatile uint8_t calibration_requested;
extern volatile uint8_t calibration_in_progress;

/* Motor1 基础接口 */
void FOC_Init1(float power);
void Check_Sensor1(void);
void Set_Velocity(float Target);

/* Motor2 接口：如果 Motor2 在其他文件中实现，则保留这些声明 */
void FOC_Init2(float power);
void Set_Velocity2(float Target);

void Motor_en(void);

float constrain(float amt, float low, float high);
float normalizeAngle(float angle);

void SetPhaseVoltage(float Uq, float Ud, float angle_el);
void SetPhaseVoltage2(float Uq, float Ud, float angle_el);

float electricAngle(void);
float electricAngle2(void);

float cal_Iq_Id(float current_a, float current_b, float angle_el);
float cal_Uq_Ud(float voltage_a, float voltage_b, float angle_el);

void Set_Angle(float Target);
void Set_CurTorque(float Target);
void Set_VolTorque(float Target);
float velocityopenloop(float target);

void Velocity_OpenLoop_Task(float target_velocity, float voltage_v);
void Velocity_proc(void);
void SVPWM_OpenLoop_Test(void);
void loopFOC(void);
void move(float new_target);
void open_loop_proc(void);
void Calibration_Process(void);

/*============================================================
 * 统一运动控制接口
 *
 * CAN、UART、按键和自主控制都应调用这些接口，不要直接清PID或写Uq。
 *============================================================*/
void Motor_Control_SetVelocity(float velocity_rad_s);
void Motor_Control_SetPosition(float position_rad);
void Motor_Control_Start(void);
void Motor_Control_RequestStop(MotorStopMode_e mode);

MotorStopMode_e Motor_Control_GetStopMode(void);
uint8_t Motor_Control_IsDriverEnabled(void);
uint8_t Motor_Control_IsRunning(void);

extern volatile MotorStopMode_e motor_stop_request;

/* 停车调试变量，可加入 Keil Watch */
extern float motor_brake_uq_dbg;
extern float motor_brake_speed_dbg;
extern uint8_t motor_brake_active_dbg;

/* 位置模式速度规划调试变量，可直接加入 Keil Watch */
extern float position_error_dbg;
extern float position_velocity_cmd_dbg;
extern float position_brake_speed_limit_dbg;
extern uint8_t position_arrived_dbg;
extern uint8_t position_overshoot_dbg;

/* 低速模式调试变量，可直接加入 Keil Watch */
extern float low_speed_ramp_angle_dbg;
extern float low_speed_ramp_error_dbg;
extern float low_speed_ramp_uq_dbg;
extern uint8_t low_speed_ramp_active_dbg;

#endif /* __YIFOC_H */
