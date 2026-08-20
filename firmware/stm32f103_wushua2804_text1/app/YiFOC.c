/*

 * @Author: yihazui6 1319743179@qq.com

 * @Date: 2026-04-29

 * @LastEditTime: 2026-07-11

 * @Description:

 *   FOC 控制程序。

 *   已加入 LOW_SPEED_RAMP_ENABLE 编译开关，用于控制是否启用低速

 *   “目标角度匀速爬坡”模式。

 */



#include <math.h>

#include <stdlib.h>

#include "YiFOC.h"



/************************************************

本程序仅供学习，引用代码请标明出处

创建日期：20260305

作    者：翼哈

b站名字：翼哈yiha



本代码中部分内容引用于：作者：loop222 @郑州 的源码

************************************************/



#define PI          3.14159265359f

#define _3PI_2      4.71238898f

#define _1_SQRT3    0.57735026919f

#define _2_SQRT3    1.15470053838f

#define _PI_2       1.57079632679f

#define _PI_3       1.04719755120f

#define _SQRT3      1.73205080757f



#define PWM_PERIOD1 3200.0f



#if LOW_SPEED_RAMP_ENABLE

/*============================================================

 * 低速“位置匀速爬坡”参数

 *

 * 当 controller == Type_velocity，并且目标速度绝对值不大于

 * LOW_SPEED_RAMP_THRESHOLD 时：

 *

 * 1. 不使用 AS5600 速度反馈；

 * 2. 让虚拟目标角度按照目标速度匀速增加；

 * 3. 根据位置误差计算 Uq；

 * 4. 通过速度前馈和静摩擦补偿改善低速启动。

 *============================================================*/

#define LOW_SPEED_RAMP_THRESHOLD      2.0f

#define LOW_SPEED_RAMP_DEFAULT_DT     0.002f

#define LOW_SPEED_RAMP_KP             4.0f

#define LOW_SPEED_RAMP_KV_FF          0.20f

#define LOW_SPEED_RAMP_UQ_STATIC      0.35f

#define LOW_SPEED_RAMP_MAX_UQ         3.0f

#define LOW_SPEED_RAMP_MAX_ERROR      0.35f

#define LOW_SPEED_RAMP_STOP_DEADBAND  0.001f

#endif

/*============================================================
 * 统一停车控制参数
 *
 * 速度命令进入零区间后，不再让普通速度PID或低速位置爬坡继续运行，
 * 而是进入主动制动状态。制动力与当前速度方向相反。
 *============================================================*/
#define MOTOR_ZERO_SPEED_COMMAND       0.02f
#define MOTOR_STOP_BRAKE_KP            0.25f
#define MOTOR_STOP_BRAKE_MAX_OUTPUT    1.20f
#define MOTOR_STOP_RELEASE_SPEED       0.15f
#define MOTOR_STOP_REENGAGE_SPEED      0.30f
#define MOTOR_STOP_RELEASE_COUNT       5U

/*============================================================
 * 位置模式防过冲参数
 *
 * 位置模式不再直接使用“位置PID输出目标速度”的简单方式，而是：
 * 1. 根据剩余距离计算制动速度上限；
 * 2. 对目标速度执行加减速斜坡；
 * 3. 进入目标附近后清除速度环积分；
 * 4. 到位后进入带回差的静止区，避免前后反复修正。
 *============================================================*/

/* move()正常每2ms调用一次；时间异常时使用该默认值 */
#define POSITION_CONTROL_DEFAULT_DT          0.002f

/* 位置目标速度的加速度和减速度，单位 rad/s^2 */
#define POSITION_CONTROL_ACCEL_LIMIT         18.0f
#define POSITION_CONTROL_DECEL_LIMIT         30.0f

/*
 * 制动距离曲线参数，单位 rad/s^2。
 * 数值越小，越早开始减速；数值越大，越晚减速。
 */
#define POSITION_CONTROL_BRAKE_DECEL         14.0f

/* 位置误差和实际速度同时满足后才判定到位 */
#define POSITION_CONTROL_ARRIVE_ERROR        0.03f
#define POSITION_CONTROL_ARRIVE_SPEED        0.18f
#define POSITION_CONTROL_ARRIVE_COUNT        10U

/*
 * 到位后的位置回差。
 * 在该范围内不再反复正反向修正；漂移超过该范围才重新启动位置控制。
 */
#define POSITION_CONTROL_HOLD_EXIT_ERROR     0.06f

/* 进入该位置范围后，速度环按无积分方式工作，防止终点积分导致回弹 */
#define POSITION_CONTROL_NO_I_ERROR          0.250f

/* 一旦发生过冲，反向修正速度限制在该值以内 */
#define POSITION_CONTROL_REVERSE_MAX_SPEED   1.00f

/* 防止位置最大速度被误设为0 */
#define POSITION_CONTROL_MIN_MAX_SPEED       0.20f

#ifndef MOTOR_ENABLE_GPIO_PORT
#define MOTOR_ENABLE_GPIO_PORT         GPIOA
#endif

#ifndef MOTOR_ENABLE_GPIO_PIN
#define MOTOR_ENABLE_GPIO_PIN          GPIO_PIN_3
#endif



/*==================== Motor1 专属变量 ====================*/

float Ua1 = 0.0f;

float Ub1 = 0.0f;

float Uc1 = 0.0f;

float Ualpha1 = 0.0f;

float Ubeta1 = 0.0f;

float dc_a1 = 0.0f;

float dc_b1 = 0.0f;

float dc_c1 = 0.0f;



float voltage_limit1 = 6.0f;

float voltage_power_supply1 = 0.0f;

float zero_electric_Angle1 = 0.0f;

float electrical_angle = 0.0f;



int pp1 = 7;

int Dir1 = 1;

/*============================================================
 * 校准相关全局变量
 *============================================================*/
volatile MotorState_e motor_state = MOTOR_STATE_UNCALIBRATED;
volatile uint8_t calibration_requested = 0;
volatile uint8_t calibration_in_progress = 0;

float target = 0.0f;

float shaft_angle_sp = 0.0f;

float shaft_velocity_sp = 0.0f;

float current_sp = 0.0f;



Dir_axis voltage = {0.0f, 0.0f};

DQCurrent_s current = {0.0f, 0.0f};

/*============================================================
 * 统一停车控制状态
 *
 * motor_stop_request 由 CAN/UART/按键等命令层写入；
 * PID复位、低速目标清除和制动输出只在 move() 控制周期中执行。
 *============================================================*/
volatile MotorStopMode_e motor_stop_request = MOTOR_STOP_NONE;

static MotorStopMode_e s_motor_stop_active = MOTOR_STOP_NONE;
static MotionControlType s_last_controller = Type_velocity;
static uint8_t s_motor_driver_enabled = 1U;
static uint8_t s_brake_released = 0U;
static uint8_t s_brake_release_count = 0U;

float motor_brake_uq_dbg = 0.0f;
float motor_brake_speed_dbg = 0.0f;
uint8_t motor_brake_active_dbg = 0U;

/*============================================================
 * 位置速度规划状态
 *============================================================*/
static volatile uint8_t s_position_new_target_request = 0U;
static uint8_t s_position_plan_inited = 0U;
static uint8_t s_position_arrived = 0U;
static uint8_t s_position_overshoot = 0U;
static uint16_t s_position_arrive_count = 0U;
static float s_position_velocity_cmd = 0.0f;
static float s_position_last_error = 0.0f;
static float s_position_active_target = 0.0f;
static uint32_t s_position_last_timestamp_us = 0U;

/* 可加入Keil Watch观察 */
float position_error_dbg = 0.0f;
float position_velocity_cmd_dbg = 0.0f;
float position_brake_speed_limit_dbg = 0.0f;
uint8_t position_arrived_dbg = 0U;
uint8_t position_overshoot_dbg = 0U;



/* 低速模式调试变量：即使关闭低速模式也保留，便于其他文件引用 */

float low_speed_ramp_angle_dbg = 0.0f;

float low_speed_ramp_error_dbg = 0.0f;

float low_speed_ramp_uq_dbg = 0.0f;

uint8_t low_speed_ramp_active_dbg = 0;



#if LOW_SPEED_RAMP_ENABLE

/* 低速模式内部状态 */

static uint8_t low_speed_ramp_inited = 0;

#endif



/*============================================================

 * 通用函数

 *============================================================*/

float constrain(float amt, float low, float high)

{

    return ((amt < low) ? low : ((amt > high) ? high : amt));

}



float normalizeAngle(float angle)

{

    float a = fmodf(angle, 2.0f * PI);

    return ((a >= 0.0f) ? a : (a + 2.0f * PI));

}



/*============================================================

 * Motor1 电角度和 PWM 输出

 *============================================================*/

float electricAngle(void)

{

    return normalizeAngle((as5600_angle * pp1 * Dir1)

                          - zero_electric_Angle1);

}



static void SetPwmDuty1(float Ta, float Tb, float Tc)

{

    dc_a1 = constrain(Ta, 0.2f, 0.98f);

    dc_b1 = constrain(Tb, 0.2f, 0.98f);

    dc_c1 = constrain(Tc, 0.2f, 0.98f);



    /*

     * 保留你原来的相序映射：

     * MOTOR_CHANNEL_U <- dc_b1

     * MOTOR_CHANNEL_V <- dc_c1

     * MOTOR_CHANNEL_W <- dc_a1

     */

    Set_MOTOR_Compare(MOTOR_CHANNEL_U, dc_b1 * PWM_PERIOD1);

    Set_MOTOR_Compare(MOTOR_CHANNEL_V, dc_c1 * PWM_PERIOD1);

    Set_MOTOR_Compare(MOTOR_CHANNEL_W, dc_a1 * PWM_PERIOD1);

}



void SetPhaseVoltage(float Uq, float Ud, float angle_el)

{

    float Uout;

    float angle_svpwm;

    uint32_t sector;

    float T0;

    float T1;

    float T2;

    float Ta = 0.5f;

    float Tb = 0.5f;

    float Tc = 0.5f;



    /* 防止母线电压为 0，避免除 0 */

    if (voltage_power_supply1 <= 0.0f)

    {

        SetPwmDuty1(0.5f, 0.5f, 0.5f);

        return;

    }



    /* 限制 D/Q 轴电压 */

    Uq = constrain(Uq, -voltage_limit1, voltage_limit1);

    Ud = constrain(Ud, -voltage_limit1, voltage_limit1);



    /* DQ 电压矢量转换为 SVPWM 幅值和角度 */

    if (Ud != 0.0f)

    {

        Uout = sqrtf(Ud * Ud + Uq * Uq) / voltage_power_supply1;

        angle_svpwm = normalizeAngle(angle_el + atan2f(Uq, Ud));

    }

    else

    {

        Uout = fabsf(Uq) / voltage_power_supply1;



        if (Uq >= 0.0f)

        {

            angle_svpwm = normalizeAngle(angle_el + _PI_2);

        }

        else

        {

            angle_svpwm = normalizeAngle(angle_el - _PI_2);

        }

    }



    /* SVPWM 线性区限制 */

    Uout = constrain(Uout, 0.0f, _1_SQRT3);



    /* sector 范围为 1～6 */

    sector = (uint32_t)(angle_svpwm / _PI_3) + 1U;

    if (sector > 6U)

    {

        sector = 6U;

    }



    T1 = _SQRT3

         * sinf((float)sector * _PI_3 - angle_svpwm)

         * Uout;



    T2 = _SQRT3

         * sinf(angle_svpwm - ((float)sector - 1.0f) * _PI_3)

         * Uout;



    T0 = 1.0f - T1 - T2;



    if (T0 < 0.0f)

    {

        T0 = 0.0f;

    }



    switch (sector)

    {

        case 1U:

            Ta = T1 + T2 + T0 / 2.0f;

            Tb = T2 + T0 / 2.0f;

            Tc = T0 / 2.0f;

            break;



        case 2U:

            Ta = T1 + T0 / 2.0f;

            Tb = T1 + T2 + T0 / 2.0f;

            Tc = T0 / 2.0f;

            break;



        case 3U:

            Ta = T0 / 2.0f;

            Tb = T1 + T2 + T0 / 2.0f;

            Tc = T2 + T0 / 2.0f;

            break;



        case 4U:

            Ta = T0 / 2.0f;

            Tb = T1 + T0 / 2.0f;

            Tc = T1 + T2 + T0 / 2.0f;

            break;



        case 5U:

            Ta = T2 + T0 / 2.0f;

            Tb = T0 / 2.0f;

            Tc = T1 + T2 + T0 / 2.0f;

            break;



        case 6U:

            Ta = T1 + T2 + T0 / 2.0f;

            Tb = T0 / 2.0f;

            Tc = T1 + T0 / 2.0f;

            break;



        default:

            Ta = 0.5f;

            Tb = 0.5f;

            Tc = 0.5f;

            break;

    }



    SetPwmDuty1(Ta, Tb, Tc);
}

/*============================================================
 * 非阻塞校准处理函数
 *
 * 由主循环调用，避免在中断中使用HAL_Delay()和Flash擦写
 *============================================================*/
void Calibration_Process(void)
{
    static uint32_t cal_start_time = 0;
    static uint8_t cal_step = 0;

    if (!calibration_requested) return;

    switch (cal_step) {
        case 0:  /* 开始校准 */
            motor_state = MOTOR_STATE_CALIBRATING;
            calibration_in_progress = 1;

            /* 检测母线电压，防止在MCU供电但电机母线没供电时保存错误角度 */
            if (voltage_power_supply1 <= 1.0f) {
                /* 母线电压太低，可能是只有MCU供电，拒绝校准 */
                calibration_requested = 0;
                calibration_in_progress = 0;
                motor_state = MOTOR_STATE_UNCALIBRATED;
                cal_step = 0;
                return;
            }

            /* 施加定位电压，将电机转子拉到已知位置 */
            SetPhaseVoltage(3, 0, _3PI_2);
            cal_start_time = HAL_GetTick();
            cal_step = 1;
            break;

        case 1:  /* 等待2秒，让电机稳定在定位位置 */
            if (HAL_GetTick() - cal_start_time >= 2000) {
                /* 读取当前位置作为零电角度 */
                as5600_angle = AS5600_I2C_Sensor_return(&as5600_encoder);
                zero_electric_Angle1 = normalizeAngle((as5600_angle * pp1 * Dir1));

                /* 关闭定位电压 */
                SetPhaseVoltage(0, 0, _3PI_2);
                cal_start_time = HAL_GetTick();
                cal_step = 2;
            }
            break;

        case 2:  /* 等待500ms */
            if (HAL_GetTick() - cal_start_time >= 500) {
                /* 校准完成，保存到Flash */
                FOC_Flash_SaveZeroAngle(zero_electric_Angle1, pp1, Dir1);

                /* 清除校准请求，恢复电机状态 */
                calibration_requested = 0;
                calibration_in_progress = 0;
                motor_state = MOTOR_STATE_READY;
                cal_step = 0;
            }
            break;
    }
}

/*============================================================
 * FOC 初始化和传感器校准

 *============================================================*/

void Check_Sensor1(void)

{

	SetPhaseVoltage(3, 0, _3PI_2);

	HAL_Delay(2000);

	 // 所以校准零电角度之前，必须先更新 as5600_angle

    as5600_angle = AS5600_I2C_Sensor_return(&as5600_encoder);



    zero_electric_Angle1 = normalizeAngle((as5600_angle * pp1 * Dir1));

	SetPhaseVoltage(0, 0, _3PI_2);

	HAL_Delay(500);

}



void FOC_Init1(float power)

{

	voltage_power_supply1 = power;

	Check_Sensor1(); // 校准motor1零电角度

}



/*============================================================

 * 开环速度控制

 *============================================================*/

void Velocity_OpenLoop_Task(float target_velocity, float voltage_v)

{

    static float open_loop_angle = 0.0f;

    static uint32_t last_timestamp = 0U;



    uint32_t current_timestamp;

    float dt;

    float el_angle;



    current_timestamp = HAL_GetTick();

    dt = (float)(current_timestamp - last_timestamp) / 1000.0f;



    if ((dt <= 0.0f) || (dt > 0.1f))

    {

        dt = 0.001f;

    }



    last_timestamp = current_timestamp;



    open_loop_angle += target_velocity * dt;

    open_loop_angle = normalizeAngle(open_loop_angle);



    el_angle = normalizeAngle(open_loop_angle * pp1 * Dir1);



    SetPhaseVoltage(voltage_v, 0.0f, el_angle);

}



void open_loop_proc(void)

{

    Velocity_OpenLoop_Task(1.0f, 2.0f);

}



/*============================================================

 * 普通速度 PID

 *

 * 当 LOW_SPEED_RAMP_ENABLE == 0 时，所有速度都会进入这里。

 *============================================================*/

static void NormalVelocityControl(float target_velocity,

                                  float measured_velocity)

{

    shaft_velocity_sp = target_velocity;



    current_sp = PID_brushless(

        &pid_motor_velocity,

        shaft_velocity_sp - measured_velocity

    );



    if (torque_controller == Type_voltage)

    {

        voltage.q = current_sp;

        voltage.d = 0.0f;

    }

}



/*============================================================

 * 低速位置匀速爬坡模式

 *============================================================*/

static void LowSpeedRamp_Deactivate(void)

{

#if LOW_SPEED_RAMP_ENABLE

    low_speed_ramp_inited = 0U;

#endif



    low_speed_ramp_angle_dbg = 0.0f;

    low_speed_ramp_error_dbg = 0.0f;

    low_speed_ramp_uq_dbg = 0.0f;

    low_speed_ramp_active_dbg = 0U;

}


/*============================================================
 * 位置模式速度规划与防过冲
 *============================================================*/
static float PositionControl_GetDt(void)
{
    uint32_t now_us;
    uint32_t delta_us;
    float dt;

    now_us = DWT_Get_Microsecond();

    if (s_position_last_timestamp_us == 0U)
    {
        dt = POSITION_CONTROL_DEFAULT_DT;
    }
    else
    {
        delta_us = now_us - s_position_last_timestamp_us;
        dt = (float)delta_us * 1e-6f;

        if ((dt <= 0.0f) || (dt > 0.05f))
        {
            dt = POSITION_CONTROL_DEFAULT_DT;
        }
    }

    s_position_last_timestamp_us = now_us;
    return dt;
}

static void PositionControl_ResetState(void)
{
    s_position_plan_inited = 0U;
    s_position_arrived = 0U;
    s_position_overshoot = 0U;
    s_position_arrive_count = 0U;
    s_position_velocity_cmd = 0.0f;
    s_position_last_error = 0.0f;
    s_position_active_target = as5600_angle;
    s_position_last_timestamp_us = 0U;

    position_error_dbg = 0.0f;
    position_velocity_cmd_dbg = 0.0f;
    position_brake_speed_limit_dbg = 0.0f;
    position_arrived_dbg = 0U;
    position_overshoot_dbg = 0U;
}

static void PositionControl_SetZeroOutput(void)
{
    shaft_velocity_sp = 0.0f;
    current_sp = 0.0f;

    if (torque_controller == Type_voltage)
    {
        voltage.q = 0.0f;
        voltage.d = 0.0f;
    }
}

static float PositionControl_SlewVelocity(float current_cmd,
                                          float desired_cmd,
                                          float dt)
{
    float delta;
    float rate_limit;
    float max_delta;

    delta = desired_cmd - current_cmd;

    /*
     * 同方向且目标幅值增大时使用加速度限制；
     * 减速或准备换向时使用更大的减速度限制。
     */
    if (((current_cmd * desired_cmd) > 0.0f) &&
        (fabsf(desired_cmd) > fabsf(current_cmd)))
    {
        rate_limit = POSITION_CONTROL_ACCEL_LIMIT;
    }
    else
    {
        rate_limit = POSITION_CONTROL_DECEL_LIMIT;
    }

    max_delta = rate_limit * dt;

    if (delta > max_delta)
    {
        delta = max_delta;
    }
    else if (delta < -max_delta)
    {
        delta = -max_delta;
    }

    return current_cmd + delta;
}

static void PositionControl_Update(float target_position)
{
    float dt;
    float position_error;
    float abs_error;
    float speed_abs;
    float max_speed;
    float speed_from_position;
    float brake_distance;
    float brake_speed_limit;
    float desired_speed_abs;
    float desired_speed;
    uint8_t no_integral_zone;

    dt = PositionControl_GetDt();
    position_error = target_position - as5600_angle;
    abs_error = fabsf(position_error);

    as5600_speed = shaftVelocity();
    speed_abs = fabsf(as5600_speed);

    /*
     * 新位置命令只在控制周期中复位PID和规划状态，
     * 避免CAN任务与定时器中断同时修改PID内部变量。
     */
    if ((s_position_new_target_request != 0U) ||
        (fabsf(target_position - s_position_active_target) > 1e-6f))
    {
        PositionControl_ResetState();
        PID_ResetState(&pid_motor_angle);
        PID_ResetState(&pid_motor_velocity);

        s_position_last_error = position_error;
        s_position_active_target = target_position;
        s_position_plan_inited = 1U;
        s_position_new_target_request = 0U;
    }

    if (s_position_plan_inited == 0U)
    {
        s_position_last_error = position_error;
        s_position_velocity_cmd = 0.0f;
        s_position_plan_inited = 1U;
    }

    /*
     * 到位锁定区：
     * 目标附近不再因为编码器噪声或微小位置误差来回正反转。
     */
    if (s_position_arrived != 0U)
    {
        if (abs_error <= POSITION_CONTROL_HOLD_EXIT_ERROR)
        {
            PositionControl_SetZeroOutput();

            position_error_dbg = position_error;
            position_velocity_cmd_dbg = 0.0f;
            position_brake_speed_limit_dbg = 0.0f;
            position_arrived_dbg = 1U;
            position_overshoot_dbg = s_position_overshoot;
            return;
        }

        /* 被外力推出回差范围后重新进行位置控制 */
        s_position_arrived = 0U;
        s_position_arrive_count = 0U;
        s_position_velocity_cmd = 0.0f;
        PID_ResetState(&pid_motor_velocity);
    }

    /*
     * 检测是否越过目标点。
     * 越过后先清除原方向的速度积分，并限制反向修正速度。
     */
    if (((position_error * s_position_last_error) < 0.0f) &&
        (fabsf(s_position_last_error) > POSITION_CONTROL_ARRIVE_ERROR))
    {
        s_position_overshoot = 1U;
        s_position_velocity_cmd = 0.0f;
        PID_ResetState(&pid_motor_velocity);
    }

    /*
     * 位置和速度必须连续满足条件，才真正判定为到位。
     */
    if ((abs_error <= POSITION_CONTROL_ARRIVE_ERROR) &&
        (speed_abs <= POSITION_CONTROL_ARRIVE_SPEED))
    {
        if (s_position_arrive_count < POSITION_CONTROL_ARRIVE_COUNT)
        {
            s_position_arrive_count++;
        }
    }
    else
    {
        s_position_arrive_count = 0U;
    }

    if (s_position_arrive_count >= POSITION_CONTROL_ARRIVE_COUNT)
    {
        s_position_arrived = 1U;
        s_position_velocity_cmd = 0.0f;

        PID_ResetState(&pid_motor_angle);
        PID_ResetState(&pid_motor_velocity);
        PositionControl_SetZeroOutput();

        position_error_dbg = position_error;
        position_velocity_cmd_dbg = 0.0f;
        position_brake_speed_limit_dbg = 0.0f;
        position_arrived_dbg = 1U;
        position_overshoot_dbg = s_position_overshoot;
        return;
    }

    /*
     * pid_motor_angle.voltage_limit继续作为位置模式最大速度使用，
     * 因而原来的CAN“位置最大速度”命令仍然有效。
     */
    max_speed = fabsf(pid_motor_angle.voltage_limit);

    if (max_speed < POSITION_CONTROL_MIN_MAX_SPEED)
    {
        max_speed = POSITION_CONTROL_MIN_MAX_SPEED;
    }

    /*
     * 位置比例速度：误差越小，目标速度越低。
     * 这里只使用位置PID的P参数，不使用位置积分，避免位置积分导致过冲。
     */
    speed_from_position = fabsf(pid_motor_angle.P) * abs_error;

    /*
     * 制动速度曲线：
     * v_allow = sqrt(2 * a_brake * remaining_distance)
     *
     * 当前速度高于该曲线时，速度环会自动输出反向转矩提前制动。
     */
    brake_distance = abs_error - POSITION_CONTROL_ARRIVE_ERROR;

    if (brake_distance < 0.0f)
    {
        brake_distance = 0.0f;
    }

    brake_speed_limit = sqrtf(
        2.0f * POSITION_CONTROL_BRAKE_DECEL * brake_distance
    );

    desired_speed_abs = max_speed;

    if (speed_from_position < desired_speed_abs)
    {
        desired_speed_abs = speed_from_position;
    }

    if (brake_speed_limit < desired_speed_abs)
    {
        desired_speed_abs = brake_speed_limit;
    }

    if ((s_position_overshoot != 0U) &&
        (desired_speed_abs > POSITION_CONTROL_REVERSE_MAX_SPEED))
    {
        desired_speed_abs = POSITION_CONTROL_REVERSE_MAX_SPEED;
    }

    if (position_error > 0.0f)
    {
        desired_speed = desired_speed_abs;
    }
    else if (position_error < 0.0f)
    {
        desired_speed = -desired_speed_abs;
    }
    else
    {
        desired_speed = 0.0f;
    }

    /*
     * 对目标速度做斜坡，防止位置误差换向后立即给出很大的反向速度。
     */
    s_position_velocity_cmd = PositionControl_SlewVelocity(
        s_position_velocity_cmd,
        desired_speed,
        dt
    );

    shaft_angle_sp = target_position;
    shaft_velocity_sp = s_position_velocity_cmd;

    /*
     * 靠近终点时关闭速度环积分。
     * 这样速度误差产生的积分不会在到点后继续推动小车前进。
     */
    no_integral_zone =
        (abs_error <= POSITION_CONTROL_NO_I_ERROR) ? 1U : 0U;

    if (no_integral_zone != 0U)
    {
        pid_motor_velocity.integral_prev = 0.0f;
    }

    current_sp = PID_brushless(
        &pid_motor_velocity,
        shaft_velocity_sp - as5600_speed
    );

    if (no_integral_zone != 0U)
    {
        pid_motor_velocity.integral_prev = 0.0f;
    }

    if (torque_controller == Type_voltage)
    {
        voltage.q = current_sp;
        voltage.d = 0.0f;
    }

    s_position_last_error = position_error;

    position_error_dbg = position_error;
    position_velocity_cmd_dbg = shaft_velocity_sp;
    position_brake_speed_limit_dbg = brake_speed_limit;
    position_arrived_dbg = s_position_arrived;
    position_overshoot_dbg = s_position_overshoot;
}


/*============================================================
 * 统一停车控制层
 *============================================================*/
static void Motor_Control_SetDriverEnableInternal(uint8_t enable)
{
    s_motor_driver_enabled = (enable != 0U) ? 1U : 0U;

    HAL_GPIO_WritePin(MOTOR_ENABLE_GPIO_PORT,
                      MOTOR_ENABLE_GPIO_PIN,
                      s_motor_driver_enabled ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void Motor_Control_ResetAllPidState(void)
{
    PID_ResetState(&pid_motor_velocity);
    PID_ResetState(&pid_motor_angle);
    PID_ResetState(&PID_current_q);
    PID_ResetState(&PID_current_d);
}

static void Motor_Control_ClearDynamicState(void)
{
    Motor_Control_ResetAllPidState();
    LowSpeedRamp_Deactivate();
    PositionControl_ResetState();

    shaft_angle_sp = as5600_angle;
    shaft_velocity_sp = 0.0f;
    current_sp = 0.0f;

    voltage.q = 0.0f;
    voltage.d = 0.0f;
}

void Motor_Control_SetVelocity(float velocity_rad_s)
{
    /* 先写目标和模式，最后写停车请求，避免控制中断读到半更新状态。 */
    target = velocity_rad_s;
    controller = Type_velocity;
    s_position_new_target_request = 0U;
    Motor_Control_SetDriverEnableInternal(1U);

    if (fabsf(velocity_rad_s) <= MOTOR_ZERO_SPEED_COMMAND)
    {
        motor_stop_request = MOTOR_STOP_ACTIVE_BRAKE;
    }
    else
    {
        motor_stop_request = MOTOR_STOP_NONE;
    }
}

void Motor_Control_SetPosition(float position_rad)
{
    target = position_rad;
    controller = Type_angle;

    /*
     * 这里只提交新目标请求。
     * PID清零和位置规划状态复位统一在move()控制周期中完成。
     */
    s_position_new_target_request = 1U;

    Motor_Control_SetDriverEnableInternal(1U);
    motor_stop_request = MOTOR_STOP_NONE;
}

void Motor_Control_Start(void)
{
    /* 没有新同步命令时，启动后保持当前机械位置。 */
    target = as5600_angle;
    controller = Type_angle;
    s_position_new_target_request = 1U;
    Motor_Control_SetDriverEnableInternal(1U);
    motor_stop_request = MOTOR_STOP_NONE;
}

void Motor_Control_RequestStop(MotorStopMode_e mode)
{
    if (mode == MOTOR_STOP_NONE)
    {
        motor_stop_request = MOTOR_STOP_NONE;
        return;
    }

    target = 0.0f;
    s_position_new_target_request = 0U;

    if (mode == MOTOR_STOP_ACTIVE_BRAKE)
    {
        controller = Type_velocity;
        Motor_Control_SetDriverEnableInternal(1U);
        motor_stop_request = MOTOR_STOP_ACTIVE_BRAKE;
    }
    else
    {
        /* 紧急滑行停止：由控制层立即断开驱动使能。 */
        motor_stop_request = MOTOR_STOP_COAST;
        Motor_Control_SetDriverEnableInternal(0U);
    }
}

MotorStopMode_e Motor_Control_GetStopMode(void)
{
    return motor_stop_request;
}

uint8_t Motor_Control_IsDriverEnabled(void)
{
    return s_motor_driver_enabled;
}

uint8_t Motor_Control_IsRunning(void)
{
    return ((s_motor_driver_enabled != 0U) &&
            (motor_stop_request == MOTOR_STOP_NONE)) ? 1U : 0U;
}

static void Motor_Control_EnterStop(MotorStopMode_e mode)
{
    Motor_Control_ClearDynamicState();

    s_motor_stop_active = mode;
    s_brake_released = 0U;
    s_brake_release_count = 0U;

    motor_brake_uq_dbg = 0.0f;
    motor_brake_speed_dbg = 0.0f;
    motor_brake_active_dbg = 0U;

    if (mode == MOTOR_STOP_COAST)
    {
        Motor_Control_SetDriverEnableInternal(0U);
    }
    else
    {
        Motor_Control_SetDriverEnableInternal(1U);
    }
}

static void Motor_Control_ExitStop(void)
{
    Motor_Control_ClearDynamicState();

    s_motor_stop_active = MOTOR_STOP_NONE;
    s_brake_released = 0U;
    s_brake_release_count = 0U;

    motor_brake_uq_dbg = 0.0f;
    motor_brake_speed_dbg = 0.0f;
    motor_brake_active_dbg = 0U;

    s_last_controller = controller;
}

static uint8_t Motor_Control_ProcessStop(void)
{
    MotorStopMode_e requested_mode = motor_stop_request;
    float speed_abs;
    float brake_output;

    if (requested_mode == MOTOR_STOP_NONE)
    {
        if (s_motor_stop_active != MOTOR_STOP_NONE)
        {
            Motor_Control_ExitStop();
        }

        return 0U;
    }

    if (requested_mode != s_motor_stop_active)
    {
        Motor_Control_EnterStop(requested_mode);
    }

    if (requested_mode == MOTOR_STOP_COAST)
    {
        current_sp = 0.0f;
        voltage.q = 0.0f;
        voltage.d = 0.0f;

        motor_brake_uq_dbg = 0.0f;
        motor_brake_speed_dbg = 0.0f;
        motor_brake_active_dbg = 0U;

        Motor_Control_SetDriverEnableInternal(0U);
        return 1U;
    }

    /* 主动制动：输出方向始终与实际速度方向相反。 */
    as5600_speed = shaftVelocity();
    speed_abs = fabsf(as5600_speed);

    if (s_brake_released == 0U)
    {
        if (speed_abs <= MOTOR_STOP_RELEASE_SPEED)
        {
            if (s_brake_release_count < MOTOR_STOP_RELEASE_COUNT)
            {
                s_brake_release_count++;
            }

            if (s_brake_release_count >= MOTOR_STOP_RELEASE_COUNT)
            {
                s_brake_released = 1U;
                Motor_Control_ResetAllPidState();
            }
        }
        else
        {
            s_brake_release_count = 0U;
        }
    }
    else if (speed_abs >= MOTOR_STOP_REENGAGE_SPEED)
    {
        /* 停车后如果被外力再次推动，重新加入制动。 */
        s_brake_released = 0U;
        s_brake_release_count = 0U;
    }

    if (s_brake_released != 0U)
    {
        brake_output = 0.0f;
    }
    else
    {
        brake_output = -MOTOR_STOP_BRAKE_KP * as5600_speed;
        brake_output = constrain(brake_output,
                                 -MOTOR_STOP_BRAKE_MAX_OUTPUT,
                                  MOTOR_STOP_BRAKE_MAX_OUTPUT);
    }

    current_sp = brake_output;

    if (torque_controller == Type_voltage)
    {
        voltage.q = brake_output;
        voltage.d = 0.0f;
    }

    motor_brake_uq_dbg = brake_output;
    motor_brake_speed_dbg = as5600_speed;
    motor_brake_active_dbg = (s_brake_released == 0U) ? 1U : 0U;

    return 1U;
}

static void Motor_Control_HandleModeChange(void)
{
    if (controller != s_last_controller)
    {
        Motor_Control_ClearDynamicState();
        s_last_controller = controller;
    }
}



#if LOW_SPEED_RAMP_ENABLE

static float LowSpeedRamp_GetDt(void)

{

    static uint32_t last_tick = 0U;



    uint32_t now_tick;

    float dt;



    now_tick = HAL_GetTick();



    if (last_tick == 0U)

    {

        dt = LOW_SPEED_RAMP_DEFAULT_DT;

    }

    else

    {

        dt = (float)(now_tick - last_tick) * 0.001f;



        if ((dt <= 0.0f) || (dt > 0.05f))

        {

            dt = LOW_SPEED_RAMP_DEFAULT_DT;

        }

    }



    last_tick = now_tick;

    return dt;

}



static void LowSpeedRamp_Update(float target_velocity, float dt)

{

    float raw_error;

    float pos_error;

    float uq_cmd;



    /* 第一次进入时从当前机械位置开始，避免目标角度跳变 */

    if (low_speed_ramp_inited == 0U)

    {

        shaft_angle_sp = as5600_angle;

        low_speed_ramp_inited = 1U;

    }



    /* 很小的速度认为是停止 */

    if (fabsf(target_velocity) < LOW_SPEED_RAMP_STOP_DEADBAND)

    {

        target_velocity = 0.0f;

    }



    /* 虚拟目标角度按照目标速度匀速增加 */

    shaft_angle_sp += target_velocity * dt;

    shaft_velocity_sp = target_velocity;



    /* as5600_angle 和 shaft_angle_sp 都应当是多圈累计角度 */

    raw_error = shaft_angle_sp - as5600_angle;



    /* 限制最大位置误差，防止堵转后突然猛冲 */

    if (raw_error > LOW_SPEED_RAMP_MAX_ERROR)

    {

        pos_error = LOW_SPEED_RAMP_MAX_ERROR;

        shaft_angle_sp = as5600_angle + LOW_SPEED_RAMP_MAX_ERROR;

    }

    else if (raw_error < -LOW_SPEED_RAMP_MAX_ERROR)

    {

        pos_error = -LOW_SPEED_RAMP_MAX_ERROR;

        shaft_angle_sp = as5600_angle - LOW_SPEED_RAMP_MAX_ERROR;

    }

    else

    {

        pos_error = raw_error;

    }



    /* 位置误差比例输出 */

    uq_cmd = LOW_SPEED_RAMP_KP * pos_error;



    /* 速度前馈 */

    uq_cmd += LOW_SPEED_RAMP_KV_FF * target_velocity;



    /* 静摩擦补偿 */

    if (target_velocity > LOW_SPEED_RAMP_STOP_DEADBAND)

    {

        uq_cmd += LOW_SPEED_RAMP_UQ_STATIC;

    }

    else if (target_velocity < -LOW_SPEED_RAMP_STOP_DEADBAND)

    {

        uq_cmd -= LOW_SPEED_RAMP_UQ_STATIC;

    }



    uq_cmd = constrain(

        uq_cmd,

        -LOW_SPEED_RAMP_MAX_UQ,

        LOW_SPEED_RAMP_MAX_UQ

    );



    /*

     * 电压控制模式：current_sp 最终直接作为 Uq。

     * 电流控制模式：current_sp 作为目标电流交给 loopFOC()。

     */

    current_sp = uq_cmd;



    if (torque_controller == Type_voltage)

    {

        voltage.q = current_sp;

        voltage.d = 0.0f;

    }



    low_speed_ramp_angle_dbg = shaft_angle_sp;

    low_speed_ramp_error_dbg = pos_error;

    low_speed_ramp_uq_dbg = uq_cmd;

    low_speed_ramp_active_dbg = 1U;

}



static uint8_t LowSpeedRamp_ShouldUse(float target_velocity)

{

    return (fabsf(target_velocity) <= LOW_SPEED_RAMP_THRESHOLD)

           ? 1U

           : 0U;

}

#endif



/*============================================================

 * 单独调用的速度控制接口

 *============================================================*/

extern LowPassFilter lpf1;



void Set_Velocity(float Target)

{

    float Vel;

    float Uq;



    /* 读取当前机械角度，用于电角度和低速位置反馈 */

    as5600_angle = AS5600_I2C_Sensor_return(&as5600_encoder);

    electrical_angle = electricAngle();



#if LOW_SPEED_RAMP_ENABLE

    if (LowSpeedRamp_ShouldUse(Target) != 0U)

    {

        float dt = LowSpeedRamp_GetDt();



        LowSpeedRamp_Update(Target, dt);

        SetPhaseVoltage(voltage.q, voltage.d, electrical_angle);

        return;

    }

#endif



    /*

     * 低速模式关闭，或者目标速度高于低速阈值：

     * 全部运行原来的普通速度 PID。

     */

    LowSpeedRamp_Deactivate();



    as5600_speed = AS5600_I2C_Sensor_getVelocity(&as5600_encoder);

    Vel = LPF_Update(&lpf1, as5600_speed);



    Uq = PID_brushless(

        &pid_motor_velocity,

        Dir1 * (Target - Vel)

    );



    current_sp = Uq;

    voltage.q = Uq;

    voltage.d = 0.0f;



    SetPhaseVoltage(Uq, 0.0f, electrical_angle);

}



void Velocity_proc(void)

{

    Set_Velocity(1.0f);

}



/*============================================================

 * FOC 内环

 *============================================================*/

void loopFOC(void)
{
    /* 校准过程中不执行正常FOC控制，防止覆盖校准定位电压。 */
    if (motor_state != MOTOR_STATE_READY)
    {
        return;
    }

    as5600_angle = shaftAngle();
    electrical_angle = electricAngle();

    /* 紧急滑行停止时驱动已经关闭，FOC内环不再更新PWM。 */
    if ((motor_stop_request == MOTOR_STOP_COAST) ||
        (s_motor_driver_enabled == 0U))
    {
        current_sp = 0.0f;
        voltage.q = 0.0f;
        voltage.d = 0.0f;
        return;
    }

    /*
     * 停车请求刚到、move() 尚未完成PID复位时，先输出零转矩一周期，
     * 防止继续使用上一周期积累的正向输出。
     */
    if ((motor_stop_request != MOTOR_STOP_NONE) &&
        (motor_stop_request != s_motor_stop_active))
    {
        current_sp = 0.0f;
        voltage.q = 0.0f;
        voltage.d = 0.0f;
        SetPhaseVoltage(0.0f, 0.0f, electrical_angle);
        return;
    }

    switch (torque_controller)
    {
        case Type_voltage:
            /* 电压模式下，move() 已经计算 voltage.q / voltage.d。 */
            break;

        case Type_dc_current:
            current.q = getDCCurrent(electrical_angle);
            current.q = LPF_Update(&LPF_current_q, current.q);

            voltage.q = PID_brushless(
                &PID_current_q,
                current_sp - current.q
            );
            voltage.d = 0.0f;
            break;

        case Type_foc_current:
            current = getFOCCurrents(electrical_angle);

            current.q = LPF_Update(&LPF_current_q, current.q);
            current.d = LPF_Update(&LPF_current_d, current.d);

            voltage.q = PID_brushless(
                &PID_current_q,
                current_sp - current.q
            );

            voltage.d = PID_brushless(
                &PID_current_d,
                -current.d
            );
            break;

        default:
            voltage.q = 0.0f;
            voltage.d = 0.0f;
            break;
    }

    SetPhaseVoltage(voltage.q, voltage.d, electrical_angle);
}

/*============================================================
 * 运动控制外环
 *
 * controller == Type_velocity：new_target 单位为 rad/s
 * controller == Type_angle：new_target 单位为 rad
 *============================================================*/
void move(float new_target)
{
    if (motor_state != MOTOR_STATE_READY)
    {
        return;
    }

    /*
     * 兼容旧代码：即使 UART/按键仍然直接修改 target，速度为0时也会
     * 自动进入统一主动制动；速度重新变为非0时自动退出主动制动。
     * 紧急滑行停止 MOTOR_STOP_COAST 必须显式调用启动接口才能解除。
     */
    if (controller == Type_velocity)
    {
        if ((fabsf(new_target) <= MOTOR_ZERO_SPEED_COMMAND) &&
            (motor_stop_request == MOTOR_STOP_NONE))
        {
            motor_stop_request = MOTOR_STOP_ACTIVE_BRAKE;
        }
        else if ((fabsf(new_target) > MOTOR_ZERO_SPEED_COMMAND) &&
                 (motor_stop_request == MOTOR_STOP_ACTIVE_BRAKE))
        {
            motor_stop_request = MOTOR_STOP_NONE;
        }
    }
    else if ((controller == Type_angle) &&
             (motor_stop_request == MOTOR_STOP_ACTIVE_BRAKE))
    {
        motor_stop_request = MOTOR_STOP_NONE;
    }

    if (Motor_Control_ProcessStop() != 0U)
    {
        return;
    }

    Motor_Control_HandleModeChange();

    switch (controller)
    {
        case Type_velocity:
        {
            shaft_velocity_sp = new_target;

#if LOW_SPEED_RAMP_ENABLE
            if (LowSpeedRamp_ShouldUse(new_target) != 0U)
            {
                float dt = LowSpeedRamp_GetDt();
                LowSpeedRamp_Update(new_target, dt);
                break;
            }
#endif

            LowSpeedRamp_Deactivate();

            as5600_speed = shaftVelocity();
            NormalVelocityControl(new_target, as5600_speed);
            break;
        }

        case Type_angle:
        {
            LowSpeedRamp_Deactivate();

            /*
             * 使用位置速度规划：
             * 提前减速、终点清积分、过冲后低速修正、到位回差锁定。
             */
            PositionControl_Update(new_target);
            break;
        }

        default:
            Motor_Control_ClearDynamicState();
            break;
    }
}

