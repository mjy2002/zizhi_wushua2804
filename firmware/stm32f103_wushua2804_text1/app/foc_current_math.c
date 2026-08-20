#include "foc_current_math.h"

/************************************************
本程序仅供学习，引用代码请标明出处
创建日期：20260511
作    者：翼哈
b站名字：翼哈yiha

本代码中部分内容引用于：作者：loop222 @郑州 的源码
************************************************/

#define _1_SQRT3   0.57735026919f
#define _2_SQRT3   1.15470053838f

/******************************************************************************/
// 获取直流电流幅值
//   - 不提供电角度时返回绝对值
//   - 提供电角度时返回带符号值
float getDCCurrent(float motor_electrical_angle)
{
    PhaseCurrent_s current;
    float sign = 1;
    float i_alpha, i_beta;

    current = getPhaseCurrents();

    if(!current.c)
    {
        i_alpha = current.a;
        i_beta = _1_SQRT3 * current.a + _2_SQRT3 * current.b;
    }
    else
    {
        float mid = (1.0f/3.0f) * (current.a + current.b + current.c);
        float a = current.a - mid;
        float b = current.b - mid;
        i_alpha = a;
        i_beta = _1_SQRT3 * a + _2_SQRT3 * b;
    }

    if(motor_electrical_angle)
        sign = (i_beta * cosf(motor_electrical_angle) - i_alpha * sinf(motor_electrical_angle)) > 0 ? 1 : -1;

    return sign * sqrtf(i_alpha*i_alpha + i_beta*i_beta);
}

/******************************************************************************/
// FOC 电流计算：Clark + Park 变换
//   - 从三相电流计算 dq 轴电流
DQCurrent_s getFOCCurrents(float angle_el)
{
    PhaseCurrent_s current;
    float i_alpha, i_beta;
    float ct, st;
    DQCurrent_s ret;

    current = getPhaseCurrents();

    if(!current.c)
    {
        i_alpha = current.a;
        i_beta = _1_SQRT3 * current.a + _2_SQRT3 * current.b;
    }
    else
    {
        float mid = (1.0f/3.0f) * (current.a + current.b + current.c);
        float a = current.a - mid;
        float b = current.b - mid;
        i_alpha = a;
        i_beta = _1_SQRT3 * a + _2_SQRT3 * b;
    }

    ct = cosf(angle_el);
    st = sinf(angle_el);
    ret.d = i_alpha * ct + i_beta * st;
    ret.q = i_beta * ct - i_alpha * st;
    return ret;
}
/******************************************************************************/
