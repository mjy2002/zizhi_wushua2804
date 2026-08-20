#include "pwm_app.h"



void PWM_Init_All(void)
{

    // 初始状态全部停止，防止上电乱转
    HAL_TIM_PWM_Stop(&MOTOR_TIM, MOTOR_CHANNEL_U);
    HAL_TIM_PWM_Stop(&MOTOR_TIM, MOTOR_CHANNEL_V);
    HAL_TIM_PWM_Stop(&MOTOR_TIM, MOTOR_CHANNEL_W);

		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); 
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); 
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3); 
	
	
	

		
}

//=======================修改arr来改变占空比的=======================
void Set_MOTOR_Compare(uint32_t Channel, uint32_t CompareValue)
{
    // 1. 获取当前定时器的ARR自动重装载值（即最大周期计数值）
    uint32_t period = __HAL_TIM_GET_AUTORELOAD(&MOTOR_TIM);
    
    // 2. 安全限幅：确保传入的比较值不会超过周期值
    // 如果超过周期值，PWM波形将保持全高（或全低），可能导致不可控状态
    if (CompareValue > period) 
    {
        CompareValue = period;
    }

    // 3. 将计算好的值写入对应的捕获/比较寄存器 (CCR)
    __HAL_TIM_SET_COMPARE(&MOTOR_TIM, Channel, CompareValue);
    
    // 4. 确保该PWM通道处于开启状态
    HAL_TIM_PWM_Start(&MOTOR_TIM, Channel);
}

//=====================================================================

//==========================修改占空比的api============================
void Set_MOTOR_Duty(uint32_t Channel, float DutyCycle)
{
    // 获取ARR自动重装载值，确保兼容性
    uint32_t period = __HAL_TIM_GET_AUTORELOAD(&MOTOR_TIM);
    
    // 限制范围
    if(DutyCycle > 100.0f) DutyCycle = 100.0f;
    if(DutyCycle < 0.0f) DutyCycle = 0.0f;

    // 计算CCR值
    uint32_t pulse = (uint32_t)(period * (DutyCycle / 100.0f));

    // 设置比较值
    __HAL_TIM_SET_COMPARE(&MOTOR_TIM, Channel, pulse);
    
    // 确保开启该通道
    HAL_TIM_PWM_Start(&MOTOR_TIM, Channel);
}



//=======================================================
// 停止某相输出 (实现高阻态/悬空)
void Stop_MOTOR_Phase(uint32_t Channel)
{
    HAL_TIM_PWM_Stop(&MOTOR_TIM, Channel);
    // 可选：将CCR清零，视具体驱动芯片逻辑而定
    __HAL_TIM_SET_COMPARE(&MOTOR_TIM, Channel, 0); 
}

