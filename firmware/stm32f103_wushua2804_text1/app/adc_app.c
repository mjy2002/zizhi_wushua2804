/*
 * @Author: yihazui6 1319743179@qq.com
 * @Date: 2026-05-11 09:03:59
 * @LastEditors: yihazui6 1319743179@qq.com
 * @LastEditTime: 2026-05-11 15:54:56
 * @FilePath: \stm32f103_wushua2804_text1\app\adc_app.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "adc_app.h"


/************************************************
本程序仅供学习，引用代码请标明出处
创建日期：20260405
作    者：翼哈
b站名字：翼哈yiha

本代码中部分内容引用于：作者：loop222 @郑州 的源码
************************************************/


uint16_t AD_Value[2];

// 电流采样增益系数 (V -> A)
static float gain_a, gain_b;

// 零电流偏置 (ADC原始值)
static float offset_ia, offset_ib;

// 三相电流结果
PhaseCurrent_s current_data;

void adc_proc(void)
{
    //current_data = getPhaseCurrents();

    // 获取当前机械角度，用于关联电流与位置
    float current_mech_angle = AS5600_I2C_Sensor_return(&as5600_encoder);
    float el_angle = electricAngle(); 
    
    // 调用你写好的 FOC 电流计算函数，更新全局 current 变量
    current = getFOCCurrents(el_angle);
    // 打印：角度, Q轴电流 (或电压)
    //my_printf(&huart1, "{COG}%.3f,%.3f\r\n", current_mech_angle, current.q);
    // my_printf(&huart1,"{algg}%.2f,%.2f\r\n",voltage.q,current.q);
    my_printf(&huart1,"{algg}%.2f,%.2f\r\n",voltage.q,current_sp);
}

void CurrentSense_config(float shunt_resistor, float gain)
{
    float volts_to_amps_ratio = 1.0f / shunt_resistor / gain;

    gain_a =  volts_to_amps_ratio;
    gain_b = -volts_to_amps_ratio;

    my_printf(&huart1, "gain_a:%.4f\r\n", gain_a);
    my_printf(&huart1, "gain_b:%.4f\r\n", gain_b);
}

void InlineCurrentSense_Init(void)
{
    const int calibration_rounds = 1000;
    float offset_sum[2] = {0, 0};

    // 电机不转时，采样1000次取平均，得到0A电流时的ADC偏置值
    for (int i = 0; i < calibration_rounds; i++)
    {
        offset_sum[0] += AD_Value[0];
        offset_sum[1] += AD_Value[1];
        HAL_Delay(1);
    }

    offset_ia = offset_sum[0] / calibration_rounds;
    offset_ib = offset_sum[1] / calibration_rounds;

    my_printf(&huart1, "offset_ia:%.2f (ADC)\r\n", offset_ia);
    my_printf(&huart1, "offset_ib:%.2f (ADC)\r\n", offset_ib);
}

PhaseCurrent_s getPhaseCurrents(void)
{
    PhaseCurrent_s current;

    // DMA循环模式下，AD_Value由硬件自动更新，直接读取即可
    current.a = ((float)AD_Value[0] - offset_ia) * 3.3f / 4095.0f * gain_a;
    current.b = ((float)AD_Value[1] - offset_ib) * 3.3f / 4095.0f * gain_b;
    current.c = 0.0f - current.a - current.b;


    return current;
}
