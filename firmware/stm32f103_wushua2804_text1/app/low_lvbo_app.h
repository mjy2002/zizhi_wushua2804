#ifndef __LOW_LVBO_APP_H
#define __LOW_LVBO_APP_H

#include "main.h" // ���� DWT ������������Ͷ���
#include "bsp_system.h"
typedef struct {
    float Tf;              // �˲�ʱ�䳣�� (Time constant)
    float y_prev;          // ��һ�ε����ֵ
    uint32_t timestamp_prev; // ��һ�ε�ʱ���
} LowPassFilter;

// ��ʼ���˲���
void LPF_Init(LowPassFilter* lpf, float time_constant);

// �˲����㺯��
float LPF_Update(LowPassFilter* lpf, float x);
float LPFoperator(LowPassFilter* lpf, float x);

extern LowPassFilter lpf1;
extern LowPassFilter lpf2;
extern LowPassFilter LPF_current_q;
extern LowPassFilter LPF_current_d;

#endif

