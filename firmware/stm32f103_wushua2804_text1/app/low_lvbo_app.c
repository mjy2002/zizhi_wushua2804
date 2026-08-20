#include "low_lvbo_app.h"



LowPassFilter lpf1;
LowPassFilter LPF_current_q;
LowPassFilter LPF_current_d;

// ��ʼ��
void LPF_Init(LowPassFilter* lpf, float time_constant)
{
    lpf->Tf = time_constant;
    lpf->y_prev = 0.0f;
    lpf->timestamp_prev = DWT_Get_Microsecond();
}

// �˲�����
float LPF_Update(LowPassFilter* lpf, float x)
{
    uint32_t timestamp_now = DWT_Get_Microsecond();
    float dt = 0.0f;

    // ���� dt (���� DWT ���)
    if (timestamp_now < lpf->timestamp_prev) {
        uint32_t us_diff = 0xFFFFFFFF - lpf->timestamp_prev + timestamp_now;
        dt = (float)us_diff / 1000000.0f;
    } else {
        uint32_t us_diff = timestamp_now - lpf->timestamp_prev;
        dt = (float)us_diff / 1000000.0f;
    }

    // 1. �����쳣ʱ���� (��ֹ����0����)
    if (dt <= 0.0f) dt = 1e-4f; // ��һ����Сֵ�������

    // 2. ֻ�е�ʱ������ķǳ���(���糬��0.5��)�����ã�
    //    ��ͨ�� 10ms-20ms ѭ����Ӧ�������˲�����
    if (dt > 0.5f) 
    {
        lpf->y_prev = x;
        lpf->timestamp_prev = timestamp_now;
        return x;
    }

    // 3. �����˲�ϵ�� alpha
    // ��ʽ: y = alpha * y_prev + (1 - alpha) * x
    // alpha = Tf / (Tf + dt)
    float alpha = lpf->Tf / (lpf->Tf + dt);
    float y = alpha * lpf->y_prev + (1.0f - alpha) * x;

    // 4. ����״̬
    lpf->y_prev = y;
    lpf->timestamp_prev = timestamp_now;

    return y;
}

