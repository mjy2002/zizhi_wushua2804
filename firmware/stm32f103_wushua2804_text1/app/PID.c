#include "pid.h"


// 1. ???????? PID ????
PID_Controller pid_motor_velocity;
PID_Controller pid_motor_angle;
PID_Controller PID_current_q;
PID_Controller PID_current_d;

// ???????????
static float _constrain(float amt, float low, float high) {
    if (amt < low) return low;
    else if (amt > high) return high;
    else return amt;
}
//PID_Init(&pid_motor, P, I, D, ??????, §Ò??????);
// ?????????????????????¨°???
void PID_Init(PID_Controller *pid, float p, float i, float d, float limit, float ramp)
{
    pid->P = p;
    pid->I = i;
    pid->D = d;
    pid->voltage_limit = limit;
    pid->ramp_limit = ramp;
    
    // ????????
    pid->integral_prev = 0.0f;
    pid->error_prev = 0.0f;
    pid->output_prev = 0.0f;
    pid->timestamp_prev = DWT_Get_Microsecond();
}
void PID_ResetState(PID_Controller *pid)
{
    pid->integral_prev = 0.0f;
    pid->error_prev    = 0.0f;
    pid->output_prev   = 0.0f;
    pid->timestamp_prev = DWT_Get_Microsecond();
}



float PID_brushless(PID_Controller *pid, float error)
{
    unsigned long now_us;
    unsigned long delta_us;
    float Ts;
    float proportional, integral, derivative, output;
    float output_rate;
    
    // 1. ???????????¦Ë us
    now_us = DWT_Get_Microsecond();
    
    // 2. ??????????? Ts
    // unsigned long ??????????????????? 32 ¦Ë???
    delta_us = now_us - pid->timestamp_prev;
    Ts = (float)delta_us * 1e-6f;
    
    // ????????
    pid->timestamp_prev = now_us;
    
    // ?????????????? 0 ????Ù²??????????
    if (Ts <= 0.0f || Ts > 0.5f) {
        Ts = 1e-3f;
    }
    
    // --- PID ???? ---
    
    // P????????
    proportional = pid->P * error;
    
    // I??????????? Tustin ???¦Ë???
    integral = pid->integral_prev + pid->I * Ts * 0.5f * (error + pid->error_prev);
    
    // ???????????????????
    integral = _constrain(integral, -pid->voltage_limit, pid->voltage_limit);
    
    // D??????????¦Ï? pid.c ??§Õ??
    // u_d(k) = D * (e(k) - e(k-1)) / Ts
    derivative = pid->D * (error - pid->error_prev) / Ts;
    
    // P + I + D ??????
    output = proportional + integral + derivative;
    
    // ??????
    output = _constrain(output, -pid->voltage_limit, pid->voltage_limit);
    
    // --- §Ò?????? (Ramp / Acceleration Limit) ---
    // ???¦Ï? pid.c????? ramp_limit > 0 ???????
    // ???? ramp_limit = 0 ???????????? output_prev
    if (pid->ramp_limit > 0.0f) {
        output_rate = (output - pid->output_prev) / Ts;
        
        if (output_rate > pid->ramp_limit) {
            output = pid->output_prev + pid->ramp_limit * Ts;
        } else if (output_rate < -pid->ramp_limit) {
            output = pid->output_prev - pid->ramp_limit * Ts;
        }
    }
    
    // --- ???????????????? ---
    pid->integral_prev = integral;
    pid->output_prev = output;
    pid->error_prev = error;
    
    return output;
}


