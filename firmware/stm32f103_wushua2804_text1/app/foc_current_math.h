#ifndef __foc_current_math_H__
#define __foc_current_math_H__

#include "bsp_system.h"

float getDCCurrent(float motor_electrical_angle);
DQCurrent_s getFOCCurrents(float angle_el);

#endif


