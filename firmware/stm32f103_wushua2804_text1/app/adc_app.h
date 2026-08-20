#ifndef __adc_app_H__
#define __adc_app_H__

typedef struct
{
    float a;
    float b;
    float c;
} PhaseCurrent_s;



#include "bsp_system.h"

void adc_proc(void);

void CurrentSense_config(float shunt_resistor, float gain);
void InlineCurrentSense_Init(void);
PhaseCurrent_s getPhaseCurrents(void);

#endif
