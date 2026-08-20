#ifndef __FOC_FLASH_H
#define __FOC_FLASH_H

#include "bsp_system.h"
#include <stdint.h>

uint8_t FOC_Flash_LoadZeroAngle(float *zero_angle, int pp, int dir);
HAL_StatusTypeDef FOC_Flash_SaveZeroAngle(float zero_angle, int pp, int dir);
HAL_StatusTypeDef FOC_Flash_ClearZeroAngle(void);

#endif
