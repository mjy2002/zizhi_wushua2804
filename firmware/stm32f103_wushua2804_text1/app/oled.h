#ifndef __OLED_H__
#define __OLED_H__

#include "bsp_system.h"

// 当前 OLED 使用硬件 I2C2
// I2C2_SCL -> PB10
// I2C2_SDA -> PB11

#define OLED_GPIO_CLK_ENABLE()          __HAL_RCC_GPIOB_CLK_ENABLE()

#define GPIOx_OLED_PORT                 GPIOB

#define OLED_SCL_PIN                    GPIO_PIN_10
#define OLED_SDA_PIN                    GPIO_PIN_11

void WriteCmd(void);
void OLED_WR_CMD(uint8_t cmd);
void OLED_WR_DATA(uint8_t data);
void OLED_Init(void);
void OLED_Clear(void);
void OLED_Set_Pos(uint8_t x, uint8_t y);
void OLED_ShowNum(uint8_t x, uint8_t y, unsigned int num, uint8_t len, uint8_t size2);
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t Char_Size);
void OLED_ShowString(uint8_t x, uint8_t y, uint8_t *chr, uint8_t Char_Size);
void OLED_ShowCHinese(uint8_t x, uint8_t y, uint8_t no);

#endif

