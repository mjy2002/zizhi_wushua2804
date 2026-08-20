#include "led_app.h"

void led_proc(void)
{
	HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_0);
}
