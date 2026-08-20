#include "oled_app.h"
#include "uart_app.h"

//执行OLED的显示
//OLED有问题晚一些修


uint8_t huancun_buf[20];

void oled_init(void)
{
	OLED_Init();
	OLED_Clear();
}

void oled_proc(void)
{
	// 显示传感器数据
	if(moshi == 1)
	{
		
		
		// 显示温度
		sprintf((char *)huancun_buf,"Temp: ");
		OLED_ShowString(0,0,huancun_buf,8);
		
	}

	else if(moshi == 2)
	{
		
	}
	
}


