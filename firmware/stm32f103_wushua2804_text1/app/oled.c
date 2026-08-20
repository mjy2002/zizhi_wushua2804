#include "oled.h"
#include "i2c.h"
#include "oledfont.h"

// OLED 使用 I2C2
#define OLED_I2C_HANDLE        hi2c2

// SSD1306 常见 I2C 地址：0x3C
// HAL 库中地址需要左移 1 位，所以是 0x78
#define OLED_I2C_ADDR          0x78

#define OLED_CMD_ADDR          0x00
#define OLED_DATA_ADDR         0x40

uint8_t CMD_Data[] =
{
    0xAE, 0x00, 0x10, 0x40, 0xB0, 0x81, 0xFF, 0xA1, 0xA6, 0xA8, 0x3F,

    0xC8, 0xD3, 0x00, 0xD5, 0x80, 0xD8, 0x05, 0xD9, 0xF1, 0xDA, 0x12,

    0xD8, 0x30, 0x8D, 0x14, 0xAF
};


// 写入 OLED 初始化命令
void WriteCmd(void)
{
    uint8_t i = 0;

    for (i = 0; i < sizeof(CMD_Data); i++)
    {
        HAL_I2C_Mem_Write(&OLED_I2C_HANDLE,
                          OLED_I2C_ADDR,
                          OLED_CMD_ADDR,
                          I2C_MEMADD_SIZE_8BIT,
                          &CMD_Data[i],
                          1,
                          0x100);
    }
}

// 向 OLED 写命令
void OLED_WR_CMD(uint8_t cmd)
{
    HAL_I2C_Mem_Write(&OLED_I2C_HANDLE,
                      OLED_I2C_ADDR,
                      OLED_CMD_ADDR,
                      I2C_MEMADD_SIZE_8BIT,
                      &cmd,
                      1,
                      0x100);
}

// 向 OLED 写数据
void OLED_WR_DATA(uint8_t data)
{
    HAL_I2C_Mem_Write(&OLED_I2C_HANDLE,
                      OLED_I2C_ADDR,
                      OLED_DATA_ADDR,
                      I2C_MEMADD_SIZE_8BIT,
                      &data,
                      1,
                      0x100);
}

// 初始化 OLED 屏幕
void OLED_Init(void)
{
    HAL_Delay(200);
    WriteCmd();
    OLED_Clear();
}

// 清屏
void OLED_Clear(void)
{
    uint8_t i, n;

    for (i = 0; i < 8; i++)
    {
        OLED_WR_CMD(0xB0 + i);
        OLED_WR_CMD(0x00);
        OLED_WR_CMD(0x10);

        for (n = 0; n < 128; n++)
        {
            OLED_WR_DATA(0);
        }
    }
}

void OLED_Set_Pos(uint8_t x, uint8_t y)
{
    OLED_WR_CMD(0xB0 + y);
    OLED_WR_CMD(((x & 0xF0) >> 4) | 0x10);
    OLED_WR_CMD(x & 0x0F);
}

unsigned int oled_pow(uint8_t m, uint8_t n)
{
    unsigned int result = 1;

    while (n--)
    {
        result *= m;
    }

    return result;
}

// 显示数字
void OLED_ShowNum(uint8_t x, uint8_t y, unsigned int num, uint8_t len, uint8_t size2)
{
    uint8_t t, temp;
    uint8_t enshow = 0;

    for (t = 0; t < len; t++)
    {
        temp = (num / oled_pow(10, len - t - 1)) % 10;

        if (enshow == 0 && t < (len - 1))
        {
            if (temp == 0)
            {
                OLED_ShowChar(x + (size2 / 2) * t, y, ' ', size2);
                continue;
            }
            else
            {
                enshow = 1;
            }
        }

        OLED_ShowChar(x + (size2 / 2) * t, y, temp + '0', size2);
    }
}

// 显示单个字符
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t Char_Size)
{
    uint8_t c = 0;
    uint8_t i = 0;

    c = chr - ' ';

    if (x > 127)
    {
        x = 0;
        y = y + 2;
    }

    if (Char_Size == 16)
    {
        OLED_Set_Pos(x, y);

        for (i = 0; i < 8; i++)
        {
            OLED_WR_DATA(F8X16[c * 16 + i]);
        }

        OLED_Set_Pos(x, y + 1);

        for (i = 0; i < 8; i++)
        {
            OLED_WR_DATA(F8X16[c * 16 + i + 8]);
        }
    }
    else
    {
        OLED_Set_Pos(x, y);

        for (i = 0; i < 6; i++)
        {
            OLED_WR_DATA(F6x8[c][i]);
        }
    }
}

// 显示字符串
void OLED_ShowString(uint8_t x, uint8_t y, uint8_t *chr, uint8_t Char_Size)
{
    uint8_t j = 0;

    while (chr[j] != '\0')
    {
        OLED_ShowChar(x, y, chr[j], Char_Size);
        x += 8;

        if (x > 120)
        {
            x = 0;
            y += 2;
        }

        j++;
    }
}

// 显示汉字
void OLED_ShowCHinese(uint8_t x, uint8_t y, uint8_t no)
{
    uint8_t t;

    OLED_Set_Pos(x, y);

    for (t = 0; t < 16; t++)
    {
        OLED_WR_DATA(Hzk[2 * no][t]);
    }

    OLED_Set_Pos(x, y + 1);

    for (t = 0; t < 16; t++)
    {
        OLED_WR_DATA(Hzk[2 * no + 1][t]);
    }
}



