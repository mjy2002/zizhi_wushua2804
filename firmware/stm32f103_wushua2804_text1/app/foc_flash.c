#include "foc_flash.h"
#include <string.h>
#include <math.h>

#define FOC_FLASH_MAGIC      0x59464F43UL   // 'YFOC'
#define FOC_FLASH_VERSION    0x00000001UL

/*
 * STM32F103C8 常见 Flash:
 * 64KB:  0x08000000 ~ 0x0800FFFF
 * 最后一页地址: 0x0800FC00
 *
 * 如果你是 128KB Flash，例如 F103CB，最后一页可能是 0x0801FC00。
 *
 * 更稳妥的写法：从芯片内部 Flash size 寄存器读取容量，然后取最后 1KB。
 */
#define FOC_FLASH_PAGE_SIZE  1024U
#define FOC_FLASH_ADDR       (FLASH_BASE + ((*((uint16_t *)FLASHSIZE_BASE)) * 1024U) - FOC_FLASH_PAGE_SIZE)

typedef struct
{
    uint32_t magic;
    uint32_t version;
    float zero_angle;
    int32_t pp;
    int32_t dir;
    uint32_t checksum;
} FOC_FlashData_t;

static uint32_t FOC_CalcChecksum(const FOC_FlashData_t *data)
{
    union
    {
        float f;
        uint32_t u32;
    } angle;

    angle.f = data->zero_angle;

    return data->magic ^
           data->version ^
           angle.u32 ^
           (uint32_t)data->pp ^
           (uint32_t)data->dir;
}

uint8_t FOC_Flash_LoadZeroAngle(float *zero_angle, int pp, int dir)
{
    const FOC_FlashData_t *data = (const FOC_FlashData_t *)FOC_FLASH_ADDR;

    if (data->magic != FOC_FLASH_MAGIC)
    {
        return 0;
    }

    if (data->version != FOC_FLASH_VERSION)
    {
        return 0;
    }

    if (data->checksum != FOC_CalcChecksum(data))
    {
        return 0;
    }

    if (data->pp != pp)
    {
        return 0;
    }

    if (data->dir != dir)
    {
        return 0;
    }

    if (data->zero_angle < 0.0f || data->zero_angle > 6.28318530718f)
    {
        return 0;
    }

    *zero_angle = data->zero_angle;
    return 1;
}

HAL_StatusTypeDef FOC_Flash_SaveZeroAngle(float zero_angle, int pp, int dir)
{
    HAL_StatusTypeDef status;
    FLASH_EraseInitTypeDef erase_init;
    uint32_t page_error = 0;

    FOC_FlashData_t data;

    data.magic = FOC_FLASH_MAGIC;
    data.version = FOC_FLASH_VERSION;
    data.zero_angle = zero_angle;
    data.pp = pp;
    data.dir = dir;
    data.checksum = FOC_CalcChecksum(&data);

    HAL_FLASH_Unlock();

    erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
    erase_init.PageAddress = FOC_FLASH_ADDR;
    erase_init.NbPages = 1;

    status = HAL_FLASHEx_Erase(&erase_init, &page_error);
    if (status != HAL_OK)
    {
        HAL_FLASH_Lock();
        return status;
    }

    uint32_t *p_data = (uint32_t *)&data;
    uint32_t address = FOC_FLASH_ADDR;

    for (uint32_t i = 0; i < sizeof(FOC_FlashData_t) / 4; i++)
    {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, p_data[i]);
        if (status != HAL_OK)
        {
            HAL_FLASH_Lock();
            return status;
        }

        address += 4;
    }

    HAL_FLASH_Lock();

    return HAL_OK;
}

HAL_StatusTypeDef FOC_Flash_ClearZeroAngle(void)
{
    HAL_StatusTypeDef status;
    FLASH_EraseInitTypeDef erase_init;
    uint32_t page_error = 0;

    HAL_FLASH_Unlock();

    erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
    erase_init.PageAddress = FOC_FLASH_ADDR;
    erase_init.NbPages = 1;

    status = HAL_FLASHEx_Erase(&erase_init, &page_error);

    HAL_FLASH_Lock();

    return status;
}


