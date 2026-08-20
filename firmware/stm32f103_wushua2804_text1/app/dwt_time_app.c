#include "dwt_time_app.h"

uint32_t DWT_Get_Microsecond(void);
/*
**********************************************************************
* 时间戳相关寄存器定义
**********************************************************************
*/
#define  DWT_CR      *(__IO uint32_t *)0xE0001000   // DWT Control Register
#define  DWT_CYCCNT  *(__IO uint32_t *)0xE0001004   // DWT Cycle Count Register
#define  DEM_CR      *(__IO uint32_t *)0xE000EDFC   // Debug Exception and Monitor Control Register
#define  DEM_CR_TRCENA                   (1 << 24)  // DEMCR TRCENA bit
#define  DWT_CR_CYCCNTENA                (1 <<  0)  // DWT CYCCNTENA bit

// 静态变量
__IO static uint32_t g_Initialized = 0; // 初始化标志位
__IO static uint32_t g_Timestamp_us = 0;
__IO static uint32_t g_SysClock = 0; // 系统时钟频率
__IO static uint32_t g_UsCNT = 0;    // 1us所需的计数值 (Cycles per microsecond)

/*****************************************************************************************************
* describe: 初始化时间戳
* note: STM32F407VET6 @ 168MHz, g_UsCNT 将会被计算为 168
*****************************************************************************************************/
HAL_StatusTypeDef DWT_Timer_Init(void)
{
    /* 1. 使能DWT外设 (Debug Exception and Monitor Control Register) */
    DEM_CR |= (uint32_t)DEM_CR_TRCENA; 
    
    /* 2. 清空计数器 */
    DWT_CYCCNT = (uint32_t)0u;
    
    /* 3. 使能Cortex-M DWT CYCCNT寄存器 */
    DWT_CR |= (uint32_t)DWT_CR_CYCCNTENA;
    
    /* 4. 获取系统时钟频率 */
    // 如果你在main.c中正确配置了时钟树，这里会返回 168000000
    g_SysClock = HAL_RCC_GetSysClockFreq(); 
    
    /* 5. 计算1us所需的计数值 */
    // 168000000 / 1000000 = 168
    g_UsCNT = g_SysClock / 1000000U;
    
    g_Initialized = 1;
    return HAL_OK;
}

/*****************************************************************************************************
* describe: 读取当前DWT计数器的值 (Cycle Count)
*****************************************************************************************************/
uint32_t DWT_Get_CNT(void)
{
    return (uint32_t)DWT_CYCCNT;
}

/*****************************************************************************************************
* describe: 获取系统时钟频率
*****************************************************************************************************/
uint32_t DWT_Get_System_Clock_Freq(void)
{
    return g_SysClock;
}

/*****************************************************************************************************
* describe: 获取1us所需的计数值
*****************************************************************************************************/
uint32_t DWT_Get_UsCNT(void)
{
    return g_UsCNT;
}

/*****************************************************************************************************
* describe: 采用CPU的内部计数实现精确延时 (阻塞式)
* input: us -> 延迟的微秒数
* note: 利用无符号整型减法的溢出特性，无需手动处理翻转
*****************************************************************************************************/
void DWT_Delay_us(uint32_t us)
{
    uint32_t start_tick, current_tick, wait_cycles;
    
    if(0 == g_Initialized) DWT_Timer_Init();
    
    wait_cycles = us * g_UsCNT; // 计算需要等待的总Cycle数
    start_tick = DWT_CYCCNT;    // 记录进入时的Tick
    
    while(1)
    {
        current_tick = DWT_CYCCNT;
        
        // Unsigned int subtraction handles overflow automatically
        // 比如: current=10, start=0xFFFFFFF0 (非常大), result = 10 - (-16) = 26. 结果正确。
        if((current_tick - start_tick) >= wait_cycles)
        {
            break;
        }
    }
}

/*****************************************************************************************************
* describe: 提供us级的时间戳
* note: 必须定期调用此函数以防止DWT 32位寄存器溢出丢失多次计数
* STM32F407 @ 168MHz 溢出时间约为 25.5秒。
* 请确保调用间隔小于 25秒。
*****************************************************************************************************/
uint32_t DWT_Get_Microsecond(void)
{
    static uint32_t last_cycle_count = 0;
    static uint32_t accumulated_cycles = 0; // 用于累加不足1us的余数
    
    uint32_t current_cycle_count = DWT_CYCCNT;
    uint32_t delta_cycles;
    
    if(0 == g_Initialized) DWT_Timer_Init();
    
    // 计算自上次调用以来经过的 cycle 数 (利用无符号减法处理溢出)
    delta_cycles = current_cycle_count - last_cycle_count;
    
    last_cycle_count = current_cycle_count; // 更新旧值
    
    accumulated_cycles += delta_cycles;     // 累加当前的 cycles
    
    if(accumulated_cycles >= g_UsCNT)
    {
        uint32_t us_increment = accumulated_cycles / g_UsCNT;
        g_Timestamp_us += us_increment;
        accumulated_cycles -= (us_increment * g_UsCNT); // 保留不足1us的余数
    }
    
    return g_Timestamp_us;
}

// 兼容 Arduino 风格的 API
void _delay(unsigned long ms){
  HAL_Delay(ms); 
}

unsigned long _micros(void){
  return DWT_Get_Microsecond();
}
