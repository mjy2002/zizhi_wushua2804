#include "key_app.h"

/* 校准相关外部变量 */
extern volatile uint8_t calibration_requested;
extern volatile MotorState_e motor_state;

/*=========================================================

 * 按键定义

 * 硬件连接：

 * KEY1：PB3

 * KEY2：PA15

 *

 * 外部10k上拉：

 * 松开 = 高电平

 * 按下 = 低电平

 *=========================================================*/



#define KEY1_MASK              0x01U

#define KEY2_MASK              0x02U



#define KEY_DEBOUNCE_TIME      20U     // 软件消抖时间，单位ms


/* 当前稳定按键状态 */

uint8_t key_val = 0;



/* 上一次稳定按键状态 */

uint8_t key_old = 0;



/* 本次按下事件 */

uint8_t key_down = 0;



/* 本次释放事件 */

uint8_t key_up = 0;





/* 模式变量 */

uint8_t moshi = 1;





/**

 * @brief 读取机械按键状态

 *

 * @return 按键状态位

 *         bit0 = KEY1

 *         bit1 = KEY2

 *

 *         返回0：没有按键按下

 *         返回1：KEY1按下

 *         返回2：KEY2按下

 *         返回3：KEY1和KEY2同时按下

 *

 * @note 当前硬件为低电平有效

 */

uint8_t key_read(void)

{

    uint8_t temp = 0;



    /*

     * KEY1：PB3

     * 按下后引脚接地，因此低电平表示按下

     */

    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == GPIO_PIN_RESET)

    {

        temp |= KEY1_MASK;

    }



    /*

     * KEY2：PA15

     * 按下后引脚接地，因此低电平表示按下

     */

    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == GPIO_PIN_RESET)

    {

        temp |= KEY2_MASK;

    }



    return temp;

}





/**

 * @brief 按键扫描与处理函数

 *

 * @note 需要在main函数的while(1)中持续调用

 */

void key_proc(void)

{

    static uint8_t key_raw_old = 0;

    static uint32_t key_change_time = 0;



    uint8_t key_raw;

    uint32_t current_time;



    /* 每次进入先清除按下和释放事件 */

    key_down = 0;

    key_up = 0;



    /* 读取当前原始按键电平 */

    key_raw = key_read();



    /* 获取系统运行时间 */

    current_time = HAL_GetTick();

    /*

     * 如果原始按键状态发生变化，

     * 重新开始消抖计时

     */

    if (key_raw != key_raw_old)

    {

        key_raw_old = key_raw;

        key_change_time = current_time;

        return;

    }



    /*

     * 状态保持时间不足20ms，

     * 认为按键可能还在抖动

     */

    if ((current_time - key_change_time) < KEY_DEBOUNCE_TIME)

    {

        return;

    }



    /*

     * 原始状态已经稳定，

     * 但稳定状态没有变化，不需要处理

     */

    if (key_raw == key_val)

    {

        return;

    }



    /* 保存上一次稳定状态 */

    key_old = key_val;



    /* 更新当前稳定状态 */

    key_val = key_raw;



    /*

     * 计算新按下的按键：

     * 旧状态为0，新状态为1

     */

    key_down = key_val & ((uint8_t)(~key_old));



    /*

     * 计算新释放的按键：

     * 旧状态为1，新状态为0

     */

    key_up = key_old & ((uint8_t)(~key_val));





    /*=====================================================

     * KEY1按下处理

     *=====================================================*/

    if ((key_down & KEY1_MASK) != 0)
    {
        /* KEY1短按触发零电角度校准 */
        #if (CALIBRATION_MODE == 2)
        if (motor_state == MOTOR_STATE_UNCALIBRATED || motor_state == MOTOR_STATE_READY) {
            calibration_requested = 1;
        }
        #endif
    }





    /*=====================================================

     * KEY2按下处理

     *=====================================================*/

    if ((key_down & KEY2_MASK) != 0)

    {

        /* 在这里编写KEY2按下后的功能 */

    }





    /*=====================================================

     * KEY1释放处理，可按需使用

     *=====================================================*/

    if ((key_up & KEY1_MASK) != 0)

    {

        /* KEY1释放后的功能 */

    }





    /*=====================================================

     * KEY2释放处理，可按需使用

     *=====================================================*/

    if ((key_up & KEY2_MASK) != 0)

    {

        /* KEY2释放后的功能 */

    }

}



