#include "scheduler.h"

// 任务调度器 == 方便每个模块的调用时间管理

uint8_t task_num;

typedef struct
{
    void (*task_func)(void);
    uint32_t task_time;
    uint32_t last_time;
} task_t;

static task_t task_scheduler[] =
{
    //{oled_proc,200,0},
    //{Velocity_proc,5,0},
    //{open_loop_proc,5,0},
    {uart_proc, 20, 0},
    {key_proc, 20, 0},
    //{led_proc,1000,0},
    //{read_speed_proc,100,0},
    //{SVPWM_OpenLoop_Test,5,0},
    //{adc_proc,20,0},
    {CAN_App_ProcessRxTask, 10, 0},
};

void init_scheduler(void)
{
    task_num = sizeof(task_scheduler) / sizeof(task_t);
}

void scheduler_run(void)
{
    for (int i = 0; i < task_num; i++)
    {
        uint32_t scheduler_time = HAL_GetTick();

        if (scheduler_time >= task_scheduler[i].task_time +
                              task_scheduler[i].last_time)
        {
            task_scheduler[i].task_func();
            task_scheduler[i].last_time = scheduler_time;
        }
    }
}

uint8_t move_count = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // 检查是不是 TIM4 触发的中断
    if (htim->Instance == TIM4)
    {
        /*
         * 高频 FOC 输出环：
         *   每次 TIM4 中断都输出一次 SVPWM。
         */
        loopFOC();

        /*
         * 低速位置爬坡外环：
         *   每 2 次中断执行一次 move(target)。
         *   YiFOC.c 里的 LOW_SPEED_RAMP_DEFAULT_DT 按 2ms 兜底处理。
         */
        move_count++;

        if (move_count >= 2)
        {
            move(target);
            move_count = 0;
        }
    }

    return;
}

void init_all(void)
{
    DWT_Timer_Init();

    /*
     * 重要修改：
     *   现在 controller = Type_velocity 时，target 表示“速度”，单位 rad/s。
     *   例如 target = 1.0f，就是让电机按 1rad/s 低速匀速爬坡运行。
     *
     *   不要再把 target = 1.2f 理解成目标角度。
     *   如果你要跑位置角度模式，再把 controller 改回 Type_angle。
     */
    target = 0.0f;

    // 使能无刷电机
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);

    /* --- AS5600 软件实例初始化 --- */
    AS5600_I2C_Init_Config(&as5600_encoder, AS5600_I2C_Config);
    AS5600_I2C_Init_HW(&as5600_encoder, &hi2c1);
    AS5600_I2C_Sensor_init(&as5600_encoder);

    // 初始化滤波器
    LPF_Init(&lpf1, 0.0005f);
    LPF_Init(&LPF_current_q, 0.001f);
    LPF_Init(&LPF_current_d, 0.001f);

    /*
     * 速度 PID 仍然保留：
     *   当 |target| > LOW_SPEED_RAMP_THRESHOLD 时，YiFOC.c 会自动切回普通速度闭环。
     *   低速爬坡模式下不会使用这个速度 PID。
     */
    PID_Init(&pid_motor_velocity, 0.10f, 1.0f, 0.0f, 6.0f, 100.0f);

    // 对应的值：P, I, D, 输出限制, 斜率限制
    PID_Init(&pid_motor_angle, 3.5f, 0.0f, 0.01f, 20.0f, 0.0f);

    PID_Init(&PID_current_q, 0.7f, 0.5f, 0.0f, 5.0f, 0.0f);
    PID_Init(&PID_current_d, 0.6f, 0.0f, 0.0f, 10.0f, 0.0f);

    // PWM 完成初始化后才能对电机电角度校准
    PWM_Init_All();

#if (CALIBRATION_MODE == 1)

    /* 方案1：每次开机自动校准 */
    voltage_power_supply1 = 12.0f;
    motor_state = MOTOR_STATE_CALIBRATING;
    FOC_Init1(12.0f);  // 这会调用Check_Sensor1()进行校准
    motor_state = MOTOR_STATE_READY;

#else

    /* 方案2：尝试从Flash加载零电角度 */
    voltage_power_supply1 = 12.0f;

    if (FOC_Flash_LoadZeroAngle(&zero_electric_Angle1, pp1, Dir1))
    {
        /* Flash中有有效数据，直接进入READY状态，免校准启动 */
        motor_state = MOTOR_STATE_READY;
    }
    else
    {
        /* Flash中没有有效数据，保持UNCALIBRATED状态，等待按键校准 */
        motor_state = MOTOR_STATE_UNCALIBRATED;
    }

#endif

    /*
     * 重要修改：
     *   这里改成 Type_velocity。
     *   这样 target = 1.0f 会进入 YiFOC.c 的低速位置爬坡逻辑。
     */
    torque_controller = Type_voltage;
    controller = Type_velocity;

    uart_app_init();
    HAL_TIM_Base_Start_IT(&htim4);

    my_printf(&huart1, "chushichenggong\r\n");

    // ADC 初始化
    HAL_ADCEx_Calibration_Start(&hadc1);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)AD_Value, 2);

    CurrentSense_config(0.01f, 50.0f);
    InlineCurrentSense_Init();

    /*
     * 如果使用电流环，先要使用普通方式矫正，防止电流环导致抽搐。
     * 当前低速爬坡推荐先使用 Type_voltage。
     */
    torque_controller = Type_voltage;

    init_scheduler();

    if (CAN_App_Init() != HAL_OK)
    {
        my_printf(&huart1, "CAN Init Failed!\r\n");
    }
    else
    {
        my_printf(&huart1, "CAN Init Success!\r\n");
    }
}

