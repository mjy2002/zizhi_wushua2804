#include "as5600_app.h"

// 实例化唯一的编码器对象
AS5600_I2C as5600_encoder;

// 定义全局变量，方便在仿真器或调试窗口观察
float as5600_angle = 0.0f;
float as5600_speed = 0.0f;
float as5600_speed_filtered = 0.0f;

#define _powtwo(n) (1 << (n))

// 默认配置：0x36地址，12位分辨率，0x0C角度寄存器
AS5600_I2CConfig_s AS5600_I2C_Config = {
    .chip_address   = 0x36,
    .bit_resolution = 12,
    .angle_register = 0x0C,
    .data_start_bit = 11
};

/**
 * @brief 获取原始传感器角度值并转换为单圈弧度 (0~2π)
 * @return 弧度值 (float)
 */
float AS5600_I2C_getSensorAngle(AS5600_I2C* dev) {
    // 原始值 (0-4095) / 4096 * 2π
    return (AS5600_I2C_getRawCount(dev) / (float)dev->cpr) * _2PI;
}

/**
 * @brief 从 I2C 寄存器读取原始 12 位角度数据
 */
int AS5600_I2C_getRawCount(AS5600_I2C* dev) {
    return AS5600_I2C_read(dev, dev->angle_register_msb);
}

/**
 * @brief I2C 底层读取函数：连续读取两个字节并合并
 */
int AS5600_I2C_read(AS5600_I2C* dev, uint8_t angle_reg_msb) {
    uint8_t buffer[2];
    HAL_StatusTypeDef status;
    
    // AS5600 角度寄存器通常是 0x0C (MSB) 和 0x0D (LSB)
    status = HAL_I2C_Mem_Read(dev->hi2c1, (dev->chip_address << 1), 
                              angle_reg_msb, I2C_MEMADD_SIZE_8BIT, 
                              buffer, 2, 10);
    
    if (status != HAL_OK) {
        dev->currWireError = status;
        return -1;
    }
    
    // 合并 12 位数据：高位左移 8 位与低位进行或运算
    return (int)((buffer[0] << 8) | buffer[1]);
}

/**
 * @brief 获取累计多圈弧度值
 * @note 该函数会自动处理“过零”检测，即检测从 2π 跨越到 0 或反向跨越
 */
float AS5600_I2C_Sensor_return(AS5600_I2C* dev) {
    if (!dev->sensor.initialized) return NAN;

    float current_angle = AS5600_I2C_getSensorAngle(dev); 
    if (current_angle < 0) return NAN;

    // 计算当前角度与上次记录角度的差值
    float angle_diff = current_angle - dev->sensor.angle_prev;

    // 过零逻辑：如果差值超过了 0.8 * 2π (约288度)，判定为跨圈了
    if (fabsf(angle_diff) > (0.8f * _2PI)) {
        // 差值为正则说明是逆时针过零(由正转负)，圈数减1；反之加1
        dev->sensor.full_rotations += (angle_diff > 0) ? -1 : 1;
    }

    dev->sensor.angle_prev = current_angle;
    dev->sensor.angle_prev_ts = _micros(); // 更新时间戳（通常用 TIM 计数器实现）

    // 返回：(总圈数 * 2π) + 当前单圈位置
    return (dev->sensor.full_rotations * _2PI) + current_angle;
}

/**
 * @brief 计算角速度 (rad/s)
 */
float AS5600_I2C_Sensor_getVelocity(AS5600_I2C* dev) {
    if (!dev->sensor.initialized) return 0.0f;

    // 计算两次采样的时间间隔 Ts (单位转换为秒)
    float Ts = (dev->sensor.angle_prev_ts - dev->sensor.vel_angle_prev_ts) * 1e-6f;

    // 异常处理：防止时间戳溢出或太小导致除以零
    if (Ts <= 0.0f || Ts < dev->sensor.min_elapsed_time) {
        return dev->sensor.velocity;
    }

    // 速度计算公式：Δ角度 / Δ时间
    // 这里包含跨圈后的总角度差
    dev->sensor.velocity = (
        (float)(dev->sensor.full_rotations - dev->sensor.vel_full_rotations) * _2PI +
        (dev->sensor.angle_prev - dev->sensor.vel_angle_prev)
    ) / Ts;

    // 保存当前状态供下次计算使用
    dev->sensor.vel_angle_prev = dev->sensor.angle_prev;
    dev->sensor.vel_full_rotations = dev->sensor.full_rotations;
    dev->sensor.vel_angle_prev_ts = dev->sensor.angle_prev_ts;

    return dev->sensor.velocity;
}

/**
 * @brief I2C 强行复位序列
 * @note 当 I2C 从机（如 AS5600）在传输中因干扰导致 SDA 一直被拉低时，
 * 主机需要手动切换引脚为 GPIO 模式，翻转 SCL 时钟信号 9 次来强制释放 SDA。
 */
void I2C_Bus_Reset(GPIO_TypeDef* SCL_Port, uint16_t SCL_Pin, GPIO_TypeDef* SDA_Port, uint16_t SDA_Pin)
{
    // 使能 GPIO 时钟
    if(SCL_Port == GPIOA || SDA_Port == GPIOA) __HAL_RCC_GPIOA_CLK_ENABLE();
    if(SCL_Port == GPIOB || SDA_Port == GPIOB) __HAL_RCC_GPIOB_CLK_ENABLE();
    if(SCL_Port == GPIOC || SDA_Port == GPIOC) __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = SCL_Pin | SDA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; 
    GPIO_InitStruct.Pull = GPIO_NOPULL; 
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(SCL_Port, &GPIO_InitStruct);

    // 释放 SDA 和 SCL
    HAL_GPIO_WritePin(SDA_Port, SDA_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SCL_Port, SCL_Pin, GPIO_PIN_SET);
    HAL_Delay(1);

    // 产生 9 个时钟脉冲，告诉从机主机还活着，请释放 SDA
    for(int i = 0; i < 9; i++)
    {
        HAL_GPIO_WritePin(SCL_Port, SCL_Pin, GPIO_PIN_RESET);
        HAL_Delay(1); 
        HAL_GPIO_WritePin(SCL_Port, SCL_Pin, GPIO_PIN_SET);
        HAL_Delay(1); 
    }
    
    // 发送一个 I2C STOP 信号的模拟波形
    HAL_GPIO_WritePin(SCL_Port, SCL_Pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(SDA_Port, SDA_Pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(SCL_Port, SCL_Pin, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(SDA_Port, SDA_Pin, GPIO_PIN_SET);
    HAL_Delay(1);
}

/**
 * @brief 加载传感器寄存器和分辨率配置
 */
void AS5600_I2C_Init_Config(AS5600_I2C* dev, AS5600_I2CConfig_s config) {
    dev->chip_address = config.chip_address;
    dev->angle_register_msb = config.angle_register;
    dev->cpr = _powtwo(config.bit_resolution);
    int bits_used_msb = config.data_start_bit - 7;
    dev->lsb_used = config.bit_resolution - bits_used_msb;
    dev->lsb_mask = (uint8_t)((2 << dev->lsb_used) - 1);
    dev->msb_mask = (uint8_t)((2 << bits_used_msb) - 1);
    dev->hi2c1 = NULL;
    dev->currWireError = 0;
    dev->sensor.initialized = 0;
}

/**
 * @brief 绑定硬件 I2C 句柄
 */
void AS5600_I2C_Init_HW(AS5600_I2C* dev, I2C_HandleTypeDef* hi2c) {
    dev->hi2c1 = hi2c; 
    dev->sensor.initialized = 1;
}

/**
 * @brief 传感器状态机初始化（清零并读取初始位置）
 */
void AS5600_I2C_Sensor_init(AS5600_I2C* dev) {
    AS5600_I2C_getSensorAngle(dev);
    HAL_Delay(1);
    dev->sensor.vel_angle_prev = AS5600_I2C_getSensorAngle(dev);
    dev->sensor.vel_angle_prev_ts = _micros();

    HAL_Delay(1);

    AS5600_I2C_getSensorAngle(dev);
    HAL_Delay(1);
    dev->sensor.angle_prev = AS5600_I2C_getSensorAngle(dev);
    dev->sensor.angle_prev_ts = _micros();

    dev->sensor.full_rotations = 0;
    dev->sensor.vel_full_rotations = 0;
    dev->sensor.velocity = 0.0f;
    dev->sensor.min_elapsed_time = 0.001f;
    dev->sensor.initialized = 1;
}

//extern MPU6050_t MPU6050;
// 假设 lpf1 在你的 bsp 中有定义，这里保持调用
extern LowPassFilter lpf1;

uint8_t speed_temp = 1;
void read_speed_proc(void)
{
//    // 1. 获取单路角度 (单位: rad)
//    as5600_angle = AS5600_I2C_Sensor_return(&as5600_encoder);
//    
//    // 2. 获取单路速度 (单位: rad/s)
//    as5600_speed = AS5600_I2C_Sensor_getVelocity(&as5600_encoder);
//    
//    // 3. 对单路速度进行低通滤波
//    as5600_speed_filtered = LPF_Update(&lpf1, as5600_speed);
    
    // 如果需要调试打印，可以直接解除注释并修改变量名
    
    //打印速度
//    my_printf(&huart1,"{algg}%.2f,%.2f\r\n",target,as5600_speed);

			my_printf(&huart1,"{algg}%.4f,%.4f\r\n",as5600_angle,as5600_speed);
    //打印角度变化
//    my_printf(&huart1,"{algg}%.2f,%.2f\r\n",target,as5600_angle);
}

