#ifndef __as5600_app_H
#define __as5600_app_H

#include "bsp_system.h" // 包含 HAL 库及系统底层配置

/**
 * @brief AS5600 配置结构体
 */
typedef struct {
    int chip_address;   // 芯片 I2C 从机地址（通常为 0x36）
    int bit_resolution; // 分辨率，AS5600 为 12 位 (4096)
    int angle_register; // 角度数据寄存器首地址
    int data_start_bit; // 数据起始位
} AS5600_I2CConfig_s;

/**
 * @brief 传感器核心数据结构体（用于计算多圈角度和速度）
 */
typedef struct {
    uint32_t angle_prev_ts;     // 上一次读取角度的时间戳（微秒）
    float angle_prev;           // 上一次读取的单圈弧度值 (0 ~ 2PI)
    int32_t full_rotations;     // 累计旋转圈数
    float vel_angle_prev;       // 用于计算速度的上一次角度
    int32_t vel_full_rotations; // 用于计算速度的上一次圈数
    uint32_t vel_angle_prev_ts; // 用于计算速度的上一次时间戳
    float velocity;             // 当前计算出的角速度 (rad/s)
    float min_elapsed_time;     // 两次速度采样之间的最小时间间隔（秒）
    uint8_t initialized;        // 初始化标志位
} Sensor;

/**
 * @brief AS5600 硬件实例结构体
 */
typedef struct {
    Sensor sensor;              // 传感器数据对象
    float cpr;                  // 每圈数值（12位对应 4096）
    uint16_t lsb_used;          // 低字节位数
    uint8_t lsb_mask;           // 低字节掩码
    uint8_t msb_mask;           // 高字节掩码
    uint8_t angle_register_msb; // 角度寄存器高 8 位地址
    uint8_t chip_address;       // 芯片地址
    I2C_HandleTypeDef* hi2c1;   // 指向具体的 HAL I2C 句柄（决定使用哪个 I2C 接口）
    uint8_t currWireError;      // 记录最近一次 I2C 传输错误码
} AS5600_I2C;

// 外部全局变量声明
extern AS5600_I2CConfig_s AS5600_I2C_Config;
extern AS5600_I2C as5600_encoder;  // 单路编码器对象
extern float as5600_angle;          // 累计多圈弧度值
extern float as5600_speed_filtered; // 滤波后的速度 (rad/s)

/* --- 函数接口 --- */

// 初始化配置：设置芯片基本参数
void AS5600_I2C_Init(AS5600_I2C* dev, uint8_t _chip_address, int _bit_resolution, uint8_t _angle_register_msb, int _msb_bits_used);
void AS5600_I2C_Init_Config(AS5600_I2C* dev, AS5600_I2CConfig_s config);
// 绑定硬件：指定 I2C 句柄
void AS5600_I2C_Init_HW(AS5600_I2C* dev, I2C_HandleTypeDef* hi2c1);

// 数据获取
float AS5600_I2C_getSensorAngle(AS5600_I2C* dev); // 获取单圈弧度值
float AS5600_I2C_Sensor_return(AS5600_I2C* dev);  // 获取并更新多圈总弧度值

// 寄存器底层读写
int AS5600_I2C_getRawCount(AS5600_I2C* dev);
int AS5600_I2C_read(AS5600_I2C* dev, uint8_t angle_reg_msb);

// 高级功能
void AS5600_I2C_Sensor_init(AS5600_I2C* dev);     // 初始化传感器状态机（清零、采样初始值）
float AS5600_I2C_Sensor_getVelocity(AS5600_I2C* dev); // 计算实时角速度
void AS5600_I2C_Sensor_update(AS5600_I2C* dev);   // 仅更新状态不返回值的更新函数

void read_speed_proc(void); // 周期性调用的处理函数

// 硬件恢复：解决 I2C SDA 被从机拉死无法释放的问题
void I2C_Bus_Reset(GPIO_TypeDef* SCL_Port, uint16_t SCL_Pin, GPIO_TypeDef* SDA_Port, uint16_t SDA_Pin);

#define _2PI  6.28318530718f // 2π 定义

#endif

