#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "board.h"

// 定时器采样周期 (ms)
#define ENCODER_SAMPLE_PERIOD_MS    5

// 降采样系数：累加N次定时器采样后再计算速度
// 4次=20ms有效周期，约6~8脉冲/次，量化噪声降至~15%
#define ENCODER_DECIMATION          6

// 速度低通滤波系数 (0~1, 越小越平滑, 越大越灵敏)
// alpha=0.15 表示约 33ms 响应 63% 阶跃变化
#define ENCODER_SPEED_FILTER_ALPHA   0.15f

/***************************************************
    注意： 请根据您的实际硬件参数调整以下宏定义：

    ENCODER_RESOLUTION - 编码器分辨率（线数）
    MOTOR_GEAR_RATIO - 电机减速比
    WHEEL_DIAMETER_MM - 车轮直径(mm)
    
****************************************************/

// 电机与编码器参数
#define ENCODER_RESOLUTION          11          // 编码器分辨率（线数）
#define MOTOR_GEAR_RATIO            21.3f       // 电机减速比
#define WHEEL_DIAMETER_MM           65          // 车轮直径 (mm)

// 速度数据结构体
typedef struct {
    int32_t left_pulses;    // 左轮脉冲数
    int32_t right_pulses;   // 右轮脉冲数
    float left_speed_rpm;   // 左轮转速 (RPM)
    float right_speed_rpm;  // 右轮转速 (RPM)
    float left_speed_mmps;  // 左轮线速度 (mm/s)
    float right_speed_mmps; // 右轮线速度 (mm/s)
} WheelSpeed;

// 每个脉冲对应的行驶距离 (mm)，由硬件参数自动计算
// = (π × WHEEL_DIAMETER_MM) / (ENCODER_RESOLUTION × MOTOR_GEAR_RATIO)
#define MM_PER_PULSE  (3.1415926f * WHEEL_DIAMETER_MM / (ENCODER_RESOLUTION * MOTOR_GEAR_RATIO))

// ==================== 编码器速度计算函数声明 ====================

/**
 * @brief 更新车轮速度数据（累积式降采样）
 * @details 每5ms定时器中断累积脉冲，每20ms(4次)计算一次速度
 * @return 1=有新速度数据, 0=数据未更新
 */
uint8_t IRDM_update_wheel_speed(void);

/**
 * @brief 获取左轮转速
 * @return 左轮转速 (RPM)
 */
float IRDM_get_left_speed_rpm(void);

/**
 * @brief 获取右轮转速
 * @return 右轮转速 (RPM)
 */
float IRDM_get_right_speed_rpm(void);

/**
 * @brief 获取左轮线速度
 * @return 左轮线速度 (mm/s)
 */
float IRDM_get_left_speed_mmps(void);

/**
 * @brief 获取右轮线速度
 * @return 右轮线速度 (mm/s)
 */
float IRDM_get_right_speed_mmps(void);

/**
 * @brief 获取左右轮速度差
 * @return 速度差 (mm/s)，正值表示左轮比右轮快
 */
float IRDM_get_speed_diff(void);

/**
 * @brief 获取左右轮平均速度
 * @return 平均速度 (mm/s)
 */
float IRDM_get_average_speed(void);

/**
 * @brief 获取编码器脉冲计数
 * @param dir 0-左轮, 1-右轮
 * @return 脉冲计数值
 */
int32_t IRDM_get_encoder_pulses(int dir);

// ==================== 路程累积计算函数声明 ====================

/**
 * @brief 获取左右轮累计行驶路程
 * @param left_mm  [out] 左轮累计路程 (mm)
 * @param right_mm [out] 右轮累计路程 (mm)
 * @note  前进为正，后退为负
 */
void IRDM_get_distance(float *left_mm, float *right_mm);

/**
 * @brief 获取左右轮平均累计路程
 * @return 平均累计路程 (mm)
 */
float IRDM_get_average_distance(void);

/**
 * @brief 重置路程累积（左右轮均归零）
 */
void IRDM_reset_distance(void);

/**
 * @brief 获取每个脉冲对应的行驶距离
 * @return 每脉冲距离 (mm/pulse)
 */
float IRDM_get_mm_per_pulse(void);

#endif /* __ENCODER_H__ */