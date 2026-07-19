#ifndef __SPEED_CONTROL_H__
#define __SPEED_CONTROL_H__

#include <stdint.h>

/* ================================================================
 *  速度环 PID 参数（宏定义，方便调试）
 *
 *  注意：编码器有效更新周期为 20ms（4×5ms降采样）
 *
 *  调参要点：
 *    KP 太大 → 速度过冲震荡；KP 太小 → 响应慢
 *    KI 太大 → 积分饱和，超调严重；KI 太小 → 稳态误差消除慢
 *    KD 太大 → 对噪声敏感，电机抖动；KD 太小 → 阻尼不足
 *    INTEGRAL_LIMIT → 防止积分饱和，建议设为 OUTPUT_LIMIT 的 60%~80%
 *    OUTPUT_LIMIT   → 速度环输出上限，对应 mixer 的速度修正量
 * ================================================================ */
#define SPEED_KP             1.5f
#define SPEED_KI             0.6f
#define SPEED_KD             0.05f
#define SPEED_INTEGRAL_LIMIT 250.0f
#define SPEED_OUTPUT_LIMIT   350.0f

/**
 * @brief 初始化速度环
 * @param target_speed_mmps 目标线速度 (mm/s)
 */
void SpeedLoop_Init(float target_speed_mmps);

/**
 * @brief 更新速度环（仅编码器有新数据时才运行PID）
 * @param dt 距离上次主循环更新的时间间隔 (秒)
 */
void SpeedLoop_Update(float dt);

/**
 * @brief 获取速度环 PID 修正量（用于 Mixer_SetSpeedDiff）
 * @return 速度修正值，正值=加速，负值=减速
 */
int16_t SpeedLoop_GetCorrection(void);

/**
 * @brief 修改目标速度
 * @param target_speed_mmps 新的目标线速度 (mm/s)
 */
void SpeedLoop_SetTarget(float target_speed_mmps);

/**
 * @brief 获取当前实际平均速度
 * @return 当前平均线速度 (mm/s)
 */
float SpeedLoop_GetCurrentSpeed(void);

/**
 * @brief 获取当前速度误差
 * @return 速度误差 (mm/s)，正值=实际比目标慢
 */
float SpeedLoop_GetError(void);

/**
 * @brief 重置速度环（清除积分和误差历史）
 */
void SpeedLoop_Reset(void);

#endif /* __SPEED_CONTROL_H__ */