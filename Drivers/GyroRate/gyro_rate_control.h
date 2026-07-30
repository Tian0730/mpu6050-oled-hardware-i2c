#ifndef _GYRO_RATE_CONTROL_H_
#define _GYRO_RATE_CONTROL_H_

#include <stdint.h>
#include "car_config.h"

/* ================================================================
 *  角速度环 PID 参数
 *
 *  调参要点：
 *    KP 太大 → 转向过激，直行时左右摇摆
 *    KP 太小 → 角速度响应慢，抑制不足
 *    KI 太大 → 积分饱和，转弯后回正慢
 *    KI 太小 → 陀螺零偏补偿不足，直行缓慢偏航
 *    KD 太大 → 对噪声敏感，电机高频抖动
 *    KD 太小 → 角加速度阻尼不足，转弯后震荡
 *    INTEGRAL_LIMIT → 防止积分饱和，建议设为 OUTPUT_LIMIT 的 50%~80%
 *    OUTPUT_LIMIT   → 角速度环输出上限，对应 mixer 的差速分量
 *
 *  以上参数请在 car_config.h 中修改
 * ================================================================ */
#define GYRO_RATE_KP             CAR_GYRO_RATE_KP
#define GYRO_RATE_KI             CAR_GYRO_RATE_KI
#define GYRO_RATE_KD             CAR_GYRO_RATE_KD
#define GYRO_RATE_INTEGRAL_LIMIT CAR_GYRO_RATE_INTEGRAL_LIMIT
#define GYRO_RATE_OUTPUT_LIMIT   CAR_GYRO_RATE_OUTPUT_LIMIT

/**
 * @brief 初始化角速度环
 * @param target_dps 目标角速度 (°/s)，正=右转，负=左转
 */
void GyroRateLoop_Init(float target_dps);

/**
 * @brief 更新角速度环 PID
 * @param dt 控制周期 (秒)
 */
void GyroRateLoop_Update(float dt);

/**
 * @brief 获取角速度环修正量（用于 Mixer_SetGyroRateDiff）
 * @return 差速修正值，正=右转修正，负=左转修正
 */
int16_t GyroRateLoop_GetCorrection(void);

/**
 * @brief 修改目标角速度
 * @param target_dps 目标角速度 (°/s)
 */
void GyroRateLoop_SetTarget(float target_dps);

/**
 * @brief 获取当前实际角速度（陀螺仪 Z 轴读数）
 * @return 当前角速度 (°/s)
 */
float GyroRateLoop_GetCurrentRate(void);

/**
 * @brief 获取当前角速度误差
 * @return 角速度误差 (°/s)
 */
float GyroRateLoop_GetError(void);

/**
 * @brief 获取 PID 各分量（用于 VOFA+ 调试）
 */
float GyroRateLoop_GetPTerm(void);
float GyroRateLoop_GetITerm(void);
float GyroRateLoop_GetDTerm(void);

/**
 * @brief 复位 PID 状态
 */
void GyroRateLoop_Reset(void);

/**
 * @brief 设置 PID 参数（运行时调参）
 */
void GyroRateLoop_SetKP(float kp);
void GyroRateLoop_SetKI(float ki);
void GyroRateLoop_SetKD(float kd);

#endif /* _GYRO_RATE_CONTROL_H_ */