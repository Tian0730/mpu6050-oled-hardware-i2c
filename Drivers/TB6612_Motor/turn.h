#ifndef _TURN_H_
#define _TURN_H_

#include <stdint.h>
#include "car_config.h"

/* ================================================================
 *  转弯角度环 PID 参数
 *
 *  调参要点：
 *    KP 太大 → 过冲震荡；KP 太小 → 转不动
 *    KD 太大 → 响应迟钝；KD 太小 → 震荡不止
 *    BASE_SPEED 太大 → 接近目标时冲过头
 *    MIN_SPEED  → 接近目标时的最低速度（克服静摩擦即可）
 *    NEAR_ZONE  → 误差小于此值后自动切换 MIN_SPEED
 *
 *  以上参数请在 car_config.h 中修改
 * ================================================================ */
#define TURN_KP             CAR_TURN_KP
#define TURN_KD             CAR_TURN_KD
#define TURN_LIMIT          CAR_TURN_LIMIT
#define TURN_TOLERANCE      CAR_TURN_TOLERANCE
#define TURN_BASE_SPEED     CAR_TURN_BASE_SPEED
#define TURN_MIN_SPEED      CAR_TURN_MIN_SPEED
#define TURN_NEAR_ZONE      CAR_TURN_NEAR_ZONE
#define TURN_TIMEOUT_MS     CAR_TURN_TIMEOUT_MS

/* ================================================================
 *  直线角度环 PID 参数
 *
 *  以上参数请在 car_config.h 中修改
 * ================================================================ */
#define STRAIGHT_KP         CAR_STRAIGHT_KP
#define STRAIGHT_KD         CAR_STRAIGHT_KD
#define STRAIGHT_LIMIT      CAR_STRAIGHT_LIMIT
#define STRAIGHT_SPEED      CAR_STRAIGHT_SPEED

/* ================================================================
 *  状态枚举
 * ================================================================ */
typedef enum {
    TURN_STATE_IDLE = 0,
    TURN_STATE_RUNNING,
    TURN_STATE_DONE,
    TURN_STATE_TIMEOUT
} TurnState_t;

/* ================================================================
 *  转弯 API（非阻塞，只传角度）
 * ================================================================ */
int8_t TurnToAngle_Start(float target_angle);   /* 转到绝对 Yaw 角度 */
int8_t TurnByAngle_Start(float delta_angle);    /* 转相对角度（+右/-左） */
int8_t Turn_Poll(void);                         /* 轮询：0=执行中 1=完成 -1=超时 -2=空闲 */
void   Turn_Stop(void);                         /* 手动停止 */
float  Turn_GetTarget(void);

/* ================================================================
 *  走直线 API（非阻塞，带航向保持）
 * ================================================================ */
int8_t GoStraight_Start(uint32_t speed);        /* 启动走直线，保持当前航向 */
int8_t GoStraight_StartAt(float target_yaw, uint32_t speed); /* 指定目标航向走直线 */
int8_t GoStraight_Poll(void);                   /* 轮询：0=执行中 -2=空闲 */
void   GoStraight_Stop(void);                   /* 停止 */
void   GoStraight_ResetPD(void);                /* 复位角度 PD（不清目标航向） */

/* ================================================================
 *  正方形行走 API（非阻塞）
 * ================================================================ */
int8_t  Square_Start(uint8_t legs, float turn_angle, uint32_t straight_ms);
int8_t  Square_Poll(void);       /* 0=执行中 1=完成 -1=超时 */
uint8_t Square_GetState(void);   /* 0=转弯 1=直行 2=完成 */
uint8_t Square_GetLeg(void);     /* 当前第几条边 (0-based) */

/* ================================================================
 *  状态查询接口
 * ================================================================ */
TurnState_t Turn_GetState(void);
float       Turn_GetCurrentError(void);
float       Turn_GetCurrentOutput(void);
uint8_t     Turn_GetStableCount(void);
int16_t     Turn_GetStraightCorrection(void);
int16_t     GoStraight_GetCorrection(void);

float GoStraight_GetPTerm(void);
float GoStraight_GetDTerm(void);
float GoStraight_GetError(void);
float GoStraight_GetOutput(void);
void  GoStraight_SetKP(float kp);
void  GoStraight_SetKD(float kd);

#endif