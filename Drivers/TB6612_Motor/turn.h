#ifndef _TURN_H_
#define _TURN_H_

#include <stdint.h>

/* ================================================================
 *  转弯 PID 参数（宏定义，方便调试）
 * ================================================================ */
#define TURN_KP             2.0f
#define TURN_KD             5.0f
#define TURN_LIMIT          800.0f
#define TURN_TOLERANCE      1.0f
#define TURN_BASE_SPEED     200
#define TURN_TIMEOUT_MS     3000

/* ================================================================
 *  走直线 PID 参数（宏定义，方便调试）
 * ================================================================ */
#define STRAIGHT_KP         2.0f
#define STRAIGHT_KD         3.0f
#define STRAIGHT_LIMIT      250.0f
#define STRAIGHT_SPEED      300

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

/* ================================================================
 *  走直线 API（非阻塞，带航向保持）
 * ================================================================ */
int8_t GoStraight_Start(uint32_t speed);        /* 启动走直线，保持当前航向 */
int8_t GoStraight_Poll(void);                   /* 轮询：0=执行中 -2=空闲 */
void   GoStraight_Stop(void);                   /* 停止 */

/* ================================================================
 *  状态查询接口
 * ================================================================ */
TurnState_t Turn_GetState(void);
float       Turn_GetCurrentError(void);
float       Turn_GetCurrentOutput(void);

#endif