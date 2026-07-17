#include "turn.h"
#include "angle_pid.h"
#include "imu_ahrs.h"
#include "bsp_tb6612.h"
#include "board.h"
#include "clock.h"
#include <math.h>

/* ================================================================
 *  内部辅助：角度环绕 [-180, 180]
 * ================================================================ */
static float wrap_180(float v)
{
    if (v > 180.0f)       v -= 360.0f;
    else if (v < -180.0f) v += 360.0f;
    return v;
}

/* ================================================================
 *  转弯状态机
 * ================================================================ */
static AnglePD_t   g_turn_pd;
static TurnState_t g_turn_state = TURN_STATE_IDLE;
static float       g_turn_target;
static float       g_turn_error;
static float       g_turn_output;
static uint32_t    g_turn_start_ms;
static uint8_t     g_turn_stable_cnt;

/* ================================================================
 *  走直线状态机
 * ================================================================ */
static AnglePD_t   g_straight_pd;
static TurnState_t g_straight_state = TURN_STATE_IDLE;
static float       g_straight_target_yaw;
static uint32_t    g_straight_speed;
// 有互斥保护——转弯和走直线不能同时运行，Start 时会检查对方状态，返回 -3 表示冲突
/* ================================================================
 *  TurnToAngle_Start —— 转到绝对 Yaw 角度
 * ================================================================ */
int8_t TurnToAngle_Start(float target_angle)
{
    if (IMU_AHRS_GetStatus() != IMU_AHRS_STATUS_READY)
        return -2;

    if (g_straight_state == TURN_STATE_RUNNING)
        return -3;

    g_turn_target = wrap_180(target_angle);
    AnglePD_Init(&g_turn_pd, TURN_KP, TURN_KD, TURN_LIMIT);
    AnglePD_Reset(&g_turn_pd);

    g_turn_error      = 0.0f;
    g_turn_output     = 0.0f;
    g_turn_start_ms   = tick_ms;
    g_turn_stable_cnt = 0;
    g_turn_state      = TURN_STATE_RUNNING;

    float current = IMU_AHRS_Get_Yaw_Compensated();
    lc_printf("[Turn] TurnToAngle start: %.1f -> %.1f\r\n", current, g_turn_target);
    return 0;
}

/* ================================================================
 *  TurnByAngle_Start —— 转相对角度
 * ================================================================ */
int8_t TurnByAngle_Start(float delta_angle)
{
    if (IMU_AHRS_GetStatus() != IMU_AHRS_STATUS_READY)
        return -2;

    float current = IMU_AHRS_Get_Yaw_Compensated();
    float target  = wrap_180(current + delta_angle);
    return TurnToAngle_Start(target);
}

/* ================================================================
 *  Turn_Poll —— 每帧在主循环中调用
 *
 *  关键修复：自适应 BASE_SPEED
 *    误差大（> NEAR_ZONE）→ 用 BASE_SPEED 保证快速响应
 *    误差小（< NEAR_ZONE）→ 线性缩放至 MIN_SPEED，不会归零
 *    稳定检测（连续 N 帧误差在容差内）→ 判定到达
 * ================================================================ */
int8_t Turn_Poll(void)
{
    if (g_turn_state != TURN_STATE_RUNNING)
    {
        if (g_turn_state == TURN_STATE_DONE)    return 1;
        if (g_turn_state == TURN_STATE_TIMEOUT) return -1;
        return -2;
    }

    float current_yaw = IMU_AHRS_Get_Yaw_Compensated();
    float output      = AnglePD_Update(&g_turn_pd, g_turn_target, current_yaw);

    g_turn_output = output;
    g_turn_error  = wrap_180(g_turn_target - current_yaw);

    float error_abs = fabsf(g_turn_error);

    /*
     *  稳定检测：连续 N 帧误差在容差内才判定到达
     *  已改为用帧计数代替 error_delta，避免 KD 震荡导致死锁
     */
    #define TURN_STABLE_FRAMES  5
    if (error_abs < TURN_TOLERANCE)
    {
        g_turn_stable_cnt++;
        if (g_turn_stable_cnt >= TURN_STABLE_FRAMES)
        {
            TB6612_Motor_Stop();
            g_turn_state = TURN_STATE_DONE;
            lc_printf("[Turn] done! yaw=%.1f error=%.2f\r\n", current_yaw, g_turn_error);
            return 1;
        }
    }
    else
    {
        g_turn_stable_cnt = 0;
    }

    /*
     *  自适应 BASE_SPEED：
     *  误差 > NEAR_ZONE  → 全速 BASE_SPEED
     *  误差 < NEAR_ZONE  → 线性缩放至 MIN_SPEED
     */
    uint32_t base;
    if (error_abs > TURN_NEAR_ZONE)
    {
        base = TURN_BASE_SPEED;
    }
    else
    {
        base = TURN_MIN_SPEED +
               (uint32_t)((TURN_BASE_SPEED - TURN_MIN_SPEED) * error_abs / TURN_NEAR_ZONE);
    }

    int8_t   sign  = (output > 0.0f) ? -1 : 1;
    uint32_t speed = base + (uint32_t)(fabsf(output));
    if (speed > 999) speed = 999;

    if (sign > 0)
    {
        AO_Control(1, speed);
        BO_Control(0, speed);
    }
    else
    {       
        AO_Control(0, speed);
        BO_Control(1, speed);
    }

    /* 诊断日志：打印误差和计数器，确认是否被意外重置 */
    lc_printf("[Turn] err=%.2f cnt=%u spd=%lu\r\n",
              g_turn_error, g_turn_stable_cnt, speed);

    if ((tick_ms - g_turn_start_ms) > TURN_TIMEOUT_MS)
    {
        TB6612_Motor_Stop();
        g_turn_state = TURN_STATE_TIMEOUT;
        lc_printf("[Turn] timeout! yaw=%.1f target=%.1f\r\n", current_yaw, g_turn_target);
        return -1;
    }

    return 0;
}

/* ================================================================
 *  Turn_Stop —— 手动停止转弯
 * ================================================================ */
void Turn_Stop(void)
{
    TB6612_Motor_Stop();
    g_turn_state = TURN_STATE_IDLE;
    lc_printf("[Turn] stopped manually\r\n");
}

/* ================================================================
 *  GoStraight_Start —— 走直线（带航向保持）
 * ================================================================ */
int8_t GoStraight_Start(uint32_t speed)
{
    if (IMU_AHRS_GetStatus() != IMU_AHRS_STATUS_READY)
        return -2;

    if (g_turn_state == TURN_STATE_RUNNING)
        return -3;

    g_straight_target_yaw = IMU_AHRS_Get_Yaw_Compensated();
    g_straight_speed      = speed;

    AnglePD_Init(&g_straight_pd, STRAIGHT_KP, STRAIGHT_KD, STRAIGHT_LIMIT);
    AnglePD_Reset(&g_straight_pd);

    g_straight_state = TURN_STATE_RUNNING;
    lc_printf("[Turn] GoStraight start: yaw=%.1f speed=%lu\r\n", g_straight_target_yaw, speed);
    return 0;
}

/* ================================================================
 *  GoStraight_Poll —— 每帧调用，维持航向
 * ================================================================ */
int8_t GoStraight_Poll(void)
{
    if (g_straight_state != TURN_STATE_RUNNING)
        return -2;

    float current_yaw = IMU_AHRS_Get_Yaw_Compensated();
    float output      = AnglePD_Update(&g_straight_pd, g_straight_target_yaw, current_yaw);

    int32_t base      = (int32_t)g_straight_speed;
    int32_t left_spd  = base + (int32_t)output;
    int32_t right_spd = base - (int32_t)output;

    if (left_spd  < 0)   left_spd  = 0;
    if (left_spd  > 999) left_spd  = 999;
    if (right_spd < 0)   right_spd = 0;
    if (right_spd > 999) right_spd = 999;

    AO_Control(1, (uint32_t)left_spd);
    BO_Control(1, (uint32_t)right_spd);

    return 0;
}

/* ================================================================
 *  GoStraight_Stop —— 停止走直线
 * ================================================================ */
void GoStraight_Stop(void)
{
    TB6612_Motor_Stop();
    g_straight_state = TURN_STATE_IDLE;
    lc_printf("[Turn] GoStraight stopped\r\n");
}

/* ================================================================
 *  状态查询接口
 * ================================================================ */
TurnState_t Turn_GetState(void)        { return g_turn_state; }
float       Turn_GetCurrentError(void)  { return g_turn_error; }
float       Turn_GetCurrentOutput(void) { return g_turn_output; }
uint8_t     Turn_GetStableCount(void)   { return g_turn_stable_cnt; }