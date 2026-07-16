#include "ti_msp_dl_config.h"
#include "main.h"
#include "test.h"
#include "angle_pid.h"
#include "imu_ahrs.h"
#include "bsp_tb6612.h"
#include "board.h"
#include "clock.h"
#include <math.h>

/* ================================================================
 *  90°转弯测试参数
 * ================================================================ */
#define TURN90_KP           2.0f
#define TURN90_KD           5.0f
#define TURN90_LIMIT        800.0f
#define TURN90_TOLERANCE    1.0f
#define TURN90_BASE_SPEED   200
#define TURN90_TIMEOUT_MS   3000

/* ================================================================
 *  内部状态机
 * ================================================================ */
typedef enum {
    TEST_STATE_IDLE = 0,
    TEST_STATE_RUNNING,
    TEST_STATE_DONE,
    TEST_STATE_TIMEOUT,
    TEST_STATE_ERROR
} TestState_t;

/* ================================================================
 *  静态全局变量
 * ================================================================ */
static AnglePD_t   g_test_pd;
static TestState_t g_test_state = TEST_STATE_IDLE;
static float       g_test_target;
static float       g_test_current_error;
static float       g_test_current_output;
static uint32_t    g_test_start_time;
static uint32_t    g_test_timeout_ms;

/* ================================================================
 *  内部辅助函数
 * ================================================================ */
static float wrap_180(float v)
{
    if (v > 180.0f)       v -= 360.0f;
    else if (v < -180.0f) v += 360.0f;
    return v;
}

/* ================================================================
 *  Test_Turn90_Start —— 启动90°转弯（非阻塞）
 * ================================================================ */
int8_t Test_Turn90_Start(void)
{
    if (IMU_AHRS_GetStatus() != IMU_AHRS_STATUS_READY)
        return -2;

    float current = IMU_AHRS_Get_Yaw_Compensated();
    float target  = wrap_180(current + 90.0f);

    return Test_TurnToAngle_Start(target, TURN90_KP, TURN90_KD, TURN90_TIMEOUT_MS);
}

/* ================================================================
 *  Test_Turn90_Poll —— 轮询90°转弯状态
 * ================================================================ */
int8_t Test_Turn90_Poll(void)
{
    return Test_TurnToAngle_Poll();
}

/* ================================================================
 *  Test_TurnToAngle_Start —— 启动任意角度转弯（非阻塞）
 * ================================================================ */
int8_t Test_TurnToAngle_Start(float target_angle, float kp, float kd, uint32_t timeout_ms)
{
    if (IMU_AHRS_GetStatus() != IMU_AHRS_STATUS_READY)
        return -2;

    g_test_target = wrap_180(target_angle);

    AnglePD_Init(&g_test_pd, kp, kd, TURN90_LIMIT);
    AnglePD_Reset(&g_test_pd);

    g_test_current_error  = 0.0f;
    g_test_current_output = 0.0f;
    g_test_start_time     = tick_ms;
    g_test_timeout_ms     = timeout_ms;
    g_test_state          = TEST_STATE_RUNNING;

    float current = IMU_AHRS_Get_Yaw_Compensated();
    lc_printf("[Test] TurnToAngle start: %.1f -> %.1f\r\n", current, g_test_target);

    return 0;
}

/* ================================================================
 *  Test_TurnToAngle_Poll —— 轮询转弯状态（每帧在主循环中调用）
 *
 *  注意：调用前主循环应已执行 IMU_AHRS_Update_Data() + Update_Attitude()
 * ================================================================ */
int8_t Test_TurnToAngle_Poll(void)
{
    if (g_test_state != TEST_STATE_RUNNING)
    {
        if (g_test_state == TEST_STATE_DONE)    return 1;
        if (g_test_state == TEST_STATE_TIMEOUT) return -1;
        return -2; /* IDLE or ERROR */
    }

    float current_yaw = IMU_AHRS_Get_Yaw_Compensated();
    float output      = AnglePD_Update(&g_test_pd, g_test_target, current_yaw);

    g_test_current_output = output;
    g_test_current_error  = wrap_180(g_test_target - current_yaw);

    /* 电机控制 */
    int8_t sign = (output > 0.0f) ? 1 : -1;
    uint32_t speed = TURN90_BASE_SPEED + (uint32_t)(fabsf(output));
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

    /* 到达目标 */
    if (fabsf(g_test_current_error) < TURN90_TOLERANCE)
    {
        TB6612_Motor_Stop();
        g_test_state = TEST_STATE_DONE;
        lc_printf("[Test] TurnToAngle done! yaw=%.1f error=%.2f\r\n", current_yaw, g_test_current_error);
        return 1;
    }

    /* 超时 */
    if ((tick_ms - g_test_start_time) > g_test_timeout_ms)
    {
        TB6612_Motor_Stop();
        g_test_state = TEST_STATE_TIMEOUT;
        lc_printf("[Test] TurnToAngle timeout! yaw=%.1f target=%.1f\r\n", current_yaw, g_test_target);
        return -1;
    }

    return 0; /* 执行中 */
}

/* ================================================================
 *  状态查询接口（供 OLED 显示）
 * ================================================================ */
float Test_GetCurrentError(void)  { return g_test_current_error; }
float Test_GetCurrentOutput(void) { return g_test_current_output; }
float Test_GetTargetAngle(void)   { return g_test_target; }
uint32_t Test_GetElapsedMs(void)
{
    if (g_test_state != TEST_STATE_RUNNING) return 0;
    return tick_ms - g_test_start_time;
}