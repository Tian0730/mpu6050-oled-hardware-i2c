#include "ti_msp_dl_config.h"
#include "main.h"
#include "gyro_rate_test.h"
#include "speed_pid.h"

/*
 *  角速度环 180 右转测试
 *
 *  目标：只用角速度环，两个轮子均前行（差速转向），右转 180
 *
 *  控制链路：
 *    陀螺仪 Z 轴  actual_rate (/s)
 *    error = GYRO_TEST_TARGET_DPS - actual_rate
 *    角速度 PID  Turn
 *    V_left  = GYRO_TEST_BASE_SPEED - Turn
 *    V_right = GYRO_TEST_BASE_SPEED + Turn
 *
 *  OLED 显示：
 *    W:目标/s  A:实际/s
 *    ANG:已转角度  T:差速
 *
 *  调参：所有参数在 car_config.h 中修改
 *    GYRO_TEST_TARGET_DPS  目标角速度，越大转得越快
 *    GYRO_TEST_BASE_SPEED  基础速度，越高惯性越大，需配合增大 KP
 *    CAR_GYRO_RATE_KP/I/D  角速度环 PID 参数
 */

void GyroRateTest_Run(float dt, uint32_t now)
{
    static SpeedPID_t g_test_pid;
    static uint8_t    g_test_inited = 0;
    static float      g_total_angle = 0.0f;
    static uint32_t   g_test_start  = 0;
    static uint8_t    g_test_done   = 0;

    if (!g_test_inited)
    {
        SpeedPID_Init(&g_test_pid,
                      GYRO_RATE_KP,
                      GYRO_RATE_KI,
                      GYRO_RATE_KD,
                      GYRO_RATE_INTEGRAL_LIMIT,
                      GYRO_RATE_OUTPUT_LIMIT);
        g_total_angle = 0.0f;
        g_test_start  = now;
        g_test_done   = 0;
        g_test_inited = 1;
    }

    if (g_test_done)
    {
        TB6612_Motor_Stop();
        return;
    }

    if ((int32_t)(now - g_test_start) >= GYRO_TEST_TIMEOUT_MS)
    {
        TB6612_Motor_Stop();
        g_test_done = 1;
        return;
    }

    const IMU_AHRS_Data_t *imu = IMU_AHRS_GetData();
    float actual_rate = imu->gyro_deg[2];

    if (dt > 0.0001f && dt < 0.5f)
        g_total_angle += actual_rate * dt;

    if (g_total_angle >= GYRO_TEST_TOTAL_ANGLE)
    {
        TB6612_Motor_Stop();
        g_test_done = 1;
        return;
    }

    float inner_output = SpeedPID_Update(&g_test_pid,
                                         GYRO_TEST_TARGET_DPS,
                                         actual_rate, dt);
    int16_t turn = (int16_t)inner_output;

    Mixer_SetBaseSpeed(GYRO_TEST_BASE_SPEED);
    Mixer_SetFollowDiff(0);
    Mixer_SetAngleDiff(0);
    Mixer_SetGyroRateDiff(turn);
    Mixer_SetSpeedDiff(0);
    Mixer_SetDiffScale(1.0f);
    Mixer_SetOuterBoost(0.0f);
    Mixer_Apply();

    {
        uint8_t oled_buffer[32];
        sprintf((char *)oled_buffer, "W:%4.0f A:%4.0f",
                (double)GYRO_TEST_TARGET_DPS, (double)actual_rate);
        OLED_ShowString(0, 6, oled_buffer, 8);
        sprintf((char *)oled_buffer, "ANG:%3.0f T:%4d",
                (double)g_total_angle, turn);
        OLED_ShowString(0, 7, oled_buffer, 8);
    }
}