#include "ti_msp_dl_config.h"
#include "main.h"
#include "gyro_rate_control.h"
#include "speed_pid.h"

static SpeedPID_t g_gyro_rate_pid;
static float      g_gyro_rate_target = 0.0f;

void GyroRateLoop_Init(float target_dps)
{
    g_gyro_rate_target = target_dps;
    SpeedPID_Init(&g_gyro_rate_pid,
                  GYRO_RATE_KP,
                  GYRO_RATE_KI,
                  GYRO_RATE_KD,
                  GYRO_RATE_INTEGRAL_LIMIT,
                  GYRO_RATE_OUTPUT_LIMIT);
}

void GyroRateLoop_Update(float dt)
{
    const IMU_AHRS_Data_t *imu = IMU_AHRS_GetData();
    float current_rate = imu->gyro_deg[2];  /* Z 轴角速度 (°/s) */
    SpeedPID_Update(&g_gyro_rate_pid, g_gyro_rate_target, current_rate, dt);
}

int16_t GyroRateLoop_GetCorrection(void)
{
    return (int16_t)g_gyro_rate_pid.output;
}

void GyroRateLoop_SetTarget(float target_dps)
{
    g_gyro_rate_target = target_dps;
}

float GyroRateLoop_GetCurrentRate(void)
{
    const IMU_AHRS_Data_t *imu = IMU_AHRS_GetData();
    return imu->gyro_deg[2];
}

float GyroRateLoop_GetError(void)
{
    return g_gyro_rate_pid.current_error;
}

float GyroRateLoop_GetPTerm(void)
{
    return g_gyro_rate_pid.p_term;
}

float GyroRateLoop_GetITerm(void)
{
    return g_gyro_rate_pid.i_term;
}

float GyroRateLoop_GetDTerm(void)
{
    return g_gyro_rate_pid.d_term;
}

void GyroRateLoop_Reset(void)
{
    SpeedPID_Reset(&g_gyro_rate_pid);
}

void GyroRateLoop_SetKP(float kp)
{
    g_gyro_rate_pid.kp = kp;
}

void GyroRateLoop_SetKI(float ki)
{
    g_gyro_rate_pid.ki = ki;
}

void GyroRateLoop_SetKD(float kd)
{
    g_gyro_rate_pid.kd = kd;
}