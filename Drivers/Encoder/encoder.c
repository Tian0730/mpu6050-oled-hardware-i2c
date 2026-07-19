#include "encoder.h"
#include "bsp_motor_hallencoder.h"

// 速度数据
static WheelSpeed wheel_speed = {0};

// 滤波后的速度（用于对外输出）
static float filtered_left_speed_mmps  = 0.0f;
static float filtered_right_speed_mmps = 0.0f;

// 脉冲累积（降采样用）
static int32_t accum_left_pulses  = 0;
static int32_t accum_right_pulses = 0;
static int32_t decim_count = 0;

/**
 * @brief 更新车轮速度数据（累积式降采样）
 * @details 每次定时器中断(5ms)累积脉冲，每4次(20ms)计算一次速度
 * @return 1=有新速度数据, 0=数据未更新
 */
uint8_t IRDM_update_wheel_speed(void)
{
    if (!g_encoder_new_data) return 0;
    g_encoder_new_data = 0;

    /* 累积本周期脉冲 */
    accum_left_pulses  += Motor_Get_Encoder(0);
    accum_right_pulses += Motor_Get_Encoder(1);
    decim_count++;

    if (decim_count < ENCODER_DECIMATION) return 0;

    /* 达到降采样次数，计算速度 */
    wheel_speed.left_pulses  = accum_left_pulses;
    wheel_speed.right_pulses = accum_right_pulses;

    float sample_period_sec = (ENCODER_SAMPLE_PERIOD_MS * ENCODER_DECIMATION) / 1000.0f;
    float pulses_per_rev = ENCODER_RESOLUTION * MOTOR_GEAR_RATIO;

    wheel_speed.left_speed_rpm  = (accum_left_pulses  / pulses_per_rev) * (60.0f / sample_period_sec);
    wheel_speed.right_speed_rpm = (accum_right_pulses / pulses_per_rev) * (60.0f / sample_period_sec);

    float wheel_circumference = 3.1415926f * WHEEL_DIAMETER_MM;
    wheel_speed.left_speed_mmps  = wheel_speed.left_speed_rpm  * wheel_circumference / 60.0f;
    wheel_speed.right_speed_mmps = wheel_speed.right_speed_rpm * wheel_circumference / 60.0f;

    /* 一阶低通滤波 */
    float a = ENCODER_SPEED_FILTER_ALPHA;
    filtered_left_speed_mmps  = filtered_left_speed_mmps  * (1.0f - a) + wheel_speed.left_speed_mmps  * a;
    filtered_right_speed_mmps = filtered_right_speed_mmps * (1.0f - a) + wheel_speed.right_speed_mmps * a;

    /* 重置累积 */
    accum_left_pulses  = 0;
    accum_right_pulses = 0;
    decim_count = 0;

    return 1;
}

/**
 * @brief 获取左轮转速
 * @return 左轮转速 (RPM)
 */
float IRDM_get_left_speed_rpm(void)
{
    return wheel_speed.left_speed_rpm;
}

/**
 * @brief 获取右轮转速
 * @return 右轮转速 (RPM)
 */
float IRDM_get_right_speed_rpm(void)
{
    return wheel_speed.right_speed_rpm;
}

/**
 * @brief 获取左轮线速度
 * @return 左轮线速度 (mm/s)
 */
float IRDM_get_left_speed_mmps(void)
{
    return filtered_left_speed_mmps;
}

/**
 * @brief 获取右轮线速度
 * @return 右轮线速度 (mm/s)
 */
float IRDM_get_right_speed_mmps(void)
{
    return filtered_right_speed_mmps;
}

/**
 * @brief 获取左右轮速度差
 * @return 速度差 (mm/s)，正值表示左轮比右轮快
 */
float IRDM_get_speed_diff(void)
{
    return filtered_left_speed_mmps - filtered_right_speed_mmps;
}

/**
 * @brief 获取左右轮平均速度
 * @return 平均速度 (mm/s)
 */
float IRDM_get_average_speed(void)
{
    return (filtered_left_speed_mmps + filtered_right_speed_mmps) / 2.0f;
}

/**
 * @brief 获取编码器脉冲计数
 * @param dir 0-左轮, 1-右轮
 * @return 脉冲计数值
 */
int32_t IRDM_get_encoder_pulses(int dir)
{
    if (dir == 0) {
        return wheel_speed.left_pulses;
    } else {
        return wheel_speed.right_pulses;
    }
}