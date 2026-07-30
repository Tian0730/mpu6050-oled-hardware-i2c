#include "control_mixer.h"
#include "bsp_tb6612.h"

static uint32_t g_mix_base = 300;
static int16_t  g_angle_diff = 0;
static int16_t  g_follow_diff = 0;
static int16_t  g_speed_diff = 0;
static int16_t  g_gyro_rate_diff = 0;

static float  g_outer_boost = 0.0f;
static float  g_diff_scale  = 1.0f;

void Mixer_Init(uint32_t base_speed)
{
    g_mix_base = base_speed;
    g_outer_boost = 0.0f;
    g_diff_scale  = 1.0f;
}

void Mixer_SetBaseSpeed(uint32_t base_speed)
{
    g_mix_base = base_speed;
}

void Mixer_SetOuterBoost(float ratio)
{
    g_outer_boost = ratio;
}

void Mixer_SetDiffScale(float scale)
{
    g_diff_scale = scale;
}

void Mixer_SetAngleDiff(int16_t diff)
{
    g_angle_diff = diff;
}

void Mixer_SetFollowDiff(int16_t diff)
{
    g_follow_diff = diff;
}

void Mixer_SetSpeedDiff(int16_t diff)
{
    g_speed_diff = diff;
}

void Mixer_SetGyroRateDiff(int16_t diff)
{
    g_gyro_rate_diff = diff;
}

void Mixer_Apply(void)
{
    int32_t total = (int32_t)g_angle_diff + (int32_t)g_follow_diff + (int32_t)g_gyro_rate_diff;
    total = (int32_t)((float)total * g_diff_scale);
    int32_t base  = (int32_t)g_mix_base + (int32_t)g_speed_diff;
    int32_t left  = base + total;
    int32_t right = base - total;

    if (g_outer_boost > 0.001f) {
        int32_t abs_total = (total > 0) ? total : -total;
        int32_t boost = (int32_t)((float)abs_total * g_outer_boost);
        if (total > 0) {
            left += boost;
        } else {
            right += boost;
        }
    }

    if (left < 0)   left = 0;
    if (left > 999) left = 999;
    if (right < 0)  right = 0;
    if (right > 999) right = 999;

    AO_Control(1, (uint32_t)left);
    BO_Control(1, (uint32_t)right);
}

void Mixer_Reset(void)
{
    g_angle_diff = 0;
    g_follow_diff = 0;
    g_speed_diff  = 0;
    g_gyro_rate_diff = 0;
}