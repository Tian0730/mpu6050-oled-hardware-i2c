#include "control_mixer.h"
#include "bsp_tb6612.h"

static uint32_t g_mix_base = 300;
static int16_t  g_angle_diff = 0;
static int16_t  g_follow_diff = 0;
static int16_t  g_speed_diff = 0;    // 预留速度环

void Mixer_Init(uint32_t base_speed)
{
    g_mix_base = base_speed;
}

void Mixer_SetAngleDiff(int16_t diff)
{
    g_angle_diff = diff;
}

void Mixer_SetFollowDiff(int16_t diff)
{
    g_follow_diff = diff;
}

void Mixer_SetSpeedDiff(int16_t diff)    // 预留速度环
{
    g_speed_diff = diff;
}

void Mixer_Apply(void)
{
    int32_t total = (int32_t)g_angle_diff + (int32_t)g_follow_diff;
    int32_t left  = (int32_t)g_mix_base + total;
    int32_t right = (int32_t)g_mix_base - total;

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
}