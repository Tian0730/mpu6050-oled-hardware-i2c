#include "speed_control.h"
#include "speed_pid.h"
#include "encoder.h"
#include "board.h"

static SpeedPID_t g_speed_pid;
static float      g_target_speed  = 0.0f;
static float      g_current_error = 0.0f;

/******************************************************************
 * 函 数 名 称：SpeedLoop_Init
 * 函 数 说 明：速度环初始化
 * 函 数 形 参：target_speed_mmps-目标线速度(mm/s)
 * 函 数 返 回：无
 * 作       者：tian
 * 备       注：无
******************************************************************/
void SpeedLoop_Init(float target_speed_mmps)
{
    g_target_speed = target_speed_mmps;
    SpeedPID_Init(&g_speed_pid,
                  SPEED_KP, SPEED_KI, SPEED_KD,
                  SPEED_INTEGRAL_LIMIT, SPEED_OUTPUT_LIMIT);
    SpeedPID_Reset(&g_speed_pid);
    lc_printf("[SpeedLoop] Init: target=%.1f mm/s\r\n", target_speed_mmps);
}

/******************************************************************
 * 函 数 名 称：SpeedLoop_Update
 * 函 数 说 明：更新速度环，仅编码器有新数据时才运行PID
 * 函 数 形 参：dt-距离上次主循环的时间间隔(秒)
 * 函 数 返 回：无
 * 作       者：tian
 * 备       注：编码器20ms更新一次，PID dt 会累积到匹配
******************************************************************/
void SpeedLoop_Update(float dt)
{
    static float accum_dt = 0.0f;
    accum_dt += dt;

    if (IRDM_update_wheel_speed())
    {
        float current_speed = IRDM_get_average_speed();
        SpeedPID_Update(&g_speed_pid, g_target_speed, current_speed, accum_dt);
        g_current_error = g_target_speed - current_speed;
        accum_dt = 0.0f;
    }
}

/******************************************************************
 * 函 数 名 称：SpeedLoop_GetCorrection
 * 函 数 说 明：获取速度环 PID 修正量
 * 函 数 形 参：无
 * 函 数 返 回：速度修正值，正值=加速，负值=减速
 * 作       者：tian
 * 备       注：此值可直接传给 Mixer_SetSpeedDiff()
******************************************************************/
int16_t SpeedLoop_GetCorrection(void)
{
    return (int16_t)g_speed_pid.output;
}

/******************************************************************
 * 函 数 名 称：SpeedLoop_SetTarget
 * 函 数 说 明：修改目标速度
 * 函 数 形 参：target_speed_mmps-新的目标线速度(mm/s)
 * 函 数 返 回：无
 * 作       者：tian
 * 备       注：切换目标时自动重置积分，避免积分饱和
******************************************************************/
void SpeedLoop_SetTarget(float target_speed_mmps)
{
    g_target_speed = target_speed_mmps;
    SpeedPID_Reset(&g_speed_pid);
    lc_printf("[SpeedLoop] Target changed: %.1f mm/s\r\n", target_speed_mmps);
}

/******************************************************************
 * 函 数 名 称：SpeedLoop_GetCurrentSpeed
 * 函 数 说 明：获取当前实际平均速度
 * 函 数 形 参：无
 * 函 数 返 回：当前平均线速度(mm/s)
 * 作       者：tian
 * 备       注：无
******************************************************************/
float SpeedLoop_GetCurrentSpeed(void)
{
    return IRDM_get_average_speed();
}

/******************************************************************
 * 函 数 名 称：SpeedLoop_GetError
 * 函 数 说 明：获取当前速度误差
 * 函 数 形 参：无
 * 函 数 返 回：速度误差(mm/s)，正值=实际比目标慢
 * 作       者：tian
 * 备       注：无
******************************************************************/
float SpeedLoop_GetError(void)
{
    return g_current_error;
}

/******************************************************************
 * 函 数 名 称：SpeedLoop_Reset
 * 函 数 说 明：重置速度环
 * 函 数 形 参：无
 * 函 数 返 回：无
 * 作       者：tian
 * 备       注：清除积分和误差历史
******************************************************************/
void SpeedLoop_Reset(void)
{
    SpeedPID_Reset(&g_speed_pid);
    g_current_error = 0.0f;
    lc_printf("[SpeedLoop] Reset\r\n");
}

/// === VOFA+ 新增 getter/setter ===
float SpeedLoop_GetTarget(void) { return g_target_speed; }
float SpeedLoop_GetPTerm(void)  { return SpeedPID_GetPTerm(&g_speed_pid); }
float SpeedLoop_GetITerm(void)  { return SpeedPID_GetITerm(&g_speed_pid); }
float SpeedLoop_GetDTerm(void)  { return SpeedPID_GetDTerm(&g_speed_pid); }
void  SpeedLoop_SetKP(float kp) { SpeedPID_SetKP(&g_speed_pid, kp); lc_printf("[VOFA] Speed KP=%.2f\r\n", kp); }
void  SpeedLoop_SetKI(float ki) { SpeedPID_SetKI(&g_speed_pid, ki); lc_printf("[VOFA] Speed KI=%.3f\r\n", ki); }
void  SpeedLoop_SetKD(float kd) { SpeedPID_SetKD(&g_speed_pid, kd); lc_printf("[VOFA] Speed KD=%.2f\r\n", kd); }