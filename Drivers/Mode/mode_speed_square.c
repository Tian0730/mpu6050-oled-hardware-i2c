#include "ti_msp_dl_config.h"
#include "main.h"
#include "button_mode.h"
#include "gyro_rate_control.h"
#include "mode_speed_square.h"

/* ================================================================
 *  纯速度环模式：恒定目标速度直行
 *  仅用于调试速度环 PID 参数
 * ================================================================ */
void ModeSpeedLoop_Run(float dt, uint32_t now)
{
    (void)dt;

    Mixer_SetSpeedDiff(SpeedLoop_GetCorrection());
    Mixer_SetAngleDiff(0);
    Mixer_SetFollowDiff(0);
#if USE_GYRO_RATE_CONTROL
    Mixer_SetGyroRateDiff(GyroRateLoop_GetCorrection());
#endif
    Mixer_Apply();

    static uint32_t spd_oled_last = 0;
    if ((int32_t)(now - spd_oled_last) >= 200)
    {
        uint8_t oled_buffer[32];
        spd_oled_last = now;
        sprintf((char *)oled_buffer, "S:%.0f T:%.0f",
                SpeedLoop_GetCurrentSpeed(),
                (float)200);
        OLED_ShowString(0, 6, oled_buffer, 8);
        sprintf((char *)oled_buffer, "E:%.1f O:%.0f",
                SpeedLoop_GetError(),
                (float)SpeedLoop_GetCorrection());
        OLED_ShowString(0, 7, oled_buffer, 8);
    }
}

/* ================================================================
 *  正方形赛道巡线状态机
 *
 *  黑线区域：位置环 + 角度环 + 速度环（三环全开）
 *  空白区域：角度环 + 速度环（航向保持 + 恒速）
 *  弯道检测：f1右转 / f8左转 → 延时预转 → IMU转90°
 *
 *  可调参数：
 *    PRE_TURN_MS   - 预转延时(ms)，车子中心走到弯道中心所需时间
 *    TURN_COOLDOWN - 转弯后冷却(ms)，防止同一个弯重复触发
 * ================================================================ */
void ModeSquareTrack_Run(float dt, uint32_t now)
{
    #define PRE_TURN_MS    CAR_PRE_TURN_MS
    #define TURN_COOLDOWN  CAR_TURN_COOLDOWN

    static enum { ST_STRAIGHT, ST_PRE_TURN, ST_TURNING } state = ST_STRAIGHT;
    static uint8_t was_on_black = 0;
    static uint32_t pre_turn_deadline = 0;
    static float    pre_turn_angle = 0.0f;
    static uint32_t turn_cooldown_deadline = 0;

    if (state == ST_PRE_TURN)
    {
        /* 预转期间：速度环 + 角度环，确保直线走到弯道中心 */
#if USE_ANGLE_CONTROL
        int16_t angle_corr = GoStraight_GetCorrection();
        Mixer_SetAngleDiff(-angle_corr);
#else
        Mixer_SetAngleDiff(0);
#endif
        Mixer_SetSpeedDiff(SpeedLoop_GetCorrection());
        Mixer_SetFollowDiff(0);
#if USE_GYRO_RATE_CONTROL
        Mixer_SetGyroRateDiff(GyroRateLoop_GetCorrection());
#endif
        Mixer_Apply();

        if ((int32_t)(tick_ms - pre_turn_deadline) >= 0)
        {
#if USE_ANGLE_CONTROL
            GoStraight_Stop();
#endif
            TurnByAngle_Start(pre_turn_angle);
            state = ST_TURNING;
        }
    }
    else if (state == ST_TURNING)
    {
        uint8_t oled_buffer[32];
        was_on_black = 0;
        int8_t tr = Turn_Poll();
        if (tr == 1 || tr == -1)
        {
#if USE_ANGLE_CONTROL
            GoStraight_StartAt(Turn_GetTarget(), 300);
#endif
            state = ST_STRAIGHT;
            turn_cooldown_deadline = tick_ms + TURN_COOLDOWN;
        }
        sprintf((char *)oled_buffer, "TURN E:%.1f", Turn_GetCurrentError());
        OLED_ShowString(0, 6, oled_buffer, 16);
    }
    else
    {
        uint8_t oled_buffer[32];
        IRDM_read_sensors();

        uint8_t cooldown = ((int32_t)(tick_ms - turn_cooldown_deadline) < 0);

        if (!cooldown && IRDM_NeedTurnLeftFast())
        {
            was_on_black = 0;
            pre_turn_deadline = tick_ms + PRE_TURN_MS;
            pre_turn_angle = -90.0f;
            state = ST_PRE_TURN;
        }
        else if (!cooldown && IRDM_NeedTurnRightFast())
        {
            was_on_black = 0;
            pre_turn_deadline = tick_ms + PRE_TURN_MS;
            pre_turn_angle = 90.0f;
            state = ST_PRE_TURN;
        }
        else if (IRDM_IsBlackLine())
        {
            was_on_black = 1;
#if USE_FOLLOW_CONTROL
            IRDM_UpdatePositionPID(dt);
            int16_t follow_corr = IRDM_GetCorrection();
#else
            int16_t follow_corr = 0;
#endif

            /* 位置偏差大时，抑制角度环，优先回到线上 */
#if USE_ANGLE_CONTROL
            int16_t follow_abs = (follow_corr > 0) ? follow_corr : -follow_corr;
            int16_t angle_corr = 0;
            if (follow_abs <= 80)
            {
                angle_corr = GoStraight_GetCorrection();
                if (follow_abs > 40)
                {
                    angle_corr = (int16_t)((int32_t)angle_corr * (80 - follow_abs) / 40);
                }
            }
#else
            int16_t angle_corr = 0;
#endif

            /* 按配置开关决定几环参与 */
            Mixer_SetFollowDiff(follow_corr);
            Mixer_SetAngleDiff(-angle_corr);
            Mixer_SetSpeedDiff(SpeedLoop_GetCorrection());
#if USE_GYRO_RATE_CONTROL
            Mixer_SetGyroRateDiff(GyroRateLoop_GetCorrection());
#endif
            Mixer_Apply();
        }
        else
        {
            /* 空白区域：按配置开关决定环参与 */
            was_on_black = 0;
#if USE_ANGLE_CONTROL
            GoStraight_Poll();
            int16_t angle_corr = GoStraight_GetCorrection();
#else
            int16_t angle_corr = 0;
#endif
            Mixer_SetSpeedDiff(SpeedLoop_GetCorrection());
            Mixer_SetAngleDiff(-angle_corr);
            Mixer_SetFollowDiff(0);
#if USE_GYRO_RATE_CONTROL
            Mixer_SetGyroRateDiff(GyroRateLoop_GetCorrection());
#endif
            Mixer_Apply();

            sprintf((char *)oled_buffer, "WHITE");
            OLED_ShowString(0, 6, oled_buffer, 16);
        }
    }
}