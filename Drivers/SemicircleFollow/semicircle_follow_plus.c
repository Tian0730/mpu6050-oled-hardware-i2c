#include "ti_msp_dl_config.h"
#include "main.h"
#include "button_mode.h"
#include "gyro_rate_control.h"
#include "speed_pid.h"
#include "semicircle_follow_plus.h"

/*
 *  半圆循迹 Plus
 *
 *  控制架构:
 *    直道: 角度环 (GoStraight) 锁定航向 + 位置环灰度居中
 *    弯道: 角速度环 (GyroRateLoop) 恒速转弯 + 位置环灰度微调
 *          双环并联，角速度环负责主转向，位置环微调对线
 *
 *  入弯: f6 连续 N 帧见黑 → 切弯道
 *  出弯: 陀螺仪累计角度 > 170° 且 f4/f5 见黑 → 切直道
 */

typedef enum {
    STATE_STRAIGHT = 0,
    STATE_CURVE    = 1,
} FollowState_t;

static float SemicircleFollowPlus_GetTargetBaseSpeed(uint32_t elapsed_ms)
{
    float cruise_speed = (float)g_semicircle_speed_max;

    if (g_run_mode == MODE_KEY2 && elapsed_ms < CAR_KEY2_STARTUP_RAMP_MS)
    {
        float t = (float)elapsed_ms / (float)CAR_KEY2_STARTUP_RAMP_MS;
        return (float)CAR_KEY2_STARTUP_SPEED
             + (cruise_speed - (float)CAR_KEY2_STARTUP_SPEED) * t;
    }

    if (g_run_mode == MODE_KEY3 && elapsed_ms < CAR_KEY3_STARTUP_RAMP_MS)
    {
        float t = (float)elapsed_ms / (float)CAR_KEY3_STARTUP_RAMP_MS;
        return (float)CAR_KEY3_STARTUP_SPEED
             + (cruise_speed - (float)CAR_KEY3_STARTUP_SPEED) * t;
    }

    return cruise_speed;
}

static uint32_t SemicircleFollowPlus_GetReentryCooldownMs(void)
{
    if (g_run_mode == MODE_KEY1 || g_run_mode == MODE_KEY3)
        return 5000;

    return CAR_SEMICIRCLE_CURVE_REENTRY_COOLDOWN_MS;
}

void SemicircleFollowPlus_Run(float dt)
{
    const uint8_t key2_straight_only = (g_run_mode == MODE_KEY2);
    static uint32_t stop_cooldown_start = 0;
    static uint8_t  stop_cooldown_set   = 0;
    static float    sc_base_speed_f     = 0.0f;
    static uint8_t  sc_speed_inited     = 0;
    static uint32_t lost_since          = 0;
    static uint8_t  was_lost            = 0;

    static FollowState_t state          = STATE_STRAIGHT;
    static uint8_t  curve_gyro_inited    = 0;
    static uint8_t  straight_angle_inited = 0;
    static uint8_t  params_inited        = 0;
    static uint32_t curve_exit_time_ms = 0;

    static float gyro_dps;
    static float exit_angle;
    static float speed_min;
    static float speed_max;
    static uint32_t lost_timeout;

    if (!params_inited)
    {
        if (g_run_mode == MODE_KEY1)
        {
            GyroRateLoop_SetKP(CAR_KEY1_CASCADE_INNER_KP);
            GyroRateLoop_SetKI(CAR_KEY1_CASCADE_INNER_KI);
            GyroRateLoop_SetKD(CAR_KEY1_CASCADE_INNER_KD);
            GoStraight_SetKP(CAR_KEY1_ANGLE_KP);
            GoStraight_SetKD(CAR_KEY1_ANGLE_KD);
            FollowLoop_SetStraightPID(CAR_KEY1_STRAIGHT_FOLLOW_KP,
                                      CAR_KEY1_STRAIGHT_FOLLOW_KI,
                                      CAR_KEY1_STRAIGHT_FOLLOW_KD);
            gyro_dps = CAR_KEY1_CASCADE2_GYRO_DPS;
            exit_angle = CAR_KEY1_CASCADE2_EXIT_ANGLE;
            speed_min = (float)CAR_KEY1_CASCADE2_SPEED_MIN;
            speed_max = (float)CAR_KEY1_CASCADE2_SPEED_MAX;
            lost_timeout = CAR_KEY1_CASCADE2_LOST_TIMEOUT_MS;
        }
        else if (g_run_mode == MODE_KEY2)
        {
            GyroRateLoop_SetKP(CAR_KEY2_CASCADE_INNER_KP);
            GyroRateLoop_SetKI(CAR_KEY2_CASCADE_INNER_KI);
            GyroRateLoop_SetKD(CAR_KEY2_CASCADE_INNER_KD);
            GoStraight_SetKP(CAR_KEY2_ANGLE_KP);
            GoStraight_SetKD(CAR_KEY2_ANGLE_KD);
            FollowLoop_SetStraightPID(CAR_KEY2_STRAIGHT_FOLLOW_KP,
                                      CAR_KEY2_STRAIGHT_FOLLOW_KI,
                                      CAR_KEY2_STRAIGHT_FOLLOW_KD);
            FollowLoop_SetCurvePID(CAR_KEY2_CASCADE_CURVE_POS_KP,
                                   CAR_KEY2_CASCADE_CURVE_POS_KI,
                                   CAR_KEY2_CASCADE_CURVE_POS_KD);
            gyro_dps = CAR_KEY2_CASCADE2_GYRO_DPS;
            exit_angle = CAR_KEY2_CASCADE2_EXIT_ANGLE;
            speed_min = (float)CAR_KEY2_CASCADE2_SPEED_MIN;
            speed_max = (float)CAR_KEY2_CASCADE2_SPEED_MAX;
            lost_timeout = CAR_KEY2_CASCADE2_LOST_TIMEOUT_MS;
        }
        else
        {
            GyroRateLoop_SetKP(CAR_KEY3_CASCADE_INNER_KP);
            GyroRateLoop_SetKI(CAR_KEY3_CASCADE_INNER_KI);
            GyroRateLoop_SetKD(CAR_KEY3_CASCADE_INNER_KD);
            GoStraight_SetKP(CAR_KEY3_ANGLE_KP);
            GoStraight_SetKD(CAR_KEY3_ANGLE_KD);
            FollowLoop_SetStraightPID(CAR_KEY3_STRAIGHT_FOLLOW_KP,
                                      CAR_KEY3_STRAIGHT_FOLLOW_KI,
                                      CAR_KEY3_STRAIGHT_FOLLOW_KD);
            FollowLoop_SetCurvePID(CAR_KEY3_CASCADE_CURVE_POS_KP,
                                   CAR_KEY3_CASCADE_CURVE_POS_KI,
                                   CAR_KEY3_CASCADE_CURVE_POS_KD);
            gyro_dps = CAR_KEY3_CASCADE2_GYRO_DPS;
            exit_angle = CAR_KEY3_CASCADE2_EXIT_ANGLE;
            speed_min = (float)CAR_KEY3_CASCADE2_SPEED_MIN;
            speed_max = (float)CAR_KEY3_CASCADE2_SPEED_MAX;
            lost_timeout = CAR_KEY3_CASCADE2_LOST_TIMEOUT_MS;
        }
        params_inited = 1;
    }

    if (!sc_speed_inited)
    {
        if (g_run_mode == MODE_KEY2 || g_run_mode == MODE_KEY3)
            sc_base_speed_f = SemicircleFollowPlus_GetTargetBaseSpeed(0);
        else
            sc_base_speed_f = (g_run_mode == MODE_KEY1) ? (float)g_semicircle_speed_max : 0.0f;
        sc_speed_inited = 1;

        if (g_mode_start_time == 0)
            g_mode_start_time = tick_ms;

        Mixer_SetDiffScale(CAR_SEMICIRCLE_DIFF_SCALE);
        Mixer_SetOuterBoost(CAR_SEMICIRCLE_OUTER_BOOST);
    }

    IRDM_read_sensors();

    if (!stop_cooldown_set)
    {
        stop_cooldown_start = tick_ms;
        stop_cooldown_set   = 1;
    }

    uint8_t black_cnt = IRDM_CountBlackSensors();
    if (black_cnt >= CAR_SEMICIRCLE_STOP_MIN_BLACK
        && (int32_t)(tick_ms - stop_cooldown_start)
           >= CAR_SEMICIRCLE_STOP_COOLDOWN_MS)
    {
        TB6612_Motor_Stop();
        g_run_mode = MODE_DONE;
        return;
    }

    if (g_run_mode == MODE_KEY2
        && g_mode_start_time != 0
        && (int32_t)(tick_ms - g_mode_start_time) >= CAR_KEY2_TIMEOUT_MS)
    {
        TB6612_Motor_Stop();
        g_run_mode = MODE_DONE;
        return;
    }

    if (!IRDM_IsBlackLine())
    {
        was_lost = 1;

        if (lost_since == 0)
            lost_since = tick_ms;

        if ((int32_t)(tick_ms - lost_since) >= lost_timeout)
        {
            TB6612_Motor_Stop();
            g_run_mode = MODE_DONE;
        }
        else
        {
            Mixer_SetFollowDiff(0);
            Mixer_SetAngleDiff(0);
            Mixer_SetSpeedDiff(0);
            Mixer_SetGyroRateDiff(0);
            Mixer_Apply();
        }
        return;
    }

    was_lost   = 0;
    lost_since = 0;

    uint8_t s[8];
    uint8_t i;
    for (i = 0; i < 8; i++)
        s[i] = IRDM_get_sensor_state(i);

    uint8_t f4f5_only = (s[3] && s[4]
                         && !s[0] && !s[1] && !s[2]
                         && !s[5] && !s[6] && !s[7]);
    uint8_t startup_ignore_curve = 0;
    if ((g_run_mode == MODE_KEY1 || g_run_mode == MODE_KEY3)
        && g_mode_start_time != 0
        && (int32_t)(tick_ms - g_mode_start_time) < CAR_SEMICIRCLE_CURVE_STARTUP_IGNORE_MS)
    {
        startup_ignore_curve = 1;
    }

    /* ---- 状态转移检测 ---- */

    if (key2_straight_only)
    {
        state = STATE_STRAIGHT;
        curve_gyro_inited = 0;
    }
    else
    {
        if (state == STATE_STRAIGHT)
        {
            uint32_t elapsed_since_exit = tick_ms - curve_exit_time_ms;
            if (!startup_ignore_curve
                && s[5]
                && elapsed_since_exit >= SemicircleFollowPlus_GetReentryCooldownMs())
            {
                state = STATE_CURVE;
                curve_gyro_inited = 0;
                GoStraight_Stop();
                FollowLoop_SwitchToCurve();
                IMU_AHRS_TurnAngle_Reset();
            }
        }

        if (state == STATE_CURVE)
        {
            float angle_acc = IMU_AHRS_TurnAngle_Get();
            if (angle_acc < 0.0f) angle_acc = -angle_acc;

            if (angle_acc >= exit_angle && f4f5_only)
            {
                state                = STATE_STRAIGHT;
                straight_angle_inited = 0;
                curve_exit_time_ms   = tick_ms;
                FollowLoop_SwitchToStraight();
                FollowLoop_ResetIntegral();
                GyroRateLoop_Reset();
                curve_gyro_inited = 0;
            }
        }
    }

    /* ---- 执行当前状态 ---- */

    if (state == STATE_STRAIGHT)
    {
        if (!straight_angle_inited)
        {
            float cur_yaw = IMU_AHRS_Get_Yaw_Compensated();
            GoStraight_StartAt(cur_yaw, CAR_STRAIGHT_SPEED);
            straight_angle_inited = 1;
        }

        IRDM_UpdatePositionPID(dt);
        int16_t follow_corr = IRDM_GetCorrection();
        float bias     = FollowLoop_GetBias();
        float bias_abs = (bias > 0.0f) ? bias : -bias;

        if (!key2_straight_only)
            FollowLoop_DecayIntegral(CAR_SEMICIRCLE_INTEGRAL_DECAY);

        float angle_weight;
        {
            float release_start = CAR_SEMICIRCLE_ANGLE_RELEASE_START;
            float release_end   = CAR_SEMICIRCLE_ANGLE_RELEASE_END;
            float release_range = release_end - release_start;

            if (bias_abs < release_start)
                angle_weight = 1.0f;
            else if (bias_abs < release_end)
                angle_weight = 1.0f - (bias_abs - release_start) / release_range;
            else
                angle_weight = 0.0f;

            float bias_deriv_abs = FollowLoop_GetBiasDeriv();
            if (bias_deriv_abs < 0.0f) bias_deriv_abs = -bias_deriv_abs;

            float deriv_factor = 1.0f - bias_deriv_abs * CAR_SEMICIRCLE_BIAS_DERIV_GAIN;
            if (deriv_factor < 0.0f) deriv_factor = 0.0f;
            if (deriv_factor > 1.0f) deriv_factor = 1.0f;

            angle_weight *= deriv_factor;

            if (angle_weight > 0.8f) angle_weight = 0.8f;
        }

        {
            uint32_t elapsed_ms = (g_mode_start_time == 0) ? 0 : (tick_ms - g_mode_start_time);
            float target_base = SemicircleFollowPlus_GetTargetBaseSpeed(elapsed_ms);

            if ((g_run_mode == MODE_KEY2 || g_run_mode == MODE_KEY3)
                && elapsed_ms < ((g_run_mode == MODE_KEY2) ? CAR_KEY2_STARTUP_RAMP_MS : CAR_KEY3_STARTUP_RAMP_MS))
            {
                sc_base_speed_f = target_base;
            }
            else if (dt > 0.0001f)
            {
                float ramp_step = CAR_SEMICIRCLE_SPEED_RAMP_UP * dt;
                if (target_base > sc_base_speed_f)
                {
                    float d = target_base - sc_base_speed_f;
                    if (d > ramp_step) d = ramp_step;
                    sc_base_speed_f += d;
                }
                else
                {
                    float d = sc_base_speed_f - target_base;
                    if (d > ramp_step) d = ramp_step;
                    sc_base_speed_f -= d;
                }
            }
            else
            {
                sc_base_speed_f = target_base;
            }
        }

        GoStraight_Poll();
        int16_t angle_corr;
        if (angle_weight > 0.001f)
        {
            angle_corr = (int16_t)((int32_t)GoStraight_GetCorrection()
                         * angle_weight);
        }
        else
        {
            angle_corr = 0;
        }

        Mixer_SetBaseSpeed((uint32_t)sc_base_speed_f);
        Mixer_SetFollowDiff(follow_corr);
        Mixer_SetAngleDiff(-angle_corr);
        Mixer_SetGyroRateDiff(0);
        Mixer_SetSpeedDiff(SpeedLoop_GetCorrection());
        Mixer_Apply();
    }
    else
    {
        if (key2_straight_only)
            return;

        if (!curve_gyro_inited)
        {
            GyroRateLoop_Init(gyro_dps);
            GyroRateLoop_SetTarget(gyro_dps);
            curve_gyro_inited = 1;
        }

        GyroRateLoop_Update(dt);

        {
            uint32_t elapsed_ms = (g_mode_start_time == 0) ? 0 : (tick_ms - g_mode_start_time);
            float target_base = SemicircleFollowPlus_GetTargetBaseSpeed(elapsed_ms);

            if ((g_run_mode == MODE_KEY2 || g_run_mode == MODE_KEY3)
                && elapsed_ms < ((g_run_mode == MODE_KEY2) ? CAR_KEY2_STARTUP_RAMP_MS : CAR_KEY3_STARTUP_RAMP_MS))
            {
                sc_base_speed_f = target_base;
            }
            else if (dt > 0.0001f)
            {
                float ramp_step = CAR_SEMICIRCLE_SPEED_RAMP_UP * dt;
                if (target_base > sc_base_speed_f)
                {
                    float d = target_base - sc_base_speed_f;
                    if (d > ramp_step) d = ramp_step;
                    sc_base_speed_f += d;
                }
                else
                {
                    float d = sc_base_speed_f - target_base;
                    if (d > ramp_step) d = ramp_step;
                    sc_base_speed_f -= d;
                }
            }
            else
            {
                sc_base_speed_f = target_base;
            }
        }

        if (sc_base_speed_f < speed_min)
            sc_base_speed_f = speed_min;
        if (sc_base_speed_f > speed_max)
            sc_base_speed_f = speed_max;

        IRDM_UpdatePositionPID(dt);
        int16_t follow_corr = IRDM_GetCorrection();

        Mixer_SetBaseSpeed((uint32_t)sc_base_speed_f);
        Mixer_SetFollowDiff(follow_corr);
        Mixer_SetAngleDiff(0);
        Mixer_SetGyroRateDiff(GyroRateLoop_GetCorrection());
        Mixer_SetSpeedDiff(SpeedLoop_GetCorrection());
        Mixer_Apply();
    }
}