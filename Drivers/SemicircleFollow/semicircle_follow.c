#include "ti_msp_dl_config.h"
#include "main.h"
#include "button_mode.h"
#include "gyro_rate_control.h"
#include "semicircle_follow.h"

/* ================================================================
 *  圆角矩形赛道巡线（精简版）
 *
 *  直道/弯道: 仅位置环 PID 灰度循迹
 *  切换: 偏差阈值切换直道/弯道 PID
 * ================================================================ */
static float SemicircleFollow_GetTargetBaseSpeed(uint32_t elapsed_ms)
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

static uint32_t SemicircleFollow_GetReentryCooldownMs(void)
{
    if (g_run_mode == MODE_KEY1 || g_run_mode == MODE_KEY3)
        return 5000;

    return CAR_SEMICIRCLE_CURVE_REENTRY_COOLDOWN_MS;
}

void SemicircleFollow_Run(float dt)
{
    const uint8_t key2_straight_only = (g_run_mode == MODE_KEY2);
    static uint32_t lost_since          = 0;
    static uint8_t  was_lost            = 0;
    static uint32_t stop_cooldown_start = 0;
    static uint8_t  stop_cooldown_set   = 0;
    static float    sc_base_speed_f     = 0.0f;
    static uint8_t  sc_speed_inited     = 0;
    static uint32_t curve_exit_time_ms  = 0;
    static uint8_t  was_on_curve        = 0;

    if (!sc_speed_inited)
    {
        sc_base_speed_f = SemicircleFollow_GetTargetBaseSpeed(0);
        sc_speed_inited = 1;

        /* 小车第一次真正驱动电机时才开始计时 */
        if (g_mode_start_time == 0)
            g_mode_start_time = tick_ms;

        Mixer_SetDiffScale(CAR_SEMICIRCLE_DIFF_SCALE);
        Mixer_SetOuterBoost(CAR_SEMICIRCLE_OUTER_BOOST);

        #if USE_ANGLE_CONTROL
        if (g_run_mode == MODE_KEY1)
        {
            GoStraight_SetKP(CAR_KEY1_ANGLE_KP);
            GoStraight_SetKD(CAR_KEY1_ANGLE_KD);
        }
        else if (g_run_mode == MODE_KEY2)
        {
            GoStraight_SetKP(CAR_KEY2_ANGLE_KP);
            GoStraight_SetKD(CAR_KEY2_ANGLE_KD);
        }
        else if (g_run_mode == MODE_KEY3)
        {
            GoStraight_SetKP(CAR_KEY3_ANGLE_KP);
            GoStraight_SetKD(CAR_KEY3_ANGLE_KD);
        }
#endif
    }

    IRDM_read_sensors();

    /*
     *  启停线检测：>=6 路同时见黑 → 垂直线段
     *  冷却时间内不触发，防止起步时误停车
     */
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

    /* Key2 模式：7s 超时自动停车 */
    if (g_run_mode == MODE_KEY2
        && g_mode_start_time != 0
        && (int32_t)(tick_ms - g_mode_start_time) >= CAR_KEY2_TIMEOUT_MS)
    {
        TB6612_Motor_Stop();
        g_run_mode = MODE_DONE;
        return;
    }

    if (IRDM_IsBlackLine())
    {
        /*
         *  丢线恢复：复位积分，防止残留积分导致恢复后跑偏
         */
        if (was_lost)
        {
            was_lost = 0;
            /*FollowLoop_ResetIntegral();*/
        }
        lost_since = 0;

        IRDM_UpdatePositionPID(dt);
        int16_t follow_corr = IRDM_GetCorrection();
        float bias = FollowLoop_GetBias();
        float bias_abs = (bias > 0.0f) ? bias : -bias;
        uint8_t f6_seen = IRDM_get_sensor_state(5);
        uint8_t startup_ignore_curve = 0;
        if ((g_run_mode == MODE_KEY1 || g_run_mode == MODE_KEY3)
            && g_mode_start_time != 0
            && (int32_t)(tick_ms - g_mode_start_time) < CAR_SEMICIRCLE_CURVE_STARTUP_IGNORE_MS)
        {
            startup_ignore_curve = 1;
        }

        if (!key2_straight_only)
        {
            /*
             *  进入弯道：f6 一旦检测到就直接切到弯道 PID
             *  退出弯道：仍然沿用偏差迟滞，回到直道后再允许重入
             */
            if (bias_abs < CAR_SEMICIRCLE_CURVE_EXIT_BIAS && was_on_curve)
            {
                was_on_curve = 0;
                FollowLoop_SwitchToStraight();
                curve_exit_time_ms = tick_ms;
#if USE_ANGLE_CONTROL
                float cur_yaw = IMU_AHRS_Get_Yaw_Compensated();
                GoStraight_StartAt(cur_yaw, CAR_STRAIGHT_SPEED);
#endif
            }
            else if (!startup_ignore_curve && !was_on_curve)
            {
                uint32_t elapsed_since_exit = tick_ms - curve_exit_time_ms;
                uint8_t cooldown_ok = (curve_exit_time_ms == 0)
                                   || (elapsed_since_exit >= SemicircleFollow_GetReentryCooldownMs());
                if (cooldown_ok && (f6_seen || bias_abs >= CAR_SEMICIRCLE_CURVE_ENTER_BIAS))
                {
                    was_on_curve = 1;
                    FollowLoop_SwitchToCurve();
#if USE_ANGLE_CONTROL
                    GoStraight_ResetPD();
#endif
                }
            }

            /*
             *  积分管理：
             *    偏差 < 0.2（直道）→ 逐步衰减积分，弯道残留不带入直道
             *    偏差 ≥ 0.2（弯道/过渡）→ 积分正常累积
             */
            /*if (bias_abs < CAR_SEMICIRCLE_CURVE_EXIT_BIAS)
                FollowLoop_DecayIntegral(CAR_SEMICIRCLE_INTEGRAL_DECAY);*/
        }

        /*
         *  角度环：渐变权重，偏差越大角度环占比越小
         */
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

        if (was_on_curve)
            angle_weight = 0.0f;

        if (key2_straight_only)
        {
            bias_abs = 0.0f;
        }

        /* 曲率-速度联动：偏差越大越降速 */
        /*float span = CAR_SEMICIRCLE_CURVE_ENTER_BIAS;
        float curve_level = (span > 0.001f) ? (bias_abs / span) : 1.0f;
        if (curve_level > 1.0f) curve_level = 1.0f;

        float target_base = (float)g_semicircle_speed_max
                          - ((float)(g_semicircle_speed_max - g_semicircle_speed_min)
                          * curve_level);

        if (dt > 0.0001f)
        {
            float ramp_up_step = CAR_SEMICIRCLE_SPEED_RAMP_UP * dt;
            float ramp_dn_step = CAR_SEMICIRCLE_SPEED_RAMP_DOWN * dt;
            if (target_base > sc_base_speed_f)
            {
                float d = target_base - sc_base_speed_f;
                if (d > ramp_up_step) d = ramp_up_step;
                sc_base_speed_f += d;
            }
            else
            {
                float d = sc_base_speed_f - target_base;
                if (d > ramp_dn_step) d = ramp_dn_step;
                sc_base_speed_f -= d;
            }
        }
        else
        {
            sc_base_speed_f = target_base;
        }

        if (sc_base_speed_f > (float)g_semicircle_speed_max)
            sc_base_speed_f = (float)g_semicircle_speed_max;*/

        if (g_mode_start_time != 0)
        {
            uint32_t elapsed_ms = tick_ms - g_mode_start_time;
            sc_base_speed_f = SemicircleFollow_GetTargetBaseSpeed(elapsed_ms);
        }
        else
        {
            sc_base_speed_f = (float)g_semicircle_speed_max;
        }

        Mixer_SetBaseSpeed((uint32_t)sc_base_speed_f);

#if USE_ANGLE_CONTROL
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
#else
        int16_t angle_corr = 0;
#endif

        /* 浅弯道修正量增强 */
        /*{
            float shallow_bias = CAR_SEMICIRCLE_SHALLOW_BOOST_BIAS;
            float shallow_max  = CAR_SEMICIRCLE_SHALLOW_BOOST_MAX;
            if (bias_abs < shallow_bias) {
                float boost = 1.0f + (shallow_bias - bias_abs) / shallow_bias * shallow_max;
                follow_corr = (int16_t)((float)follow_corr * boost);
            }
        }*/

        Mixer_SetFollowDiff(follow_corr);
        Mixer_SetAngleDiff(-angle_corr);
        /*Mixer_SetSpeedDiff(SpeedLoop_GetCorrection());*/
        /*#if USE_GYRO_RATE_CONTROL
        Mixer_SetGyroRateDiff(GyroRateLoop_GetCorrection());
        #endif*/
        Mixer_Apply();
    }
    else
    {
        was_lost = 1;

        if (lost_since == 0)
            lost_since = tick_ms;

        if ((int32_t)(tick_ms - lost_since) >= CAR_SEMICIRCLE_LOST_TIMEOUT_MS)
        {
            TB6612_Motor_Stop();
            g_run_mode = MODE_DONE;
        }
        else
        {
            Mixer_SetFollowDiff(0);
            Mixer_SetAngleDiff(0);
            /*Mixer_SetSpeedDiff(0);*/
            /*#if USE_GYRO_RATE_CONTROL
            Mixer_SetGyroRateDiff(GyroRateLoop_GetCorrection());
            #endif*/
            Mixer_Apply();
        }
    }
}