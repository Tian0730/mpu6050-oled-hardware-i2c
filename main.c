/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ti_msp_dl_config.h"
#include "main.h"
#include "stdio.h"

uint8_t oled_buffer[32];

int main(void)
{
    SYSCFG_DL_init();
    SysTick_Init();

    TB6612_Motor_Stop();
    // AO_Control(1, 300);
    // BO_Control(1, 300);
    // mspm0_delay_ms(2000);
    // TB6612_Motor_Stop();

    mspm0_delay_ms(50);

    for(int i = 0;i < 4; i++)
    {
        /* 这相当于新的 MPU6050_Init() */
        IMU_AHRS_Init_Start();
        while (IMU_AHRS_Init_Poll() == IMU_AHRS_STATUS_INIT_BUSY)
        {
            mspm0_delay_ms(1);
        }
        IMU_AHRS_Attitude_Enable();
        IMU_AHRS_Yaw_Calib_Start();
    }
    
    OLED_Init();

    /* Don't remove this! */
    Interrupt_Init();

    /*
    OLED_ShowString(0,7,(uint8_t *)"MPU6050 Demo",8);

    OLED_ShowString(0,0,(uint8_t *)"Pitch",8);
    OLED_ShowString(0,2,(uint8_t *)" Roll",8);
    OLED_ShowString(0,4,(uint8_t *)"  Yaw",8);

    OLED_ShowString(16*6,3,(uint8_t *)"Accel",8);
    OLED_ShowString(17*6,4,(uint8_t *)"Turn",8);
    */

    Mixer_Init(CAR_DEFAULT_BASE_SPEED);

#if USE_VOFA_DEBUG
    VOFA_Init();
#endif

#if USE_SEMICIRCLE_FOLLOW
    FollowLoop_SetKP(CAR_SEMICIRCLE_CURVE_FOLLOW_KP);
    FollowLoop_SetKI(CAR_SEMICIRCLE_CURVE_FOLLOW_KI);
    FollowLoop_SetKD(CAR_SEMICIRCLE_CURVE_FOLLOW_KD);
    Mixer_SetBaseSpeed(CAR_SEMICIRCLE_SPEED);
#endif

#if USE_SPEED_CONTROL
    Motor_Init();
    SpeedLoop_Init(200);
#endif

#if USE_OBSTACLE_AVOIDANCE
    Ultrasonic_Avoidance_Init();
#endif

    while (1) 
    {
#if USE_OBSTACLE_AVOIDANCE
        Ultrasonic_Test_Display(oled_buffer);       // 测试模式
        // Ultrasonic_Avoidance_Update(oled_buffer);   // 避障模式
        continue;
#endif
        static uint32_t last_update = 0;
        uint32_t now = tick_ms;
        float dt = (float)(now - last_update) / 1000.0f;
        last_update = now;

        IMU_AHRS_Update_Data();           // 读取原始数据
        IMU_AHRS_Update_Attitude(dt);     // Mahony 姿态解算

#if USE_VOFA_DEBUG
        VOFA_ReceivePoll();
        static uint32_t vofa_last = 0;
        if ((int32_t)(now - vofa_last) >= 20)
        {
            vofa_last = now;
            VOFA_Output();
        }
        if (g_vofa_stop)
        {
            TB6612_Motor_Stop();
            continue;
        }
#endif

#if USE_SPEED_CONTROL
        SpeedLoop_Update(dt);
#endif

#if USE_ANGLE_CONTROL || USE_FOLLOW_CONTROL
        static int8_t yaw_calib_done = 0;
        if (yaw_calib_done == 0)
        {
            int8_t ret = IMU_AHRS_Yaw_Calib_Poll();
            if (ret == 1)
            {
                yaw_calib_done = 1;
                IMU_AHRS_TurnAngle_Reset();
#if USE_ANGLE_CONTROL
                GoStraight_Start(300);
#endif
            }
            continue;
        }
#endif

        /*
        static uint32_t oled_last_update = 0;
        if ((int32_t)(now - oled_last_update) >= 50)
        {
            oled_last_update = now;

            const IMU_AHRS_Data_t *imu = IMU_AHRS_GetData();

            sprintf((char *)oled_buffer, "%-6.1f", imu->pitch);
            OLED_ShowString(5*8,0,oled_buffer,16);
            sprintf((char *)oled_buffer, "%-6.1f", imu->roll);
            OLED_ShowString(5*8,2,oled_buffer,16);
            sprintf((char *)oled_buffer, "%-6.1f", IMU_AHRS_Get_Yaw_Compensated());
            OLED_ShowString(5*8,4,oled_buffer,16);

            sprintf((char *)oled_buffer, "%6d", imu->ax);
            OLED_ShowString(15*6,0,oled_buffer,8);
            sprintf((char *)oled_buffer, "%6d", imu->ay);
            OLED_ShowString(15*6,1,oled_buffer,8);
            sprintf((char *)oled_buffer, "%6d", imu->az);
            OLED_ShowString(15*6,2,oled_buffer,8);

            sprintf((char *)oled_buffer, "%6.1f", IMU_AHRS_TurnAngle_Get());
            OLED_ShowString(15*6,5,oled_buffer,8);
        }
*/

#if USE_SEMICIRCLE_FOLLOW
        /* ================================================================
         *  圆角矩形赛道巡线（闭合赛道 + 启停线）
         *
         *  赛道：半圆 → 直道 → 半圆(反向) → 直道 → 回到起点
         *  启停线：5cm 长、1.8cm 宽、居中、与赛道垂直
         *
         *  核心策略：
         *    1. 位置环 PID 主导转向（灰度传感器反馈）
         *    2. 动态角度环：直道全开(稳定航向)，弯道抑制(灵活转向)
         *    3. 积分管理：弯道正常累积，直道逐步衰减，丢线恢复后复位
         *    4. 速度环恒速
         *    5. >=6 路传感器同时见黑 + 冷却已过 → 启停线，停车
         *
         *  OLED 显示：
         *    ST = 直道  TR = 过渡  CV = 弯道
         *    B:偏差  A:角度环%  BK:黑点数
         * ================================================================ */
        {
            static uint32_t lost_since          = 0;
            static uint8_t  was_lost            = 0;
            static uint32_t stop_cooldown_start = 0;
            static uint8_t  stop_cooldown_set   = 0;
            static float    sc_base_speed_f     = (float)CAR_SEMICIRCLE_SPEED_MAX;

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
                sprintf((char *)oled_buffer, "STOP BK:%d", black_cnt);
                OLED_ShowString(0, 6, oled_buffer, 16);
                continue;
            }

            if (IRDM_IsBlackLine())
            {
                /*
                 *  丢线恢复：复位积分，防止残留积分导致恢复后跑偏
                 */
                if (was_lost)
                {
                    was_lost = 0;
                    FollowLoop_ResetIntegral();
                }
                lost_since = 0;

                IRDM_UpdatePositionPID(dt);
                int16_t follow_corr = IRDM_GetCorrection();
                float bias = FollowLoop_GetBias();
                float bias_abs = (bias > 0.0f) ? bias : -bias;

                /*
                 *  直道/弯道切换（带迟滞） & 角度环航向自适应
                 *
                 *  进入弯道阈值：bias_abs >= 0.4（弯道起始处弯度小，需尽早切换）
                 *  退出弯道阈值：bias_abs <  0.2（确保回到直道才切回）
                 *  迟滞带 0.2~0.4 防止在阈值附近来回切换
                 */
                static uint8_t was_on_curve = 0;
                if (bias_abs < CAR_SEMICIRCLE_CURVE_EXIT_BIAS && was_on_curve)
                {
                    was_on_curve = 0;
                    FollowLoop_SwitchToStraight();
#if USE_ANGLE_CONTROL
                    float cur_yaw = IMU_AHRS_Get_Yaw_Compensated();
                    GoStraight_StartAt(cur_yaw, CAR_STRAIGHT_SPEED);
#endif
                }
                else if (bias_abs >= CAR_SEMICIRCLE_CURVE_ENTER_BIAS && !was_on_curve)
                {
                    was_on_curve = 1;
                    FollowLoop_SwitchToCurve();
#if USE_ANGLE_CONTROL
                    GoStraight_ResetPD();
#endif
                }

                /*
                 *  积分管理：
                 *    偏差 < 0.2（直道）→ 逐步衰减积分，弯道残留不带入直道
                 *    偏差 ≥ 0.2（弯道/过渡）→ 积分正常累积
                 */
                if (bias_abs < CAR_SEMICIRCLE_CURVE_EXIT_BIAS)
                    FollowLoop_DecayIntegral(CAR_SEMICIRCLE_INTEGRAL_DECAY);

                /*
                 *  角度环：渐变权重，偏差越大角度环占比越小
                 *
                 *  改进：角度环松手曲线前移，在偏差还很小时就开始逐步放手
                 *    旧：bias < 0.18→100%  |  0.18~0.38→线性  |  ≥0.38→0%
                 *    新：bias < 0.06→100%  |  0.06~0.22→线性  |  ≥0.22→0%
                 *
                 *  额外：偏差导数前馈——偏差变化越快，角度环越早松手
                 *    入弯时偏差快速增大 → 即使绝对值还小也提前松手 → 避免冲到外道
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

                    /* 角度环上限钳位：直道角度环主导，留少许位置环做微调 */
                    if (angle_weight > 0.8f) angle_weight = 0.8f;
                }

                /* 曲率-速度联动：偏差越大越降速，并用不同升降斜率做平滑 */
                float span = CAR_SEMICIRCLE_CURVE_ENTER_BIAS;
                float curve_level = (span > 0.001f) ? (bias_abs / span) : 1.0f;
                if (curve_level > 1.0f) curve_level = 1.0f;

                float target_base = (float)CAR_SEMICIRCLE_SPEED_MAX
                                  - ((float)(CAR_SEMICIRCLE_SPEED_MAX - CAR_SEMICIRCLE_SPEED_MIN)
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

                if (sc_base_speed_f < (float)CAR_SEMICIRCLE_SPEED_MIN)
                    sc_base_speed_f = (float)CAR_SEMICIRCLE_SPEED_MIN;
                if (sc_base_speed_f > (float)CAR_SEMICIRCLE_SPEED_MAX)
                    sc_base_speed_f = (float)CAR_SEMICIRCLE_SPEED_MAX;

                Mixer_SetBaseSpeed((uint32_t)sc_base_speed_f);

#if USE_ANGLE_CONTROL
                int16_t angle_corr;
                if (angle_weight > 0.001f)
                {
                    GoStraight_Poll();
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

                /*
                 *  浅弯道修正量增强：偏差小时放大修正，加快入弯响应
                 *    偏差 < 0.5 → 线性增强，bias=0时放大3.5倍（1+2.5）
                 *    偏差 ≥ 0.5 → 不增强
                 */
                {
                    float shallow_bias = CAR_SEMICIRCLE_SHALLOW_BOOST_BIAS;
                    float shallow_max  = CAR_SEMICIRCLE_SHALLOW_BOOST_MAX;
                    if (bias_abs < shallow_bias) {
                        float boost = 1.0f + (shallow_bias - bias_abs) / shallow_bias * shallow_max;
                        follow_corr = (int16_t)((float)follow_corr * boost);
                    }
                }

                Mixer_SetFollowDiff(follow_corr);
                Mixer_SetAngleDiff(angle_corr);
                Mixer_SetSpeedDiff(SpeedLoop_GetCorrection());
                Mixer_SetOuterBoost(CAR_SEMICIRCLE_OUTER_BOOST);
                Mixer_Apply();

                static uint32_t sc_oled_last = 0;
                if ((int32_t)(now - sc_oled_last) >= 200)
                {
                    sc_oled_last = now;
                    const char *mode = (bias_abs < CAR_SEMICIRCLE_CURVE_EXIT_BIAS) ? "ST" :
                                   (bias_abs < CAR_SEMICIRCLE_CURVE_ENTER_BIAS) ? "TR" : "CV";
                        sprintf((char *)oled_buffer, "%s B:%.1f V:%3d A:%d%%",
                            mode, bias, (int)sc_base_speed_f,
                            (int)(angle_weight * 100.0f));
                        OLED_ShowString(0, 6, oled_buffer, 16);
                        sprintf((char *)oled_buffer, "BK:%d P:%.0f D:%.0f",
                            black_cnt,
                            FollowLoop_GetPTerm(), FollowLoop_GetDTerm());
                        OLED_ShowString(0, 7, oled_buffer, 16);
                        sprintf((char *)oled_buffer, "I:%-.0f", FollowLoop_GetITerm());
                        OLED_ShowString(15*6, 7, oled_buffer, 8);
                }
            }
            else
            {
                was_lost = 1;

                if (lost_since == 0)
                    lost_since = tick_ms;

                if ((int32_t)(tick_ms - lost_since) >= CAR_SEMICIRCLE_LOST_TIMEOUT_MS)
                {
                    TB6612_Motor_Stop();
                    sprintf((char *)oled_buffer, "DONE");
                    OLED_ShowString(0, 6, oled_buffer, 16);
                }
                else
                {
                    Mixer_SetFollowDiff(0);
                    Mixer_SetAngleDiff(0);
                    Mixer_SetSpeedDiff(0);
                    Mixer_Apply();
                    sprintf((char *)oled_buffer, "LOST %lums",
                            tick_ms - lost_since);
                    OLED_ShowString(0, 6, oled_buffer, 16);
                }
            }
        }
#elif USE_SPEED_CONTROL && !USE_ANGLE_CONTROL && !USE_FOLLOW_CONTROL
        /* ================================================================
         *  纯速度环模式：恒定目标速度直行
         *  仅用于调试速度环 PID 参数
         * ================================================================ */
        {
            Mixer_SetSpeedDiff(SpeedLoop_GetCorrection());
            Mixer_SetAngleDiff(0);
            Mixer_SetFollowDiff(0);
            Mixer_Apply();

            static uint32_t spd_oled_last = 0;
            if ((int32_t)(now - spd_oled_last) >= 200)
            {
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
#else
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
            Mixer_SetAngleDiff(angle_corr);
#else
            Mixer_SetAngleDiff(0);
#endif
            Mixer_SetSpeedDiff(SpeedLoop_GetCorrection());
            Mixer_SetFollowDiff(0);
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
                Mixer_SetAngleDiff(angle_corr);
                Mixer_SetSpeedDiff(SpeedLoop_GetCorrection());
                Mixer_Apply();
            }
            else
            {
                /* 空白区域：按配置开关决定环参与 */
                // if (was_on_black)
                // {
                    was_on_black = 0;
// #if USE_ANGLE_CONTROL
//                     GoStraight_Start(300);
// #endif
                // }
#if USE_ANGLE_CONTROL
                GoStraight_Poll();
                int16_t angle_corr = GoStraight_GetCorrection();
#else
                int16_t angle_corr = 0;
#endif
                Mixer_SetSpeedDiff(SpeedLoop_GetCorrection());
                // Mixer_SetSpeedDiff(0);
                Mixer_SetAngleDiff(angle_corr);
                Mixer_SetFollowDiff(0);
                Mixer_Apply();

                sprintf((char *)oled_buffer, "WHITE");
                OLED_ShowString(0, 6, oled_buffer, 16);
            }
        }
#endif
    }
}