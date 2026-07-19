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

    OLED_ShowString(0,7,(uint8_t *)"MPU6050 Demo",8);

    OLED_ShowString(0,0,(uint8_t *)"Pitch",8);
    OLED_ShowString(0,2,(uint8_t *)" Roll",8);
    OLED_ShowString(0,4,(uint8_t *)"  Yaw",8);

    OLED_ShowString(16*6,3,(uint8_t *)"Accel",8);
    OLED_ShowString(17*6,4,(uint8_t *)"Turn",8);

    Mixer_Init(300);

#if USE_SPEED_CONTROL
    Motor_Init();
    SpeedLoop_Init(200);
#endif

    while (1) 
    {
        static uint32_t last_update = 0;
        uint32_t now = tick_ms;
        float dt = (float)(now - last_update) / 1000.0f;
        last_update = now;

        IMU_AHRS_Update_Data();           // 读取原始数据
        IMU_AHRS_Update_Attitude(dt);     // Mahony 姿态解算

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

        /* OLED 每 50ms 更新一次，避免阻塞控制回路 */
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

#if USE_SPEED_CONTROL && !USE_ANGLE_CONTROL && !USE_FOLLOW_CONTROL
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
        #define PRE_TURN_MS    300
        #define TURN_COOLDOWN  500

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
