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
#include "button_mode.h"
#include "mode_speed_square.h"
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

#if USE_GYRO_RATE_CONTROL
    GyroRateLoop_Init(0.0f);  /* 目标角速度 0°/s（抑制旋转） */
#endif

#if USE_VOFA_DEBUG
    VOFA_Init();
#endif

#if USE_SPEED_CONTROL
    Motor_Init();
    SpeedLoop_Init(200);
#endif

#if USE_OBSTACLE_AVOIDANCE
    Ultrasonic_Avoidance_Init();
#endif

    /* ================================================================
     *  按键模式选择阶段
     *  KEY1(B18): 20s 半圆循线
     *  KEY2(B17): 直线行走，右传感器(f7)见黑即停
     *  KEY3(B19): 30s 半圆循线
     * ================================================================ */
    ButtonMode_Select();

    while (1) 
    {
#if USE_OBSTACLE_AVOIDANCE
        Ultrasonic_Test_Display(oled_buffer);
        continue;
#endif
        static uint32_t last_update = 0;
        uint32_t now = tick_ms;
        float dt = (float)(now - last_update) / 1000.0f;
        last_update = now;

        /* 运动时间显示（OLED 第4行），停机后冻结 */
        ButtonMode_UpdateTimeDisplay(now);

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

        /* 模式已完成，停止一切控制 */
        if (g_run_mode == MODE_DONE)
        {
            continue;
        }

#if USE_SPEED_CONTROL
        SpeedLoop_Update(dt);
#endif

#if USE_GYRO_RATE_CONTROL
        GyroRateLoop_Update(dt);
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
        SemicircleFollow_Run(dt);
#elif USE_SPEED_CONTROL && !USE_ANGLE_CONTROL && !USE_FOLLOW_CONTROL
        ModeSpeedLoop_Run(dt, now);
#else
        ModeSquareTrack_Run(dt, now);
#endif
    }
}