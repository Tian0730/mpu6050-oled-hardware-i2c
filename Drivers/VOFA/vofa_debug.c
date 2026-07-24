#include "vofa_debug.h"
#include "board.h"
#include "imu_ahrs.h"
#include "speed_control.h"
#include "angle_pid.h"
#include "follow.h"
#include "turn.h"
#include "bsp_tb6612.h"
#include "ti_msp_dl_config.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

/* ================================================================
 *  VOFA+ FireWater 协议：MCU → VOFA+
 *    格式: ch1,ch2,ch3,...,chN\r\n
 *    VOFA+ 中选 FireWater 协议即可自动解析
 * ================================================================ */

uint8_t g_vofa_stop = 0;

void VOFA_Init(void)
{
    lc_printf("[VOFA] Mode=%d (0=Speed 1=Angle 2=Follow) Ready\r\n", VOFA_MODE);
}

void VOFA_Output(void)
{
#if VOFA_MODE == 0
    /* ch0:目标  ch1:当前  ch2:误差  ch3:P  ch4:I  ch5:D  ch6:输出 */
    lc_printf("%.1f,%.1f,%.1f,%.2f,%.2f,%.2f,%.2f\r\n",
              SpeedLoop_GetTarget(),
              SpeedLoop_GetCurrentSpeed(),
              SpeedLoop_GetError(),
              SpeedLoop_GetPTerm(),
              SpeedLoop_GetITerm(),
              SpeedLoop_GetDTerm(),
              (float)SpeedLoop_GetCorrection());

#elif VOFA_MODE == 1
    /* ch0:目标Yaw  ch1:当前Yaw  ch2:误差  ch3:P  ch4:D  ch5:输出 */
    lc_printf("0.0,%.1f,%.2f,%.2f,%.2f,%.2f\r\n",
              IMU_AHRS_Get_Yaw_Compensated(),
              GoStraight_GetError(),
              GoStraight_GetPTerm(),
              GoStraight_GetDTerm(),
              GoStraight_GetOutput());

#elif VOFA_MODE == 2
    /* ch0:偏差  ch1:P  ch2:I  ch3:D  ch4:修正量  ch5:传感器mask */
    uint8_t mask = 0;
    for (int i = 0; i < 8; i++)
        if (IRDM_get_sensor_state(i)) mask |= (1 << i);

    lc_printf("%.2f,%.2f,%.2f,%.2f,%d,%d\r\n",
              FollowLoop_GetBias(),
              FollowLoop_GetPTerm(),
              FollowLoop_GetITerm(),
              FollowLoop_GetDTerm(),
              IRDM_GetCorrection(),
              mask);
#endif
}

/* ================================================================
 *  VOFA+ → MCU 命令接收
 *    格式: KP=1.5  /  KI=0.02  /  KD=0.3
 *    根据 VOFA_MODE 自动路由到当前调试的环
 * ================================================================ */
#define RX_BUF_SIZE  32
static char  rx_buf[RX_BUF_SIZE];
static uint8_t rx_idx = 0;

static void VOFA_ParseCommand(const char *cmd)
{
    if (strcmp(cmd, "STOP") == 0 || strcmp(cmd, "stop") == 0)
    {
        g_vofa_stop = 1;
        TB6612_Motor_Stop();
        GoStraight_Stop();
        Turn_Stop();
        lc_printf("[VOFA] STOP! Motors halted\r\n");
        return;
    }

    if (strcmp(cmd, "GO") == 0 || strcmp(cmd, "go") == 0)
    {
        g_vofa_stop = 0;
        lc_printf("[VOFA] GO! Resume\r\n");
        return;
    }

    float val;
    char  key[4] = {0};

    if (sscanf(cmd, "%3[^=]=%f", key, &val) != 2) return;

    if (strcmp(key, "KP") == 0 || strcmp(key, "kp") == 0)
    {
#if VOFA_MODE == 0
        SpeedLoop_SetKP(val);
#elif VOFA_MODE == 1
        GoStraight_SetKP(val);
#elif VOFA_MODE == 2
        FollowLoop_SetKP(val);
#endif
    }
    else if (strcmp(key, "KI") == 0 || strcmp(key, "ki") == 0)
    {
#if VOFA_MODE == 0
        SpeedLoop_SetKI(val);
#elif VOFA_MODE == 2
        FollowLoop_SetKI(val);
#endif
    }
    else if (strcmp(key, "KD") == 0 || strcmp(key, "kd") == 0)
    {
#if VOFA_MODE == 0
        SpeedLoop_SetKD(val);
#elif VOFA_MODE == 1
        GoStraight_SetKD(val);
#elif VOFA_MODE == 2
        FollowLoop_SetKD(val);
#endif
    }
}

void VOFA_ReceivePoll(void)
{
    while (DL_UART_Main_isRXFIFOEmpty(UART_0_INST) == false)
    {
        char c = (char)DL_UART_Main_receiveData(UART_0_INST);

        if (c == '\r' || c == '\n')
        {
            if (rx_idx > 0)
            {
                rx_buf[rx_idx] = '\0';
                VOFA_ParseCommand(rx_buf);
                rx_idx = 0;
            }
        }
        else if (rx_idx < RX_BUF_SIZE - 1)
        {
            rx_buf[rx_idx++] = c;
        }
    }
}