#include "ti_msp_dl_config.h"
#include "main.h"
#include "button_mode.h"

/* 按键读取宏（上拉输入，按下为低电平） */
#define KEY1_PRESSED()  (!DL_GPIO_readPins(KEY_PORT, KEY_KEY1_PIN))
#define KEY2_PRESSED()  (!DL_GPIO_readPins(KEY_PORT, KEY_KEY2_PIN))
#define KEY3_PRESSED()  (!DL_GPIO_readPins(KEY_PORT, KEY_KEY3_PIN))

/* 全局状态变量 */
run_mode_t g_run_mode = MODE_WAITING;
uint32_t   g_mode_start_time = 0;
uint32_t   g_semicircle_base_speed = CAR_DEFAULT_BASE_SPEED;
uint32_t   g_semicircle_speed_min  = 0;
uint32_t   g_semicircle_speed_max  = 0;

void ButtonMode_Select(void)
{
    g_run_mode = MODE_WAITING;
    g_mode_start_time = 0;

    OLED_ShowString(0, 0, (uint8_t *)"SELECT MODE", 16);
    OLED_ShowString(0, 2, (uint8_t *)"1:20s 2:Line 3:30s", 16);

    while (g_run_mode == MODE_WAITING)
    {
        if (KEY1_PRESSED())
        {
            g_run_mode = MODE_KEY1;
            g_mode_start_time = 0;  /* 等小车真正动起来再开始计时 */
            g_semicircle_base_speed = CAR_KEY1_SPEED;
            g_semicircle_speed_min  = CAR_KEY1_SPEED_MIN;
            g_semicircle_speed_max  = CAR_KEY1_SPEED_MAX;
#if USE_SEMICIRCLE_FOLLOW
            FollowLoop_SetCurvePID(CAR_KEY1_CURVE_FOLLOW_KP, CAR_KEY1_CURVE_FOLLOW_KI, CAR_KEY1_CURVE_FOLLOW_KD);
            FollowLoop_SetStraightPID(CAR_KEY1_STRAIGHT_FOLLOW_KP, CAR_KEY1_STRAIGHT_FOLLOW_KI, CAR_KEY1_STRAIGHT_FOLLOW_KD);
            FollowLoop_SetKP(CAR_KEY1_CURVE_FOLLOW_KP);
            FollowLoop_SetKI(CAR_KEY1_CURVE_FOLLOW_KI);
            FollowLoop_SetKD(CAR_KEY1_CURVE_FOLLOW_KD);
            Mixer_SetBaseSpeed(g_semicircle_base_speed);
#endif
            OLED_Clear();
        }
        else if (KEY2_PRESSED())
        {
            g_run_mode = MODE_KEY2;
            g_mode_start_time = 0;  /* 等小车真正动起来再开始计时 */
            g_semicircle_base_speed = CAR_KEY2_SPEED;
            g_semicircle_speed_min  = CAR_KEY2_SPEED_MIN;
            g_semicircle_speed_max  = CAR_KEY2_SPEED_MAX;
#if USE_SEMICIRCLE_FOLLOW
            FollowLoop_SetCurvePID(CAR_KEY2_CURVE_FOLLOW_KP, CAR_KEY2_CURVE_FOLLOW_KI, CAR_KEY2_CURVE_FOLLOW_KD);
            FollowLoop_SetStraightPID(CAR_KEY2_STRAIGHT_FOLLOW_KP, CAR_KEY2_STRAIGHT_FOLLOW_KI, CAR_KEY2_STRAIGHT_FOLLOW_KD);
            FollowLoop_SetKP(CAR_KEY2_CURVE_FOLLOW_KP);
            FollowLoop_SetKI(CAR_KEY2_CURVE_FOLLOW_KI);
            FollowLoop_SetKD(CAR_KEY2_CURVE_FOLLOW_KD);
            Mixer_SetBaseSpeed(g_semicircle_base_speed);
#endif
            OLED_Clear();
        }
        else if (KEY3_PRESSED())
        {
            g_run_mode = MODE_KEY3;
            g_mode_start_time = 0;  /* 等小车真正动起来再开始计时 */
            g_semicircle_base_speed = CAR_KEY3_SPEED;
            g_semicircle_speed_min  = CAR_KEY3_SPEED_MIN;
            g_semicircle_speed_max  = CAR_KEY3_SPEED_MAX;
#if USE_SEMICIRCLE_FOLLOW
            FollowLoop_SetCurvePID(CAR_KEY3_CURVE_FOLLOW_KP, CAR_KEY3_CURVE_FOLLOW_KI, CAR_KEY3_CURVE_FOLLOW_KD);
            FollowLoop_SetStraightPID(CAR_KEY3_STRAIGHT_FOLLOW_KP, CAR_KEY3_STRAIGHT_FOLLOW_KI, CAR_KEY3_STRAIGHT_FOLLOW_KD);
            FollowLoop_SetKP(CAR_KEY3_CURVE_FOLLOW_KP);
            FollowLoop_SetKI(CAR_KEY3_CURVE_FOLLOW_KI);
            FollowLoop_SetKD(CAR_KEY3_CURVE_FOLLOW_KD);
            Mixer_SetBaseSpeed(g_semicircle_base_speed);
#endif
            OLED_Clear();
        }
        mspm0_delay_ms(10);
    }

    mspm0_delay_ms(500);    /* 按键消抖延时 */
}

void ButtonMode_UpdateTimeDisplay(uint32_t now)
{
    static uint32_t elapsed_frozen = 0;
    static uint32_t time_oled_last = 0;
    uint32_t elapsed;
    uint8_t oled_buffer[32];

    if (g_run_mode == MODE_DONE)
        elapsed = elapsed_frozen;
    else if (g_mode_start_time == 0)
        elapsed = 0;  /* 小车还没真正动起来，计时为0 */
    else {
        elapsed = now - g_mode_start_time;
        elapsed_frozen = elapsed;
    }
    if ((int32_t)(now - time_oled_last) >= 200)
    {
        time_oled_last = now;
        sprintf((char *)oled_buffer, "TIME: %u.%us",
                elapsed / 1000, (elapsed % 1000) / 100);
        OLED_ShowString(0, 4, oled_buffer, 16);
    }
}