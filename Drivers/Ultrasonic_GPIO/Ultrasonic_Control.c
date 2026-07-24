#include "Ultrasonic_Control.h"
#include "ultrasonic_gpio.h"
#include "oled_hardware_i2c.h"
#include "bsp_tb6612.h"
#include "clock.h"
#include "stdio.h"

static Ultrasonic_Avoid_State_t avoid_state = AVOID_STATE_FORWARD;
static unsigned long avoid_deadline = 0;
static uint8_t  avoid_turn_dir = 0;

void Ultrasonic_Test_Display(uint8_t *oled_buffer)
{
    OLED_ShowString(0, 0, (uint8_t *)"Dist:", 16);

    uint16_t distVal = Read_Ultrasonic();
    sprintf((char *)oled_buffer, "%4u", distVal);
    OLED_ShowString(6 * 8, 0, oled_buffer, 16);
}

void Ultrasonic_Avoidance_Init(void)
{
    Ultrasonic_Init();
    avoid_state = AVOID_STATE_FORWARD;
    avoid_deadline = 0;
    avoid_turn_dir = 0;
}

void Ultrasonic_Avoidance_Update(uint8_t *oled_buffer)
{
    uint16_t distance = Read_Ultrasonic();
    unsigned long now;
    mspm0_get_clock_ms(&now);

    switch (avoid_state)
    {
    case AVOID_STATE_FORWARD:
        OLED_ShowString(0, 0, (uint8_t *)"Dist:", 16);
        sprintf((char *)oled_buffer, "%4u mm", distance);
        OLED_ShowString(6 * 8, 0, oled_buffer, 16);
        OLED_ShowString(0, 2, (uint8_t *)"GO  ", 16);

        if (distance > 0 && distance < AVOID_DIST_THRESHOLD)
        {
            avoid_state = AVOID_STATE_BACK;
            avoid_deadline = now + AVOID_BACK_TIME_MS;
            TB6612_Motor_Stop();
        }
        else
        {
            AO_Control(1, AVOID_SPEED);
            BO_Control(1, AVOID_SPEED);
        }
        break;

    case AVOID_STATE_BACK:
        OLED_ShowString(0, 0, (uint8_t *)"Dist:", 16);
        sprintf((char *)oled_buffer, "%4u mm", distance);
        OLED_ShowString(6 * 8, 0, oled_buffer, 16);
        OLED_ShowString(0, 2, (uint8_t *)"BACK", 16);

        if ((long)(now - avoid_deadline) >= 0)
        {
            avoid_state = AVOID_STATE_TURN;
            avoid_deadline = now + AVOID_TURN_TIME_MS;
            avoid_turn_dir ^= 1;
            TB6612_Motor_Stop();
        }
        else
        {
            AO_Control(0, AVOID_BACK_SPEED);
            BO_Control(0, AVOID_BACK_SPEED);
        }
        break;

    case AVOID_STATE_TURN:
        OLED_ShowString(0, 0, (uint8_t *)"Dist:", 16);
        sprintf((char *)oled_buffer, "%4u mm", distance);
        OLED_ShowString(6 * 8, 0, oled_buffer, 16);
        OLED_ShowString(0, 2, (uint8_t *)"TURN", 16);

        if ((int32_t)(now - avoid_deadline) >= 0)
        {
            avoid_state = AVOID_STATE_FORWARD;
            TB6612_Motor_Stop();
        }
        else
        {
            if (avoid_turn_dir == 0)
            {
                AO_Control(1, AVOID_TURN_SPEED);
                BO_Control(0, AVOID_TURN_SPEED);
            }
            else
            {
                AO_Control(0, AVOID_TURN_SPEED);
                BO_Control(1, AVOID_TURN_SPEED);
            }
        }
        break;
    }
}