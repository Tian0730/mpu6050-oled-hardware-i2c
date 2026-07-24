#ifndef __ULTRASONIC_CONTROL_H
#define __ULTRASONIC_CONTROL_H

#include "ti_msp_dl_config.h"

#define AVOID_DIST_THRESHOLD    200
#define AVOID_BACK_TIME_MS      500
#define AVOID_TURN_TIME_MS      600
#define AVOID_SPEED             300
#define AVOID_BACK_SPEED        250
#define AVOID_TURN_SPEED        250

typedef enum {
    AVOID_STATE_FORWARD = 0,
    AVOID_STATE_BACK,
    AVOID_STATE_TURN
} Ultrasonic_Avoid_State_t;

void Ultrasonic_Test_Display(uint8_t *oled_buffer);
void Ultrasonic_Avoidance_Init(void);
void Ultrasonic_Avoidance_Update(uint8_t *oled_buffer);

#endif