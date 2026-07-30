#ifndef _BUTTON_MODE_H_
#define _BUTTON_MODE_H_

#include <stdint.h>

/* ================================================================
 *  按键模式定义
 * ================================================================ */
typedef enum {
    MODE_WAITING = 0,
    MODE_KEY1,
    MODE_KEY2,
    MODE_KEY3,
    MODE_DONE,
} run_mode_t;

/* 全局状态变量 */
extern run_mode_t g_run_mode;
extern uint32_t   g_mode_start_time;
extern uint32_t   g_semicircle_base_speed;
extern uint32_t   g_semicircle_speed_min;
extern uint32_t   g_semicircle_speed_max;

void ButtonMode_Select(void);
void ButtonMode_UpdateTimeDisplay(uint32_t now);

#endif /* _BUTTON_MODE_H_ */