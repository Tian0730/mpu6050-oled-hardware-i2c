#ifndef _VOFA_DEBUG_H_
#define _VOFA_DEBUG_H_

#include <stdint.h>

/* ================================================================
 *  VOFA+ 调试模式选择
 *    0 = 速度环调试
 *    1 = 角度环调试
 *    2 = 位置环(循迹)调试
 * ================================================================ */
#define VOFA_MODE  2

extern uint8_t g_vofa_stop;

void VOFA_Init(void);
void VOFA_Output(void);
void VOFA_ReceivePoll(void);

#endif