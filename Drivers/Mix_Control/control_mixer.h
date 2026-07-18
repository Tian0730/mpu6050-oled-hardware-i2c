#ifndef _CONTROL_MIXER_H_
#define _CONTROL_MIXER_H_

#include <stdint.h>

void Mixer_Init(uint32_t base_speed);
void Mixer_SetAngleDiff(int16_t diff);
void Mixer_SetFollowDiff(int16_t diff);
void Mixer_SetSpeedDiff(int16_t diff);    // 预留速度环
void Mixer_Apply(void);
void Mixer_Reset(void);

#endif