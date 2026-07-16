#ifndef _TEST_H_
#define _TEST_H_

#include <stdint.h>

/* 返回值：0=执行中, 1=完成, -1=超时, -2=IMU未就绪 */
int8_t Test_Turn90_Start(void);
int8_t Test_Turn90_Poll(void);

int8_t Test_TurnToAngle_Start(float target_angle, float kp, float kd, uint32_t timeout_ms);
int8_t Test_TurnToAngle_Poll(void);

/* 获取当前测试状态信息（供 OLED 显示用） */
float Test_GetCurrentError(void);
float Test_GetCurrentOutput(void);
float Test_GetTargetAngle(void);
uint32_t Test_GetElapsedMs(void);

#endif