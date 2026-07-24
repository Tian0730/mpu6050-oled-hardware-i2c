#ifndef _FOLLOW_H_
#define _FOLLOW_H_

#include <stdint.h>
#include "ti_msp_dl_config.h"
#include "board.h"
#include "bsp_tb6612.h"

// // 循线状态枚举
// typedef enum {
//     LINE_STRAIGHT = 0,    // 直行
//     LINE_LEFT_SMALL,      // 小左转
//     LINE_LEFT_BIG,        // 大左转
//     LINE_RIGHT_SMALL,     // 小右转
//     LINE_RIGHT_BIG,       // 大右转
//     LINE_LOST             // 丢失线路
// } LineStatus;

void     IRDM_read_sensors(void);
float    IRDM_calculate_bias(void);
void     IRDM_UpdatePositionPID(float dt);
int16_t  IRDM_GetCorrection(void);
uint8_t  IRDM_IsBlackLine(void);
uint8_t  IRDM_NeedTurnLeft(void);
uint8_t  IRDM_NeedTurnRight(void);
uint8_t  IRDM_NeedTurnLeftFast(void);
uint8_t  IRDM_NeedTurnRightFast(void);
uint8_t  IRDM_get_sensor_state(uint8_t index);

float FollowLoop_GetBias(void);
float FollowLoop_GetPTerm(void);
float FollowLoop_GetITerm(void);
float FollowLoop_GetDTerm(void);
void  FollowLoop_SetKP(float kp);
void  FollowLoop_SetKI(float ki);
void  FollowLoop_SetKD(float kd);

#endif  /* _FOLLOW_H_ */