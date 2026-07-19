#ifndef	__BSP_MOTOR_HALLENCODER_H__
#define __BSP_MOTOR_HALLENCODER_H__

#include "board.h"

// 获得绝对值
#define ABS(a)      (a>0 ? a:(-a))

typedef struct{
    int Should_Get_Encoder_Count;   // 将要获得的编码器计数
    int Obtained_Get_Encoder_Count; // 得到的编码器的计数
}Encoder;


void Motor_Init(void);
int Motor_Get_Encoder(int dir);
void Motor_Set_PWM(int pwma,int pwmb);
void Motor_Stop(void);

extern volatile uint8_t g_encoder_new_data;

#endif