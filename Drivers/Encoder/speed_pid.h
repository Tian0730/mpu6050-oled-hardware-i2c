#ifndef _SPEED_PID_H_
#define _SPEED_PID_H_

#include <stdint.h>

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral_limit;
    float output_limit;
    float last_error;
    float integral;
    float output;
    /// === VOFA+ 新增字段 ===
    float p_term;
    float i_term;
    float d_term;
    float current_error;
    /// === VOFA+ 新增字段 END ===
} SpeedPID_t;

void SpeedPID_Init(SpeedPID_t *pid, float kp, float ki, float kd,
                   float integral_limit, float output_limit);
float SpeedPID_Update(SpeedPID_t *pid, float target, float current, float dt);
void SpeedPID_Reset(SpeedPID_t *pid);

/// === VOFA+ 新增接口 ===
float SpeedPID_GetPTerm(const SpeedPID_t *pid);
float SpeedPID_GetITerm(const SpeedPID_t *pid);
float SpeedPID_GetDTerm(const SpeedPID_t *pid);
float SpeedPID_GetError(const SpeedPID_t *pid);
float SpeedPID_GetOutput(const SpeedPID_t *pid);
void  SpeedPID_SetKP(SpeedPID_t *pid, float kp);
void  SpeedPID_SetKI(SpeedPID_t *pid, float ki);
void  SpeedPID_SetKD(SpeedPID_t *pid, float kd);
/// === VOFA+ 新增接口 END ===

#endif