#ifndef _ANGLE_PID_H_
#define _ANGLE_PID_H_

#include <stdint.h>

typedef struct {
    float kp;
    float kd;
    float limit;
    float last_error;
    float output;
    /// === VOFA+ 新增字段 ===
    float p_term;           // 比例分量
    float d_term;           // 微分分量
    float current_error;    // 当前误差
    /// === VOFA+ 新增字段 END ===
} AnglePD_t;

void AnglePD_Init(AnglePD_t *pd, float kp, float kd, float limit);
float AnglePD_Update(AnglePD_t *pd, float target, float current);
void AnglePD_Reset(AnglePD_t *pd);

/// === VOFA+ 新增接口 ===
float AnglePD_GetPTerm(const AnglePD_t *pd);
float AnglePD_GetDTerm(const AnglePD_t *pd);
float AnglePD_GetError(const AnglePD_t *pd);
float AnglePD_GetOutput(const AnglePD_t *pd);
void  AnglePD_SetKP(AnglePD_t *pd, float kp);
void  AnglePD_SetKD(AnglePD_t *pd, float kd);
/// === VOFA+ 新增接口 END ===

#endif