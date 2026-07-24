#include "angle_pid.h"
#include "board.h"
#include <math.h>

static float clamp(float v, float limit)
{
    if (v > limit)  return limit;
    if (v < -limit) return -limit;
    return v;
}

void AnglePD_Init(AnglePD_t *pd, float kp, float kd, float limit)
{
    pd->kp = kp;
    pd->kd = kd;
    pd->limit = limit;
    pd->last_error = 0.0f;
    pd->output = 0.0f;
    /// === VOFA+ 新增初始化 ===
    pd->p_term = 0.0f;
    pd->d_term = 0.0f;
    pd->current_error = 0.0f;
    /// === VOFA+ 新增初始化 END ===
    lc_printf("[PID] AnglePD init: KP=%.2f KD=%.2f Limit=%.1f\r\n", kp, kd, limit);
}

float AnglePD_Update(AnglePD_t *pd, float target, float current)
{
    float error = target - current;

    while (error > 180.0f)  error -= 360.0f;
    while (error < -180.0f) error += 360.0f;

    float derivative = error - pd->last_error;
    pd->last_error = error;
    /// === VOFA+ 修改：分开存储 P/D 分量便于调试 ===
    pd->current_error = error;
    pd->p_term = pd->kp * error;
    pd->d_term = pd->kd * derivative;
    pd->output = pd->p_term + pd->d_term;
    /// === VOFA+ 修改 END ===
    pd->output = clamp(pd->output, pd->limit);
    return pd->output;
}

void AnglePD_Reset(AnglePD_t *pd)
{
    pd->last_error = 0.0f;
    pd->output = 0.0f;
    /// === VOFA+ 新增重置 ===
    pd->p_term = 0.0f;
    pd->d_term = 0.0f;
    pd->current_error = 0.0f;
    /// === VOFA+ 新增重置 END ===
}

/// === VOFA+ 新增 getter/setter ===
float AnglePD_GetPTerm(const AnglePD_t *pd)  { return pd->p_term; }
float AnglePD_GetDTerm(const AnglePD_t *pd)  { return pd->d_term; }
float AnglePD_GetError(const AnglePD_t *pd)  { return pd->current_error; }
float AnglePD_GetOutput(const AnglePD_t *pd) { return pd->output; }
void  AnglePD_SetKP(AnglePD_t *pd, float kp) { pd->kp = kp; }
void  AnglePD_SetKD(AnglePD_t *pd, float kd) { pd->kd = kd; }
/// === VOFA+ 新增 getter/setter END ===