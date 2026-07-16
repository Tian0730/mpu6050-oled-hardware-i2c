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
    lc_printf("[PID] AnglePD init: KP=%.2f KD=%.2f Limit=%.1f\r\n", kp, kd, limit);
}

float AnglePD_Update(AnglePD_t *pd, float target, float current)
{
    float error = target - current;

    /* 角度环绕处理：保证 error 在 [-180, 180] 范围内 */
    if (error > 180.0f)       error -= 360.0f;
    else if (error < -180.0f) error += 360.0f;

    float derivative = error - pd->last_error;
    pd->last_error = error;
    pd->output = pd->kp * error + pd->kd * derivative;
    pd->output = clamp(pd->output, pd->limit);
    return pd->output;
}

void AnglePD_Reset(AnglePD_t *pd)
{
    pd->last_error = 0.0f;
    pd->output = 0.0f;
}