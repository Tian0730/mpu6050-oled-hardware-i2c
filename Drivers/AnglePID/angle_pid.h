#ifndef _ANGLE_PID_H_
#define _ANGLE_PID_H_

typedef struct {
    float kp;
    float kd;
    float limit;
    float last_error;
    float output;
} AnglePD_t;

void AnglePD_Init(AnglePD_t *pd, float kp, float kd, float limit);
float AnglePD_Update(AnglePD_t *pd, float target, float current);
void AnglePD_Reset(AnglePD_t *pd);

#endif