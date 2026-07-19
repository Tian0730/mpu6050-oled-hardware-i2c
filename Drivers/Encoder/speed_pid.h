#ifndef __SPEED_PID_H__
#define __SPEED_PID_H__

typedef struct {
    float kp;               // 比例系数
    float ki;               // 积分系数
    float kd;               // 微分系数
    float integral_limit;   // 积分限幅（抗积分饱和）
    float output_limit;     // 输出限幅
    float last_error;       // 上次误差（用于微分计算）
    float integral;         // 积分累加值
    float output;           // 当前输出值
} SpeedPID_t;

void SpeedPID_Init(SpeedPID_t *pid, float kp, float ki, float kd,
                   float integral_limit, float output_limit);
float SpeedPID_Update(SpeedPID_t *pid, float target, float current, float dt);
void SpeedPID_Reset(SpeedPID_t *pid);

#endif /* __SPEED_PID_H__ */