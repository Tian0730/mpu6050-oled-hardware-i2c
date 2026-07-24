#include "speed_pid.h"
#include "board.h"
#include <math.h>

static float clamp(float v, float limit)
{
    if (v > limit)  return limit;
    if (v < -limit) return -limit;
    return v;
}

/******************************************************************
 * 函 数 名 称：SpeedPID_Init
 * 函 数 说 明：PID 控制器初始化
 * 函 数 形 参：pid-PID结构体指针 kp/ki/kd-PID系数
 *              integral_limit-积分限幅 output_limit-输出限幅
 * 函 数 返 回：无
 * 作       者：tian
 * 备       注：无
******************************************************************/
void SpeedPID_Init(SpeedPID_t *pid, float kp, float ki, float kd,
                   float integral_limit, float output_limit)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral_limit = integral_limit;
    pid->output_limit = output_limit;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
    pid->p_term = 0.0f;
    pid->i_term = 0.0f;
    pid->d_term = 0.0f;
    pid->current_error = 0.0f;
    lc_printf("[PID] SpeedPID init: KP=%.2f KI=%.3f KD=%.2f ILim=%.1f OLim=%.1f\r\n",
              kp, ki, kd, integral_limit, output_limit);
}

/******************************************************************
 * 函 数 名 称：SpeedPID_Update
 * 函 数 说 明：PID 控制器更新计算
 * 函 数 形 参：pid-PID结构体指针 target-目标值 current-当前值 dt-时间间隔(秒)
 * 函 数 返 回：PID输出值
 * 作       者：tian
 * 备       注：1.积分项带抗积分饱和
 *              2.微分项使用"误差微分"方式，避免设定值突变时的微分冲击
 *              3.输出带限幅保护
******************************************************************/
float SpeedPID_Update(SpeedPID_t *pid, float target, float current, float dt)
{
    float error = target - current;

    /* 积分项（带抗积分饱和） */
    pid->integral += error * dt;
    pid->integral = clamp(pid->integral, pid->integral_limit);

    /* 微分项（误差微分，避免设定值突变产生微分冲击） */
    float derivative = 0.0f;
    if (dt > 0.0001f) {
        derivative = (error - pid->last_error) / dt;
    }
    pid->last_error = error;

    pid->current_error = error;
    pid->p_term = pid->kp * error;
    pid->i_term = pid->ki * pid->integral;
    pid->d_term = pid->kd * derivative;
    pid->output = pid->p_term + pid->i_term + pid->d_term;
    pid->output = clamp(pid->output, pid->output_limit);

    return pid->output;
}

/******************************************************************
 * 函 数 名 称：SpeedPID_Reset
 * 函 数 说 明：重置 PID 控制器状态
 * 函 数 形 参：pid-PID结构体指针
 * 函 数 返 回：无
 * 作       者：tian
 * 备       注：清除积分累加和误差历史
******************************************************************/
void SpeedPID_Reset(SpeedPID_t *pid)
{
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
    pid->p_term = 0.0f;
    pid->i_term = 0.0f;
    pid->d_term = 0.0f;
    pid->current_error = 0.0f;
}

float SpeedPID_GetPTerm(const SpeedPID_t *pid)  { return pid->p_term; }
float SpeedPID_GetITerm(const SpeedPID_t *pid)  { return pid->i_term; }
float SpeedPID_GetDTerm(const SpeedPID_t *pid)  { return pid->d_term; }
float SpeedPID_GetError(const SpeedPID_t *pid)  { return pid->current_error; }
float SpeedPID_GetOutput(const SpeedPID_t *pid) { return pid->output; }
void  SpeedPID_SetKP(SpeedPID_t *pid, float kp) { pid->kp = kp; }
void  SpeedPID_SetKI(SpeedPID_t *pid, float ki) { pid->ki = ki; }
void  SpeedPID_SetKD(SpeedPID_t *pid, float kd) { pid->kd = kd; }