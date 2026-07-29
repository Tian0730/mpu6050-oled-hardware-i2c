#include "ti_msp_dl_config.h"
#include "board.h"
#include "bsp_tb6612.h"
#include "follow.h"
#include "car_config.h"

/******************************************************************
 * 循迹控制（仅提供位置环PID修正量，不直接驱动电机）
 * 小偏差：比例控制微调
 * 大偏差：由主状态机处理（f1=右转触发，f8=左转触发）
 ******************************************************************/

//==================== 灰度位置环PID相关参数 ====================
static float g_follow_kp = CAR_FOLLOW_KP;
static float g_follow_ki = CAR_FOLLOW_KI;
static float g_follow_kd = CAR_FOLLOW_KD;
#define  MAX_CORRECTION  CAR_FOLLOW_MAX_CORRECTION
#define  MAX_INTEGRAL    CAR_FOLLOW_MAX_INTEGRAL

// 直道/弯道双 PID 备份
static float g_kp_straight = CAR_SEMICIRCLE_STRAIGHT_FOLLOW_KP;
static float g_ki_straight = CAR_SEMICIRCLE_STRAIGHT_FOLLOW_KI;
static float g_kd_straight = CAR_SEMICIRCLE_STRAIGHT_FOLLOW_KD;
static float g_kp_curve    = CAR_SEMICIRCLE_CURVE_FOLLOW_KP;
static float g_ki_curve    = CAR_SEMICIRCLE_CURVE_FOLLOW_KI;
static float g_kd_curve    = CAR_SEMICIRCLE_CURVE_FOLLOW_KD;

// 传感器状态
static uint8_t sensor_states[8] = {0};      // 黑：1    白：0
static uint8_t sensor_filtered[8] = {0};    // 滤波后的传感器状态
static int32_t sensor_history[8] = {0};    // 传感器历史记录
#define SENSOR_FILTER_COUNT 3  // 连续3次相同状态才确认

// // 外侧传感器独立消抖（2次确认，比主滤波更快）
// static uint8_t turn_f1_filtered = 0;
// static uint8_t turn_f8_filtered = 0;
// #define TURN_FILTER_COUNT 2

static int16_t last_correction = 0;

// PID控制变量
static float last_bias = 0.0f;  // 上次偏差，用于计算微分项
static float integral = 0.0f;   // 积分累积项，用于消除稳态误差

static float follow_p_term = 0.0f;
static float follow_i_term = 0.0f;
static float follow_d_term = 0.0f;
static float follow_bias   = 0.0f;
static float follow_bias_filt = 0.0f;
static float follow_d_filt = 0.0f;
static float follow_bias_deriv = 0.0f;

/******************************************************************
 * 读取传感器状态
 ******************************************************************/
void IRDM_read_sensors(void)
{
    uint8_t raw_states[8];
    
    // 读取原始状态
    raw_states[0] = 1 - !(DL_GPIO_readPins(FOLLOW_f1_PORT, FOLLOW_f1_PIN));
    raw_states[1] = 1 - !(DL_GPIO_readPins(FOLLOW_f2_PORT, FOLLOW_f2_PIN));
    raw_states[2] = 1 - !(DL_GPIO_readPins(FOLLOW_f3_PORT, FOLLOW_f3_PIN));
    raw_states[3] = 1 - !(DL_GPIO_readPins(FOLLOW_f4_PORT, FOLLOW_f4_PIN));
    raw_states[4] = 1 - !(DL_GPIO_readPins(FOLLOW_f5_PORT, FOLLOW_f5_PIN));
    raw_states[5] = 1 - !(DL_GPIO_readPins(FOLLOW_f6_PORT, FOLLOW_f6_PIN));
    raw_states[6] = 1 - !(DL_GPIO_readPins(FOLLOW_f7_PORT, FOLLOW_f7_PIN));
    raw_states[7] = 1 - !(DL_GPIO_readPins(FOLLOW_f8_PORT, FOLLOW_f8_PIN));
    
    // 传感器状态滤波：连续多次相同状态才确认
    for (int i = 0; i < 8; i++) {
        if (raw_states[i]) {
            if (sensor_history[i] < SENSOR_FILTER_COUNT)  // ← 上限锁死
                sensor_history[i]++;
            if (sensor_history[i] >= SENSOR_FILTER_COUNT) {
                sensor_filtered[i] = 1;
            }
        } else {
            if (sensor_history[i] > -SENSOR_FILTER_COUNT) // ← 下限锁死
                sensor_history[i]--;
            if (sensor_history[i] <= -SENSOR_FILTER_COUNT) {
                sensor_filtered[i] = 0;
            }
        }
        sensor_states[i] = sensor_filtered[i];
    }
    // // 外侧传感器 f1/f8 独立消抖（2次确认，确保转弯检测灵敏）
    // {
    //     static int8_t f1_cnt = 0, f8_cnt = 0;
    //     if (raw_states[0]) { if (f1_cnt <  TURN_FILTER_COUNT) f1_cnt++; }
    //     else               { if (f1_cnt > -TURN_FILTER_COUNT) f1_cnt--; }
    //     turn_f1_filtered = (f1_cnt >= TURN_FILTER_COUNT) ? 1 : 0;

    //     if (raw_states[7]) { if (f8_cnt <  TURN_FILTER_COUNT) f8_cnt++; }
    //     else               { if (f8_cnt > -TURN_FILTER_COUNT) f8_cnt--; }
    //     turn_f8_filtered = (f8_cnt >= TURN_FILTER_COUNT) ? 1 : 0;
    // }
}

/******************************************************************
 * 计算黑线偏差
 * 返回：-3.5 ~ +3.5
 * 负数=偏左→右转，正数=偏右→左转
 ******************************************************************/
float IRDM_calculate_bias(void)
{
    int sum_weight = 0, sum_active = 0;
    const int weights[8] = {-30, -25, -20, -5, 5, 20, 25, 30};
    
    for (int i = 0; i < 8; i++) {
        if (sensor_states[i]) {
            sum_weight += weights[i];
            sum_active++;
        }
    }
    
    if (sum_active == 0) return 999.0f;
    return (float)sum_weight / (sum_active * 10.0f);
}

/******************************************************************
 * 位置环PID更新（仅计算修正量，不驱动电机）
 ******************************************************************/
void IRDM_UpdatePositionPID(float dt)
{
    float bias_raw = IRDM_calculate_bias();

    if (bias_raw > 100.0f || bias_raw < -100.0f) {
        last_correction = 0;
        return;
    }

    /* 对离散灰度偏差做一阶低通，降低弯道边缘抖动 */
    follow_bias_filt += (bias_raw - follow_bias_filt) * CAR_FOLLOW_BIAS_LPF_ALPHA;
    float bias = follow_bias_filt;

    /* 积分项：每帧累加偏差，频率越高积分越快（配合 KI 调参） */
    integral += bias;
    if (integral > MAX_INTEGRAL) integral = MAX_INTEGRAL;
    if (integral < -MAX_INTEGRAL) integral = -MAX_INTEGRAL;

    /* 微分项：除以 dt 得到真正的变化率 */
    float bias_diff = 0.0f;
    if (dt > 0.0001f) {
        bias_diff = (bias - last_bias) / dt;
    }
    /* 微分项低通，避免传感器量化带来的 D 抖动 */
    follow_d_filt += (bias_diff - follow_d_filt) * CAR_FOLLOW_DTERM_LPF_ALPHA;
    follow_bias_deriv = follow_d_filt;
    last_bias = bias;

    follow_bias   = bias;
    follow_p_term = bias * g_follow_kp;
    follow_i_term = integral * g_follow_ki;
    follow_d_term = follow_d_filt * g_follow_kd;
    int correction = (int)(follow_p_term + follow_i_term + follow_d_term);

    if (correction > MAX_CORRECTION) correction = MAX_CORRECTION;
    if (correction < -MAX_CORRECTION) correction = -MAX_CORRECTION;

    /* 修正量斜率限制，避免进入/退出弯道时左右轮突变 */
    if (dt > 0.0001f)
    {
        float max_step_f = CAR_FOLLOW_CORR_SLEW_PER_SEC * dt;
        int16_t max_step = (int16_t)((max_step_f < 1.0f) ? 1.0f : max_step_f);
        int16_t prev = last_correction;
        int16_t delta = (int16_t)correction - prev;
        if (delta > max_step) delta = max_step;
        if (delta < -max_step) delta = -max_step;
        last_correction = prev + delta;
    }
    else
    {
        last_correction = (int16_t)correction;
    }
}

/******************************************************************
 * 获取位置环修正量
 ******************************************************************/
int16_t IRDM_GetCorrection(void)
{
    return -last_correction;
}

/******************************************************************
 * 是否检测到黑线（任意传感器触发即返回1）
 ******************************************************************/
uint8_t IRDM_IsBlackLine(void)
{
    for (int i = 0; i < 8; i++) {
        if (sensor_states[i]) return 1;
    }
    return 0;
}

/******************************************************************
 * 是否全部传感器同时检测到黑线（启停线检测）
 * 8 路全部触发 → 垂直线段（启停线）
 ******************************************************************/
uint8_t IRDM_IsAllBlack(void)
{
    for (int i = 0; i < 8; i++) {
        if (!sensor_states[i]) return 0;
    }
    return 1;
}

/******************************************************************
 * 统计当前检测到黑线的传感器数量
 ******************************************************************/
uint8_t IRDM_CountBlackSensors(void)
{
    uint8_t cnt = 0;
    for (int i = 0; i < 8; i++) {
        if (sensor_states[i]) cnt++;
    }
    return cnt;
}

/******************************************************************
 * 是否需要左转（f8检测到黑线）
 ******************************************************************/
uint8_t IRDM_NeedTurnLeft(void)
{
    return sensor_states[7];
}

/******************************************************************
 * 是否需要右转（f1检测到黑线）
 ******************************************************************/
uint8_t IRDM_NeedTurnRight(void)
{
    return sensor_states[0];
}

/******************************************************************
 * 是否需要左转（f8 原始状态，快速识别）
 ******************************************************************/
uint8_t IRDM_NeedTurnLeftFast(void)
{
    return (uint8_t)(1 - !(DL_GPIO_readPins(FOLLOW_f8_PORT, FOLLOW_f8_PIN)));
}

/******************************************************************
 * 是否需要右转（f1 原始状态，快速识别）
 ******************************************************************/
uint8_t IRDM_NeedTurnRightFast(void)
{
    return (uint8_t)(1 - !(DL_GPIO_readPins(FOLLOW_f1_PORT, FOLLOW_f1_PIN)));
}

/******************************************************************
 * 获取指定传感器状态
 ******************************************************************/
uint8_t IRDM_get_sensor_state(uint8_t index)
{
    if (index > 7) return 0;
    return sensor_states[index];
}

float FollowLoop_GetBias(void)  { return follow_bias; }
float FollowLoop_GetBiasDeriv(void) { return follow_bias_deriv; }
float FollowLoop_GetPTerm(void) { return follow_p_term; }
float FollowLoop_GetITerm(void) { return follow_i_term; }
float FollowLoop_GetDTerm(void) { return follow_d_term; }
void  FollowLoop_SetKP(float kp) { g_follow_kp = kp; lc_printf("[VOFA] Follow KP=%.2f\r\n", kp); }
void  FollowLoop_SetKI(float ki) { g_follow_ki = ki; lc_printf("[VOFA] Follow KI=%.2f\r\n", ki); }
void  FollowLoop_SetKD(float kd) { g_follow_kd = kd; lc_printf("[VOFA] Follow KD=%.2f\r\n", kd); }

void FollowLoop_ResetIntegral(void)
{
    integral  = 0.0f;
    last_bias = 0.0f;
    follow_bias_filt = 0.0f;
    follow_d_filt = 0.0f;
}

void FollowLoop_DecayIntegral(float decay)
{
    integral *= decay;
}

/*
 *  直道/弯道 PID 切换
 *
 *  VOFA 兼容：切换时先保存当前 PID 到对应备份，再加载另一套。
 *  这样 VOFA 实时调参的值不会因为切模式而丢失。
 */
void FollowLoop_SwitchToStraight(void)
{
    g_kp_curve = g_follow_kp;
    g_ki_curve = g_follow_ki;
    g_kd_curve = g_follow_kd;
    g_follow_kp = g_kp_straight;
    g_follow_ki = g_ki_straight;
    g_follow_kd = g_kd_straight;
}

void FollowLoop_SwitchToCurve(void)
{
    g_kp_straight = g_follow_kp;
    g_ki_straight = g_follow_ki;
    g_kd_straight = g_follow_kd;
    g_follow_kp = g_kp_curve;
    g_follow_ki = g_ki_curve;
    g_follow_kd = g_kd_curve;
}