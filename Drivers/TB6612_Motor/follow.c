#include "ti_msp_dl_config.h"
#include "board.h"
#include "bsp_tb6612.h"
#include "follow.h"

/******************************************************************
 * 循迹控制（方案4：仅提供位置环PID修正量，不直接驱动电机）
 * 小偏差：比例控制微调
 * 大偏差：由主状态机处理（f1=右转触发，f8=左转触发）
 ******************************************************************/

//==================== 参数配置 ====================
#define KP              100     // 比例系数100
#define KI              0       // 积分系数（消除稳态误差）
#define KD              0       // 微分系数（抑制振动）
#define MAX_CORRECTION  100     // 最大修正量
#define MAX_INTEGRAL    100     // 积分限幅（防止积分饱和）

// 传感器状态
static uint8_t sensor_states[8] = {0};      // 黑：1    白：0
static uint8_t sensor_filtered[8] = {0};  // 滤波后的传感器状态
static uint32_t sensor_history[8] = {0};   // 传感器历史记录
#define SENSOR_FILTER_COUNT 3  // 连续3次相同状态才确认

static int16_t last_correction = 0;

// PID控制变量
static float last_bias = 0.0f;  // 上次偏差，用于计算微分项
static float integral = 0.0f;   // 积分累积项，用于消除稳态误差

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
            sensor_history[i]++;
            if (sensor_history[i] >= SENSOR_FILTER_COUNT) {
                sensor_filtered[i] = 1;
            }
        } else {
            sensor_history[i]--;
            if (sensor_history[i] <= -SENSOR_FILTER_COUNT) {
                sensor_filtered[i] = 0;
            }
        }
        sensor_states[i] = sensor_filtered[i];
    }
}

/******************************************************************
 * 计算黑线偏差
 * 返回：-3.5 ~ +3.5
 * 负数=偏左→右转，正数=偏右→左转
 ******************************************************************/
float IRDM_calculate_bias(void)
{
    int sum_weight = 0, sum_active = 0;
    const int weights[8] = {-35, -25, -15, -5, 5, 15, 25, 35};
    
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
void IRDM_UpdatePositionPID(void)
{
    float bias = IRDM_calculate_bias();

    if (bias > 100.0f || bias < -100.0f) {
        last_correction = 0;
        return;
    }

    integral += bias;
    if (integral > MAX_INTEGRAL) integral = MAX_INTEGRAL;
    if (integral < -MAX_INTEGRAL) integral = -MAX_INTEGRAL;

    float bias_diff = bias - last_bias;
    int correction = (int)(bias * KP + integral * KI + bias_diff * KD);

    last_bias = bias;

    if (correction > MAX_CORRECTION) correction = MAX_CORRECTION;
    if (correction < -MAX_CORRECTION) correction = -MAX_CORRECTION;

    last_correction = (int16_t)correction;
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
    for (int i = 2; i < 6; i++) {
        if (sensor_states[i]) return 1;
    }
    return 0;
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
 * 获取指定传感器状态
 ******************************************************************/
uint8_t IRDM_get_sensor_state(uint8_t index)
{
    if (index > 7) return 0;
    return sensor_states[index];
}
