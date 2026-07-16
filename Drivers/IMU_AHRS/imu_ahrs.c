#include "ti_msp_dl_config.h"
#include "imu_ahrs.h"
// #include "oled_hardware_i2c.h"
#include "mspm0_i2c.h"
#include "clock.h"
#include <math.h>
// #include <stdio.h>

/* ================================================================
 *  MPU6050 内部寄存器定义
 * ================================================================ */
#define MPU6050_REG_SMPLRT_DIV    0x19U
#define MPU6050_REG_CONFIG        0x1AU
#define MPU6050_REG_GYRO_CONFIG   0x1BU
#define MPU6050_REG_ACCEL_CONFIG  0x1CU
#define MPU6050_REG_PWR_MGMT_1    0x6BU
#define MPU6050_REG_WHO_AM_I      0x75U
#define MPU6050_REG_ACCEL_XOUT_H  0x3BU

#define MPU6050_RESET_CMD         0x80U
#define MPU6050_WAKEUP_CMD        0x00U

/* DLPF 42Hz, 2000dps, ±4g */
#define MPU6050_CONFIG_42HZ       0x03U
#define MPU6050_GYRO_2000DPS      0x18U
#define MPU6050_ACCEL_4G          0x08U

// extern uint8_t oled_buffer[32];

/* ================================================================
 *  内部状态机
 * ================================================================ */
typedef enum {
    INIT_STATE_IDLE = 0,
    INIT_STATE_RESET_WAIT,
    INIT_STATE_CONFIG,
    INIT_STATE_CALIB_SAMPLE,
    INIT_STATE_DONE,
    INIT_STATE_ERROR
} InitState_t;

typedef enum {
    YAW_CALIB_IDLE = 0,
    YAW_CALIB_COLLECTING,
    YAW_CALIB_DONE
} YawCalibState_t;

/* ================================================================
 *  静态全局变量
 * ================================================================ */
static IMU_AHRS_Data_t g_imu;
static IMU_AHRS_Status_t g_status = IMU_AHRS_STATUS_NOT_INIT;
static InitState_t g_init_state = INIT_STATE_IDLE;
static uint32_t g_init_deadline = 0;
static uint32_t g_init_start_time = 0;

/* 陀螺仪校准 */
static uint16_t g_calib_sample_idx = 0;
static int32_t g_calib_sum_x = 0;
static int32_t g_calib_sum_y = 0;
static int32_t g_calib_sum_z = 0;
static uint8_t g_calib_burst[14];

/* Mahony 积分反馈 */
static float g_integral_fb_x = 0.0f;
static float g_integral_fb_y = 0.0f;
static float g_integral_fb_z = 0.0f;

/* Yaw 漂移校准 */
static YawCalibState_t g_yaw_calib_state = YAW_CALIB_IDLE;
static uint32_t g_yaw_calib_count = 0;
static double g_yaw_sum_t = 0.0, g_yaw_sum_y = 0.0;
static double g_yaw_sum_t2 = 0.0, g_yaw_sum_ty = 0.0;
static float g_yaw_drift_k = 0.0f;
static uint32_t g_yaw_start_time = 0;
static uint32_t g_yaw_calib_start_time = 0;
static uint8_t g_yaw_enable = 0;

/* 转弯角度（陀螺仪 Z 轴积分，短时高精度） */
static float g_turn_angle = 0.0f;

/* ================================================================
 *  快速平方根倒数（Quake III 算法）
 *  用于四元数/向量归一化，比标准 sqrtf 快约 3 倍
 * ================================================================ */
static float imu_ahrs_inv_sqrt(float x)
{
    union { float f; uint32_t i; } conv;
    float xhalf = 0.5f * x;
    conv.f = x;
    conv.i = 0x5f3759dfU - (conv.i >> 1);
    conv.f = conv.f * (1.5f - (xhalf * conv.f * conv.f));
    return conv.f;
}

/* ================================================================
 *  工具函数：拼接两个字节为 int16_t（大端）
 * ================================================================ */
static int16_t imu_ahrs_make_s16(uint8_t high, uint8_t low)
{
    return (int16_t)(((uint16_t)high << 8U) | (uint16_t)low);
}

/* ================================================================
 *  I2C 读写封装（已改为项目现有 mspm0_i2c 接口）
 * ================================================================ */
static int32_t imu_ahrs_write_reg(uint8_t reg, uint8_t data)
{
    return mspm0_i2c_write(IMU_AHRS_I2C_ADDR, reg, 1, &data);
}

static int32_t imu_ahrs_read_multi(uint8_t reg, uint8_t *buf, uint16_t len)
{
    return mspm0_i2c_read(IMU_AHRS_I2C_ADDR, reg, len, buf);
}

/* ================================================================
 *  陀螺仪零偏校准：采集单次样本
 *  算法：连续采集 N 个静止样本，对三轴陀螺仪数据求算术平均
 * ================================================================ */
static int32_t imu_ahrs_calibrate_gyro_offset_step(void)
{
    if (imu_ahrs_read_multi(MPU6050_REG_ACCEL_XOUT_H, g_calib_burst, 14U) != 0)
        return -1;

    /* 陀螺仪数据在 burst[8..13] */
    g_calib_sum_x += (int32_t)imu_ahrs_make_s16(g_calib_burst[8],  g_calib_burst[9]);
    g_calib_sum_y += (int32_t)imu_ahrs_make_s16(g_calib_burst[10], g_calib_burst[11]);
    g_calib_sum_z += (int32_t)imu_ahrs_make_s16(g_calib_burst[12], g_calib_burst[13]);
    g_calib_sample_idx++;

    if (g_calib_sample_idx >= IMU_AHRS_CALIB_SAMPLES)
    {
        g_imu.gx_offset = (int16_t)(g_calib_sum_x / (int32_t)IMU_AHRS_CALIB_SAMPLES);
        g_imu.gy_offset = (int16_t)(g_calib_sum_y / (int32_t)IMU_AHRS_CALIB_SAMPLES);
        g_imu.gz_offset = (int16_t)(g_calib_sum_z / (int32_t)IMU_AHRS_CALIB_SAMPLES);
    }
    return 0;
}

/* ================================================================
 *  初始化：启动（非阻塞）
 * ================================================================ */
void IMU_AHRS_Init_Start(void)
{
    if (g_status == IMU_AHRS_STATUS_INIT_BUSY) return;

    /* 初始化四元数为单位四元数 */
    g_imu.q0 = 1.0f; g_imu.q1 = 0.0f; g_imu.q2 = 0.0f; g_imu.q3 = 0.0f;
    g_imu.pitch = 0.0f; g_imu.roll = 0.0f; g_imu.yaw = 0.0f;

    /* 清零积分反馈 */
    g_integral_fb_x = 0.0f; g_integral_fb_y = 0.0f; g_integral_fb_z = 0.0f;

    /* 清零校准状态 */
    g_calib_sample_idx = 0;
    g_calib_sum_x = 0; g_calib_sum_y = 0; g_calib_sum_z = 0;

    /* 复位 MPU6050 */
    imu_ahrs_write_reg(MPU6050_REG_PWR_MGMT_1, MPU6050_RESET_CMD);
    g_init_start_time = tick_ms;
    g_init_state = INIT_STATE_RESET_WAIT;
    g_init_deadline = tick_ms + 100U;
    g_status = IMU_AHRS_STATUS_INIT_BUSY;
}

/* ================================================================
 *  初始化：轮询（非阻塞状态机）
 *  需在主循环中反复调用，直到返回 READY 或 ERROR
 * ================================================================ */
IMU_AHRS_Status_t IMU_AHRS_Init_Poll(void)
{
    uint32_t now = tick_ms;

    switch (g_init_state)
    {
        case INIT_STATE_IDLE:
            g_status = IMU_AHRS_STATUS_ERROR;
            break;

        case INIT_STATE_RESET_WAIT:
            /* 等待 100ms 复位完成 */
            if ((int32_t)(now - g_init_deadline) >= 0)
            {
                if (imu_ahrs_write_reg(MPU6050_REG_PWR_MGMT_1, MPU6050_WAKEUP_CMD) != 0)
                {
                    /* 2 秒内重试 */
                    if ((int32_t)(now - g_init_start_time) < 2000)
                    {
                        imu_ahrs_write_reg(MPU6050_REG_PWR_MGMT_1, MPU6050_RESET_CMD);
                        g_init_deadline = now + 100U;
                    }
                    else
                    {
                        g_init_state = INIT_STATE_ERROR;
                        g_status = IMU_AHRS_STATUS_ERROR;
                    }
                }
                else
                {
                    g_init_state = INIT_STATE_CONFIG;
                }
            }
            break;

        case INIT_STATE_CONFIG:
            /* 配置采样率、DLPF、量程 */
            if (imu_ahrs_write_reg(MPU6050_REG_SMPLRT_DIV,  0x00U) != 0 ||
                imu_ahrs_write_reg(MPU6050_REG_CONFIG,      MPU6050_CONFIG_42HZ) != 0 ||
                imu_ahrs_write_reg(MPU6050_REG_GYRO_CONFIG, MPU6050_GYRO_2000DPS) != 0 ||
                imu_ahrs_write_reg(MPU6050_REG_ACCEL_CONFIG, MPU6050_ACCEL_4G) != 0)
            {
                g_init_state = INIT_STATE_ERROR;
                g_status = IMU_AHRS_STATUS_ERROR;
            }
            else
            {
                g_init_state = INIT_STATE_CALIB_SAMPLE;
                g_init_deadline = now;
            }
            break;

        case INIT_STATE_CALIB_SAMPLE:
            /* 批量采集校准样本 */
            while (g_calib_sample_idx < IMU_AHRS_CALIB_SAMPLES)
            {
                if (imu_ahrs_calibrate_gyro_offset_step() != 0)
                {
                    g_init_state = INIT_STATE_ERROR;
                    g_status = IMU_AHRS_STATUS_ERROR;
                    break;
                }
            }
            if (g_init_state == INIT_STATE_CALIB_SAMPLE)
                g_init_state = INIT_STATE_DONE;
            break;

        case INIT_STATE_DONE:
            g_status = IMU_AHRS_STATUS_READY;
            break;

        case INIT_STATE_ERROR:
        default:
            g_status = IMU_AHRS_STATUS_ERROR;
            break;
    }
    return g_status;
}

IMU_AHRS_Status_t IMU_AHRS_GetStatus(void)
{
    return g_status;
}

/* ================================================================
 *  读取原始传感器数据（14 字节 burst read）
 *  并减去陀螺仪零偏，转换为物理单位
 * ================================================================ */
int32_t IMU_AHRS_Update_Data(void)
{
    uint8_t burst[14];

    if (imu_ahrs_read_multi(MPU6050_REG_ACCEL_XOUT_H, burst, 14U) != 0)
        return -1;

    /* 加速度计（原始值） */
    g_imu.ax = imu_ahrs_make_s16(burst[0], burst[1]);
    g_imu.ay = imu_ahrs_make_s16(burst[2], burst[3]);
    g_imu.az = imu_ahrs_make_s16(burst[4], burst[5]);

    /* 陀螺仪（减去零偏） */
    g_imu.gx = (int16_t)(imu_ahrs_make_s16(burst[8],  burst[9])  - g_imu.gx_offset);
    g_imu.gy = (int16_t)(imu_ahrs_make_s16(burst[10], burst[11]) - g_imu.gy_offset);
    g_imu.gz = (int16_t)(imu_ahrs_make_s16(burst[12], burst[13]) - g_imu.gz_offset);

    /* 转换为物理单位 */
    g_imu.acc_g[0]    = ((float)g_imu.ax) / IMU_AHRS_ACC_LSB_PER_G;
    g_imu.acc_g[1]    = ((float)g_imu.ay) / IMU_AHRS_ACC_LSB_PER_G;
    g_imu.acc_g[2]    = ((float)g_imu.az) / IMU_AHRS_ACC_LSB_PER_G;
    g_imu.gyro_deg[0] = ((float)g_imu.gx) / IMU_AHRS_GYRO_LSB_PER_DPS;
    g_imu.gyro_deg[1] = ((float)g_imu.gy) / IMU_AHRS_GYRO_LSB_PER_DPS;
    g_imu.gyro_deg[2] = ((float)g_imu.gz) / IMU_AHRS_GYRO_LSB_PER_DPS;
    g_imu.gyro_rad[0] = g_imu.gyro_deg[0] * IMU_AHRS_DEG2RAD;
    g_imu.gyro_rad[1] = g_imu.gyro_deg[1] * IMU_AHRS_DEG2RAD;
    g_imu.gyro_rad[2] = g_imu.gyro_deg[2] * IMU_AHRS_DEG2RAD;
    g_imu.temp_c       = (((float)imu_ahrs_make_s16(burst[6], burst[7])) / IMU_AHRS_TEMP_SENS) + IMU_AHRS_TEMP_OFFSET_C;

    return 0;
}

/* ================================================================
 *  Mahony 互补滤波器 — 姿态解算核心算法
 *
 *  原理：
 *    1. 陀螺仪积分：用角速度更新四元数（高频响应好，但会漂移）
 *    2. 加速度计校正：假设长期平均加速度 = 重力矢量，
 *       通过叉积计算姿态误差，用 PI 控制器补偿陀螺仪漂移
 *    3. 自适应 KP：根据加速度模长判断是否受外力干扰，
 *       干扰时降低 KP 避免错误校正
 *    4. KI 积分：累积低频误差，补偿陀螺仪温漂等慢变因素
 *    5. 四元数归一化：防止数值发散
 * ================================================================ */
void IMU_AHRS_Update_Attitude(float dt_s)
{
    if (g_yaw_enable == 0) return;
    if (dt_s <= 0.0f) dt_s = IMU_AHRS_CTRL_PERIOD_S;

    float q0 = g_imu.q0, q1 = g_imu.q1, q2 = g_imu.q2, q3 = g_imu.q3;
    float gx = g_imu.gyro_rad[0], gy = g_imu.gyro_rad[1], gz = g_imu.gyro_rad[2];
    float acc_x = g_imu.acc_g[0], acc_y = g_imu.acc_g[1], acc_z = g_imu.acc_g[2];

    /* ---- 加速度计向量归一化 ---- */
    float recip_norm = imu_ahrs_inv_sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z));
    float acc_norm = 1.0f / recip_norm;
    acc_x *= recip_norm; acc_y *= recip_norm; acc_z *= recip_norm;

    /* ---- 自适应 KP：加速度模长偏离 1g 时降低校正强度 ---- */
    float kp = IMU_AHRS_MAHONY_KP_NOMINAL;
    if ((acc_norm < IMU_AHRS_ACC_NORM_LOW_G) || (acc_norm > IMU_AHRS_ACC_NORM_HIGH_G))
        kp = IMU_AHRS_MAHONY_KP_DISTURBED;

    // 临时调试用，调完删掉，别让 IMU 算法模块直接依赖 OLED！
    // sprintf((char *)oled_buffer, "%.2f", acc_norm);
    // OLED_ShowString(0, 6, oled_buffer, 8);

    /* ---- 计算重力矢量在机体坐标系中的投影（由当前四元数推导） ---- */
    float vx = 2.0f * ((q1 * q3) - (q0 * q2));
    float vy = 2.0f * ((q0 * q1) + (q2 * q3));
    float vz = (q0 * q0) - (q1 * q1) - (q2 * q2) + (q3 * q3);

    /* ---- 叉积求姿态误差（加速度计测量 vs 估计重力方向的偏差） ---- */
    float ex = (acc_y * vz) - (acc_z * vy);
    float ey = (acc_z * vx) - (acc_x * vz);

    /* ---- KI 积分反馈（累积低频误差，补偿陀螺仪慢变漂移） ---- */
    g_integral_fb_x += IMU_AHRS_MAHONY_KI * ex * dt_s;
    g_integral_fb_y += IMU_AHRS_MAHONY_KI * ey * dt_s;
    g_integral_fb_z = 0.0f; /* Yaw 不通过加速度计校正 */

    /* ---- PI 控制器输出叠加到陀螺仪角速度 ---- */
    gx += (kp * ex) + g_integral_fb_x;
    gy += (kp * ey) + g_integral_fb_y;
    gz += g_integral_fb_z;

    /* ---- 一阶龙格-库塔法更新四元数 ---- */
    float q0p = q0, q1p = q1, q2p = q2, q3p = q3;
    q0 += 0.5f * ((-q1p * gx) - (q2p * gy) - (q3p * gz)) * dt_s;
    q1 += 0.5f * (( q0p * gx) + (q2p * gz) - (q3p * gy)) * dt_s;
    q2 += 0.5f * (( q0p * gy) - (q1p * gz) + (q3p * gx)) * dt_s;
    q3 += 0.5f * (( q0p * gz) + (q1p * gy) - (q2p * gx)) * dt_s;

    /* ---- 四元数归一化 ---- */
    recip_norm = imu_ahrs_inv_sqrt((q0 * q0) + (q1 * q1) + (q2 * q2) + (q3 * q3));
    q0 *= recip_norm; q1 *= recip_norm; q2 *= recip_norm; q3 *= recip_norm;
    g_imu.q0 = q0; g_imu.q1 = q1; g_imu.q2 = q2; g_imu.q3 = q3;

    /* ---- 转弯角度累加（陀螺仪 Z 轴积分，短时高精度） ---- */
    g_turn_angle += g_imu.gyro_deg[2] * dt_s;
    
    /* ---- 四元数转欧拉角 ---- */
    g_imu.roll  = atan2f(2.0f * ((q0 * q1) + (q2 * q3)),
                          1.0f - 2.0f * ((q1 * q1) + (q2 * q2))) * IMU_AHRS_RAD2DEG;
    g_imu.pitch = atan2f(-2.0f * ((q1 * q3) - (q0 * q2)),
                          1.0f - 2.0f * ((q2 * q2) + (q3 * q3))) * IMU_AHRS_RAD2DEG;
    g_imu.yaw   = atan2f(2.0f * ((q0 * q3) + (q1 * q2)),
                          1.0f - 2.0f * ((q2 * q2) + (q3 * q3))) * IMU_AHRS_RAD2DEG;
}

/* ================================================================
 *  获取姿态数据
 * ================================================================ */
const IMU_AHRS_Data_t *IMU_AHRS_GetData(void)
{
    return &g_imu;
}

/* ================================================================
 *  Yaw 漂移校准 — 最小二乘线性回归
 *
 *  算法：采集 N 个样本点 (t, yaw)，用最小二乘法拟合直线
 *        yaw = k * t + b
 *        斜率 k 即为漂移速率（°/样本）
 *        后续使用: yaw_compensated = yaw_raw - k * t
 * ================================================================ */
void IMU_AHRS_Yaw_Calib_Start(void)
{
    g_yaw_calib_state = YAW_CALIB_COLLECTING;
    g_yaw_calib_count = 0;
    g_yaw_calib_start_time = tick_ms;
    g_yaw_sum_t  = 0.0; g_yaw_sum_y  = 0.0;
    g_yaw_sum_t2 = 0.0; g_yaw_sum_ty = 0.0;
}

int8_t IMU_AHRS_Yaw_Calib_Poll(void)
{
    if (g_yaw_calib_state == YAW_CALIB_IDLE) return -1;
    if (g_yaw_calib_state == YAW_CALIB_DONE) return 1;

    if (g_yaw_calib_count < IMU_AHRS_YAW_CALIB_SAMPLES)
    {
        double t = (double)(tick_ms - g_yaw_calib_start_time);
        double y = (double)g_imu.yaw;
        g_yaw_sum_t  += t;
        g_yaw_sum_y  += y;
        g_yaw_sum_t2 += t * t;
        g_yaw_sum_ty += t * y;
        g_yaw_calib_count++;
        return 0; /* 采集中 */
    }

    /* 最小二乘法: k = (n*Σ(t*y) - Σt*Σy) / (n*Σ(t²) - (Σt)²) */
    double n = (double)g_yaw_calib_count;
    double denom = (n * g_yaw_sum_t2) - (g_yaw_sum_t * g_yaw_sum_t);
    if (fabs(denom) > 0.0001)
        g_yaw_drift_k = (float)(((n * g_yaw_sum_ty) - (g_yaw_sum_t * g_yaw_sum_y)) / denom);

    g_yaw_calib_state = YAW_CALIB_DONE;
    return 1; /* 完成 */
}

float IMU_AHRS_Get_Yaw_Compensated(void)
{
    if (g_yaw_drift_k == 0.0f) return g_imu.yaw;
    uint32_t now = tick_ms;
    float dt_ms = (float)(now - g_yaw_calib_start_time);
    return g_imu.yaw - g_yaw_drift_k * dt_ms;
}

void IMU_AHRS_Set_Yaw_Drift_K(float k)  { g_yaw_drift_k = k; }
float IMU_AHRS_Get_Yaw_Drift_K(void)     { return g_yaw_drift_k; }

/* ================================================================
 *  姿态重置 & 使能控制
 * ================================================================ */
void IMU_AHRS_Yaw_Reset(void)
{
    g_yaw_enable = 0;
    g_imu.q0 = 1.0f; g_imu.q1 = 0.0f; g_imu.q2 = 0.0f; g_imu.q3 = 0.0f;
    g_imu.pitch = 0.0f; g_imu.roll = 0.0f; g_imu.yaw = 0.0f;
    g_integral_fb_x = 0.0f; g_integral_fb_y = 0.0f; g_integral_fb_z = 0.0f;
    g_yaw_start_time = tick_ms;
}

void IMU_AHRS_Attitude_Enable(void)
{
    g_yaw_enable = 1;
    g_yaw_start_time = tick_ms;
}

void IMU_AHRS_Attitude_Disable(void) { g_yaw_enable = 0; }
uint8_t IMU_AHRS_Attitude_IsEnabled(void) { return g_yaw_enable; }

/* ================================================================
 *  转弯角度测量（陀螺仪 Z 轴积分）
 *  转弯前调用 Reset()，转弯后调用 Get() 获取累计角度
 *  短时（< 5 秒）精度可达 ±0.5°，支持任意角度
 * ================================================================ */
void IMU_AHRS_TurnAngle_Reset(void)
{
    g_turn_angle = 0.0f;
}

float IMU_AHRS_TurnAngle_Get(void)
{
    return g_turn_angle;
}