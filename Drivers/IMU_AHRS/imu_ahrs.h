#ifndef _IMU_AHRS_H_
#define _IMU_AHRS_H_

#include <stdint.h>

/* ================================================================
 *  MPU6050 配置参数【重点：要根据实际硬件调整】
 * ================================================================ */
#define IMU_AHRS_I2C_ADDR           0x68

/* 陀螺仪零偏校准样本数 */
#define IMU_AHRS_CALIB_SAMPLES      500

/* 以下两个是 MPU6050 的量程决定的，只要和 imu_ahrs.c 里配的寄存器值一致就不用动 */
/* 加速度计 LSB/g（±4g 量程: 8192） */
#define IMU_AHRS_ACC_LSB_PER_G      8192.0f

/* 陀螺仪 LSB/°/s（±2000dps 量程: 16.4） */
#define IMU_AHRS_GYRO_LSB_PER_DPS   16.4f

/* MPU6050 数据手册规定的，不用动。温度数据本来也不参与姿态解算，只是辅助信息。 */
/* 温度传感器参数 */            // 可不用
#define IMU_AHRS_TEMP_SENS          340.0f
#define IMU_AHRS_TEMP_OFFSET_C      36.53f

/* ================================================================
 *  Mahony 互补滤波器参数
 * ================================================================ */
 /* 调试说明
 KP 值	                效果	                                现象
 太小 	        过度平滑，响应慢	             晃动传感器，OLED 数值半天才跟上
 合适            响应快且不抖	                晃动后立即跟上，静止时数值稳定
 太大 	        加速度计噪声直接耦合进来	     静止时数值也在 ±1° 跳动

调试步骤：
先把 KI 设为 0.0f
烧录程序，OLED 上观察 Pitch/Roll
快速晃动 MPU6050 再停下：
停下后数值过冲再回来 → KP 太大了，减小
停下后数值慢慢爬回来 → KP 太小了，增大
停下后立即稳定 → 合适
从 2.0 开始，每次 ±0.5，找到最舒服的值


再调 KI，KI 的作用是补偿陀螺仪慢速漂移。调法：
恢复 KP 到调试好的值
把传感器放完全静止，盯着 OLED 看 1 分钟
如果 Pitch/Roll 在 1 分钟内漂了超过 0.5° → 增大 KI（0.005 → 0.01 → 0.02）
如果静止时数值在 ±0.3° 内来回振荡 → KI 太大了，减小
经验值：KI 通常是 KP 的 1/200 ~ 1/1000，0.005 对 KP=2.0 来说已经很合理了。

KP_DISTURBED：
这个是在传感器受力干扰（比如你用手推小车）时自动切换到的低 KP，防止错误校正。
一般设为 KP_NOMINAL 的 1/4 左右，0.5 很合理，可以不动。
*/

/* 名义 KP（加速度计校正强度，正常状态） */
#define IMU_AHRS_MAHONY_KP_NOMINAL  2.0f

/* 扰动 KP（加速度模长异常时降低校正强度，如运动/撞击时） */
#define IMU_AHRS_MAHONY_KP_DISTURBED 0.5f

/* KI 积分系数（补偿陀螺仪慢变漂移） */
#define IMU_AHRS_MAHONY_KI          0.005f


/* 
    这是判断"是否受外力干扰"的阈值。静止时加速度模长应该 ≈ 1.0g。
    正常使用场景下晃动传感器，看 OLED 上显示的模长变化范围
    如果正常晃动时模长经常掉到 0.7 或蹦到 1.5，就适当放宽阈值（0.7~1.3）
    调完删掉调试代码
*/
/* 加速度模长正常范围（g），超出则判定为扰动 */
#define IMU_AHRS_ACC_NORM_LOW_G     0.8f
#define IMU_AHRS_ACC_NORM_HIGH_G    1.2f

/* ================================================================
 *  Yaw 漂移补偿参数
 * ================================================================ */
/* Yaw 校准样本数（用于线性回归） */
#define IMU_AHRS_YAW_CALIB_SAMPLES  500

/* 默认控制周期（秒），dt 无效时的回退值 */
#define IMU_AHRS_CTRL_PERIOD_S      0.01f

/* 弧度/角度转换 */
#define IMU_AHRS_DEG2RAD            0.01745329251f
#define IMU_AHRS_RAD2DEG            57.2957795131f

/* ================================================================
 *  数据结构
 * ================================================================ */
typedef struct {
    float q0, q1, q2, q3;           /* 四元数 */
    float pitch, roll, yaw;         /* 欧拉角（度） */
    int16_t ax, ay, az;             /* 原始加速度计 */
    int16_t gx, gy, gz;             /* 原始陀螺仪（已减零偏） */
    int16_t gx_offset, gy_offset, gz_offset; /* 陀螺仪零偏 */
    float acc_g[3];                 /* 加速度（g） */
    float gyro_deg[3];              /* 角速度（°/s） */
    float gyro_rad[3];              /* 角速度（rad/s） */
    float temp_c;                   /* 温度（°C） */
} IMU_AHRS_Data_t;

typedef enum {
    IMU_AHRS_STATUS_NOT_INIT = 0,
    IMU_AHRS_STATUS_INIT_BUSY,
    IMU_AHRS_STATUS_READY,
    IMU_AHRS_STATUS_ERROR
} IMU_AHRS_Status_t;

/* ================================================================
 *  API 函数声明
 * ================================================================ */

/* 初始化（非阻塞，需配合 Init_Poll 轮询） */
void IMU_AHRS_Init_Start(void);
IMU_AHRS_Status_t IMU_AHRS_Init_Poll(void);
IMU_AHRS_Status_t IMU_AHRS_GetStatus(void);

/* 读取原始传感器数据 */
int32_t IMU_AHRS_Update_Data(void);

/* Mahony 姿态更新（每次 Update_Data 后调用） */
void IMU_AHRS_Update_Attitude(float dt_s);

/* 获取姿态数据 */
const IMU_AHRS_Data_t *IMU_AHRS_GetData(void);

/* Yaw 漂移校准 */
void IMU_AHRS_Yaw_Calib_Start(void);
int8_t IMU_AHRS_Yaw_Calib_Poll(void);   /* 0=采集中, 1=完成, -1=空闲 */
float IMU_AHRS_Get_Yaw_Compensated(void);
void IMU_AHRS_Set_Yaw_Drift_K(float k);
float IMU_AHRS_Get_Yaw_Drift_K(void);

/* 转弯角度测量（陀螺仪 Z 轴积分，短时高精度，支持任意角度） */
void IMU_AHRS_TurnAngle_Reset(void);
float IMU_AHRS_TurnAngle_Get(void);

/* 姿态重置 & 使能 */
void IMU_AHRS_Yaw_Reset(void);
void IMU_AHRS_Attitude_Enable(void);
void IMU_AHRS_Attitude_Disable(void);
uint8_t IMU_AHRS_Attitude_IsEnabled(void);

#endif /* _IMU_AHRS_H_ */