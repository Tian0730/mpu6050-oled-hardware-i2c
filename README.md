# 循迹小车 — 电控逻辑总览

> 基于 TI MSPM0G3507 的两轮差分驱动小车，支持 **速度环 / 角度环 / 位置环** 三环混控，通过 VOFA+ 实时在线调参，可在正方形赛道上自主循迹。

---

## 一、硬件架构

```
┌──────────┐   I2C     ┌──────────────┐   I2C    ┌────────┐
│  MPU6050 │──────────→│  MSPM0G3507  │─────────→│  OLED  │
│ (6轴IMU)  │           │   (主控MCU)   │          │ (显示) │
└──────────┘           └──────────────┘          └────────┘
                            │      │
              ┌─────────────┘      └─────────────┐
              ↓                                   ↓
   ┌──────────────────┐               ┌──────────────────────┐
   │ 灰度传感器阵列 ×8  │               │  TB6612 双路电机驱动   │
   │ (f1~f8 黑线检测)  │               │  + 霍尔编码器 ×2       │
   └──────────────────┘               └──────────────────────┘
```

| 模块 | 接口 | 功能 |
|------|------|------|
| MPU6050 | I2C (0x68) | 6轴姿态感知（加速度计+陀螺仪） |
| OLED 128×64 | I2C | 实时显示 Pitch/Roll/Yaw/速度/误差 |
| 灰度传感器 ×8 | GPIO | 黑线检测，循迹位置环 |
| TB6612 | GPIO + PWM | 双路电机驱动，PWM 0~999 |
| 霍尔编码器 ×2 | 外部中断 + Timer | 车轮转速/里程测量 |

---

## 二、软件架构

```
mpu6050-oled-hardware-i2c/
├── main.c                              # 主循环 + 赛道状态机
├── Drivers/
│   ├── IMU_AHRS/                       # MPU6050 + Mahony 姿态解算
│   │   └── imu_ahrs.c/.h               #   零偏校准 / 自适应KP / Yaw漂移补偿
│   ├── AnglePID/                       # 角度 PD 控制器
│   ├── Encoder/                        # 编码器 + 速度环
│   │   ├── bsp_motor_hallencoder.c/.h  #   霍尔编码器中断计数 (5ms)
│   │   ├── encoder.c/.h               #   速度计算 (降采样 + EMA滤波)
│   │   ├── speed_pid.c/.h             #   PID 算法 (dt归一化 + 抗积分饱和)
│   │   └── speed_control.c/.h         #   速度环封装
│   ├── TB6612_Motor/                   # 电机驱动 + 角度/位置控制
│   │   ├── bsp_tb6612.c/.h            #   TB6612 底层驱动
│   │   ├── turn.c/.h                  #   转弯+走直线状态机 (角度环)
│   │   └── follow.c/.h                #   灰度传感器PID + 弯道检测 (位置环)
│   ├── Mix_Control/                    # 三环混控器
│   │   └── control_mixer.c/.h
│   ├── VOFA/                           # VOFA+ 串口在线调参
│   ├── Control System Mode/            # car_config.h + control_config.h
│   ├── OLED_Hardware_I2C/              # OLED 显示驱动
│   └── Ultrasonic_Capture/             # 超声波避障 (可选)
```

---

## 三、三环混控模型（核心）

### 3.1 混控公式

```c
base  = g_mix_base  + g_speed_diff;        // 速度通道：左右轮同向加减速
total = g_angle_diff + g_follow_diff;       // 转向通道：左右轮差速
left  = clamp(base + total, 0, 999);        // 左轮 PWM
right = clamp(base - total, 0, 999);        // 右轮 PWM
```

### 3.2 正交解耦

```
                      速度环 PID (控制快慢)
                            │
                    g_speed_diff
                            ↓
   g_mix_base ──→ [ base ] ──→ 左轮 = base + total
                            ──→ 右轮 = base - total
                            ↑
                    g_angle_diff + g_follow_diff
                            │
              ┌─────────────┴─────────────┐
              ↓                           ↓
         角度环 PD (方向)            位置环 PID (循线)
         MPU6050 Yaw              灰度传感器 ×8
```

**速度环走 `base` 通道，角度环和位置环走 `total` 通道，正交解耦，互不干扰。**

### 3.3 三环参数

| 控制环 | 类型 | 传感器 | 输出变量 | 作用 |
|--------|------|--------|---------|------|
| 速度环 | PID | 霍尔编码器×2 | `g_speed_diff` | 维持目标车速 |
| 角度环 | PD | MPU6050 Yaw | `g_angle_diff` | 航向保持/精确转弯 |
| 位置环 | PID | 灰度传感器×8 | `g_follow_diff` | 沿黑线行驶 |

---

## 四、IMU 姿态解算

### 4.1 初始化（非阻塞状态机）

```
IMU_AHRS_Init_Start() → 复位 → 配置 (1kHz/42Hz/±2000dps/±4g) → 500次零偏校准 → 就绪
```

### 4.2 Mahony 互补滤波

```
陀螺仪角速度 ──→ 四元数积分 (高频好，会漂移)
                      ↑
加速度计重力 ──→ PI 校正 (长期稳定，修正漂移)
```

- **自适应 KP**：加速度模长偏离 1.0g 时自动降低校正强度，避免外力干扰
- **Yaw 漂移补偿**：启动时采集 1000 样本，最小二乘线性回归拟合漂移速率

### 4.3 关键参数

```c
KP_NOMINAL = 2.0f    // 名义 KP
KP_DISTURBED = 0.5f  // 扰动 KP
KI = 0.005f           // 积分系数
```

---

## 五、编码器与速度计算

```
霍尔编码器 A/B → GPIO 外部中断 (4倍频) → 5ms 定时器拷贝 → 主循环累积6次(30ms) → 计算速度
```

```
每圈脉冲 = 分辨率 × 减速比
轮速(RPM) = (脉冲 / 每圈脉冲) × (60 / 0.03s)
线速度(mm/s) = RPM × π×直径 / 60
```

- **降采样**：6次累积 (30ms)，降低量化噪声
- **EMA 滤波**：α=0.15，约 33ms 响应 63% 阶跃变化

| 参数 | 小车A | 小车B |
|------|-------|-------|
| 编码器分辨率 | 11 | 13 |
| 减速比 | 21.3 | 28 |
| 轮子直径 | 65mm | 65mm |
| 每脉冲距离 | 0.87mm | 0.56mm |

---

## 六、PID 算法

### 速度环 PID — dt 归一化 + 抗积分饱和

```c
error = target - current
integral += error × dt;   integral = clamp(integral, ±limit)
derivative = (error - last_error) / dt
output = KP×error + KI×integral + KD×derivative
output = clamp(output, ±output_limit)
```

### 角度环 PD — 角度环绕，无 dt 归一化

```c
error = wrap_180(target - current)
derivative = error - last_error
output = KP×error + KD×derivative;  output = clamp(output, ±limit)
```

### 位置环 PID — 灰度传感器消抖

```c
bias = 8传感器加权偏差 (-3.5 ~ +3.5)
integral += bias × dt;  derivative = (bias - last_bias) / dt
correction = KP×bias + KI×integral + KD×derivative
```

传感器消抖：连续 3 次相同状态才确认，计数器上下限锁死。

---

## 七、电机控制

```c
AO_Control(dir, speed)  // A 电机: dir=1正转, speed=0~999
BO_Control(dir, speed)  // B 电机: 同上
TB6612_Motor_Stop()     // 四路全高，刹车
```

**转弯**（非阻塞状态机）：`TurnByAngle_Start(±90°)` → `Turn_Poll()` 轮询，自适应基速，连续 5 帧稳定判定到达，超时保护。

**走直线**：`GoStraight_Start(speed)` 快照当前 Yaw 启动 PD 航向保持，`GoStraight_Poll()` 每帧维持。

---

## 八、VOFA+ 在线调参

- **MCU→VOFA+**：FireWater 协议 `ch1,...,chN\r\n`
- **VOFA+→MCU**：`KP=1.5` / `KI=0.02` / `KD=0.3` / `STOP` / `GO`

```c
#define VOFA_MODE  0   // 0=速度环  1=角度环  2=位置环
```

| 模式 | ch0 | ch1 | ch2 | ch3 | ch4 | ch5 | ch6 |
|------|-----|-----|-----|-----|-----|-----|-----|
| 速度环 | 目标 | 当前 | 误差 | P | I | D | 输出 |
| 角度环 | - | Yaw | 误差 | P | D | 输出 | - |
| 位置环 | 偏差 | P | I | D | 修正 | 传感器mask | - |

---

## 九、配置系统

### 控制环开关 (`control_config.h`)

```c
#define USE_SPEED_CONTROL   1   // 速度环
#define USE_ANGLE_CONTROL   1   // 角度环
#define USE_FOLLOW_CONTROL  1   // 位置环
#define USE_VOFA_DEBUG      1   // VOFA+ 调参
```

调试建议：单独调试某环时只开那个环，其余关掉；完整循迹三环全开。

### 小车切换 (`car_config.h`)

```c
#define CAR_SELECT  0   // 0=小车A, 1=小车B，所有参数自动切换
```

### 两车 PID 参数

| 控制环 | 参数 | 小车A | 小车B |
|--------|------|-------|-------|
| 速度环 | KP / KI / KD | 1.2 / 0.0 / 0.5 | 1.2 / 0.0 / 0.5 |
| | 输出限幅 / 基速 | ±400 / 300 | ±130 / 100 |
| 转弯角度环 | KP / KD | 0.5 / 0.15 | 0.5 / 0.15 |
| | 限幅 / 基速 | ±800 / 300 | ±800 / 100 |
| 直线角度环 | KP / KD / 限幅 | 35.0 / 0.5 / ±150 | 35.0 / 0.5 / ±150 |
| 循迹位置环 | KP / KI / KD | 30.0 / 0.0 / 14.0 | 30.0 / 0.0 / 14.0 |

---

## 十、主循环与赛道状态机

```
while(1):
    dt = 帧间隔
    IMU_AHRS_Update_Data()           ← 读 MPU6050
    IMU_AHRS_Update_Attitude(dt)     ← Mahony 姿态解算
    VOFA_Output() (每20ms)           ← 发送 FireWater
    SpeedLoop_Update(dt)             ← 速度环 PID

    ┌─ ST_PRE_TURN: 直走 300ms → TurnByAngle_Start(±90°)
    ├─ ST_TURNING:  Turn_Poll() 轮询 → 完成回直行
    └─ ST_STRAIGHT:
         ├─ 弯道检测 (f1/f8) + 冷却期 → ST_PRE_TURN
         ├─ 黑线区域: 三环全开 (位置偏差大时削弱角度环)
         └─ 空白区域: 角度环 + 速度环
```

**双环权重**：位置偏差 ≤40 全权重，40~80 线性衰减，>80 纯位置环全力回线。

---

## 十一、关键修复记录

| 问题 | 根因 | 修复 |
|------|------|------|
| 编码器速度跳动大 | 单次采样脉冲太少 | 累积降采样6次 + EMA滤波 |
| 位置环低频摇摆 | 微分项未 dt 归一化 | 除以 dt |
| 双环振动 | 位置环 KP 过大 | KP 120→30, KD→14 |
| 空白区偏航 | 每次重新快照 Yaw | 删掉空白区 GoStraight_Start |
| 转弯后偏航 | 快照当前 Yaw 而非目标 | 用 GoStraight_StartAt(Target) |
| 速度显示负数 | 编码器 A 相方向反 | 取反 Encoder_A 计数 |
| 转弯精度不足 | TOLERANCE 过大 | 5°→1.5° + 连续稳定帧检测 |

---

## 十二、快速上手

1. 选择小车：`car_config.h` → `CAR_SELECT`
2. 选择调试模式：`control_config.h` → 三环开关 + `USE_VOFA_DEBUG`
3. 烧录 → VOFA+ 选 FireWater 协议 → 观察波形 → 发送 `KP=xx` 调参
4. 调好后三环全开 → 放赛道上 → 自动循迹

| 工具 | 说明 |
|------|------|
| IDE | Code Composer Studio + SysConfig |
| MCU | MSPM0G3507 (80MHz Cortex-M0+) |
| 调参 | VOFA+ v1.3.10+ (FireWater 协议) |