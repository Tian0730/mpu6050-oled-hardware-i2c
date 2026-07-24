# 工程电控逻辑总览

## 一、硬件架构

```
┌──────────┐   I2C    ┌──────────┐   SPI/I2C  ┌────────┐
│  MPU6050 │─────────→│  MSPM0   │──────────→│  OLED  │
│  (6轴IMU) │          │  (MCU)   │           │ (显示) │
└──────────┘          └──────────┘           └────────┘
                          │    │
              ┌───────────┘    └───────────┐
              ↓                            ↓
     ┌────────────────┐          ┌────────────────┐
     │ 灰度传感器 ×8   │          │  TB6612 电机驱动│
     │ (f1~f8 巡线)   │          │  + 霍尔编码器 ×2│
     └────────────────┘          └────────────────┘
```

---

## 二、文件架构

```
Drivers/
├── IMU_AHRS/          imu_ahrs.c/.h     ← Mahony姿态解算，航向校准
├── AnglePID/          angle_pid.c/.h    ← 角度PD算法（无dt归一化）
├── TB6612_Motor/
│   ├── bsp_tb6612.c/.h                 ← 电机底层驱动（AO_Control/BO_Control）
│   ├── follow.c/.h                      ← 位置环：灰度传感器+PID，弯道检测
│   └── turn.c/.h                        ← 角度环：转弯+走直线状态机
├── Encoder/
│   ├── encoder.c/.h                     ← 编码器速度计算（降采样+EMA滤波）
│   ├── speed_pid.c/.h                   ← 速度PID算法（带dt归一化+积分抗饱和）
│   └── speed_control.c/.h               ← 速度环封装
├── Mix_Control/      control_mixer.c/.h ← 三环混控器
└── Control System Mode/
    └── control_config.h                 ← 三环开关
main.c                                   ← 主循环+正方形赛道状态机
```

---

## 三、三环混控模型

### 3.1 混控公式（`control_mixer.c`）

```c
base  = g_mix_base  + g_speed_diff;        // 速度通道 (300 ± 速度修正)
total = g_angle_diff + g_follow_diff;       // 转向通道 (角度+位置)
left  = base + total;                       // 左轮 PWM
right = base - total;                       // 右轮 PWM
```

| 通道 | 变量 | 来源 | 作用 |
|------|------|------|------|
| 基速 | `g_mix_base` | `Mixer_Init(300)` | 固定基速 300 |
| 速度修正 | `g_speed_diff` | 速度环 PID 输出 | 调速（±350） |
| 角度修正 | `g_angle_diff` | GoStraight PD 输出 | 航向保持（±150） |
| 位置修正 | `g_follow_diff` | 位置环 PID 输出 | 循线微调（±200） |

### 3.2 三环正交解耦

```
   速度环 → 调整 base（左右轮同向加减速）→ 控制"快慢"
   角度环 ↘
            → 调整 total（左右轮差速）→ 控制"方向"
   位置环 ↗
```

**速度环和角度/位置环走不同通道，不会互相干扰。**

---

## 四、控制环参数一览

### 4.1 速度环（`speed_control.h`）

```
SPEED_KP    = 1.5      比例系数
SPEED_KI    = 0.6      积分系数（dt归一化）
SPEED_KD    = 0.05     微分系数（dt归一化）
OUTPUT_LIMIT = 350     输出限幅（对应 PWM 差速修正量）
目标速度    = 200 mm/s
```

- 编码器：20ms 有效更新周期（4×5ms 降采样）
- EMA 滤波系数：0.15
- PID 带 dt 归一化 + 积分抗饱和

### 4.2 角度环（`turn.h`）

```
STRAIGHT_KP   = 40.0   走直线比例系数
STRAIGHT_KD   = 0.5    走直线微分系数（无dt归一化，等效于dt归一化后的~100）
STRAIGHT_LIMIT = 150   输出限幅

TURN_KP       = 0.5    转弯比例系数
TURN_KD       = 0.15   转弯微分系数
TURN_TOLERANCE = 2.0   转弯精度（°）
TURN_BASE_SPEED = 300  转弯基速
```

- 纯 PD 控制（无 I）
- **无 dt 归一化**（`AnglePD_Update` 直接 `error - last_error`）

### 4.3 位置环（`follow.c`，当前实测值）

```
KP = 30                比例系数
KI = 0                 积分系数（未启用）
KD = 14.0              微分系数（dt归一化）
MAX_CORRECTION = 200   最大修正量
SENSOR_FILTER_COUNT = 3  传感器消抖次数
```

- 偏差范围：-3.5 ~ +3.5（8 传感器加权平均）
- PID 带 dt 归一化
- 传感器消抖：计数器上下限锁死，防止无限累积

---

## 五、主循环流程（`main.c`，≈200Hz）

```
while(1):
    dt = 计算帧间隔

    ┌─ IMU_AHRS_Update_Data()          ← 读 MPU6050 原始数据
    ├─ IMU_AHRS_Update_Attitude(dt)    ← Mahony 姿态解算（pitch/roll/yaw）
    ├─ SpeedLoop_Update(dt)            ← 速度环 PID 更新
    ├─ Yaw 校准（仅启动时一次）        ← 校准完成后 GoStraight_Start(300)
    ├─ OLED 更新（每 50ms）
    │
    └─ 状态机 ────────────────────────┐
        ├─ ST_PRE_TURN（预转）         │
        │   角度环直走 300ms           │
        │   到时间 → TurnByAngle_Start(±90°) → ST_TURNING
        │                             │
        ├─ ST_TURNING（转弯）           │
        │   Turn_Poll() 轮询           │
        │   完成 → GoStraight_StartAt(Turn_GetTarget(), 300) → ST_STRAIGHT
        │                             │
        └─ ST_STRAIGHT（直行）          │
            ├─ 弯道检测（f1/f8 原始值）  │
            │   → 冷却期检查 → ST_PRE_TURN
            │
            ├─ 黑线区域（有传感器触发）  │
            │   位置环 PID + 角度环 PD + 速度环 PID
            │   双环权重：follow_abs≤40 → 角度环100%
            │            40<follow_abs≤80 → 角度环线性衰减
            │            follow_abs>80 → 角度环关闭
            │
            └─ 空白区域（全白）         │
               角度环 PD + 速度环 PID
               位置环关闭（follow=0）
```

---

## 六、状态机迁移图

```
                    ┌──────────────────────────────────┐
                    │           ST_STRAIGHT             │
                    │  ┌────────────┐ ┌──────────────┐ │
                    │  │ 黑线：三环  │ │ 空白：角度+速度│ │
                    │  └────────────┘ └──────────────┘ │
                    └──────┬───────────────┬───────────┘
                           │ f1/f8 检测到弯  │
                           ↓                  │
                    ┌──────────────┐          │
                    │ ST_PRE_TURN  │          │
                    │ 直走 300ms   │          │
                    └──────┬───────┘          │
                           │ 时间到            │
                           ↓                  │
                    ┌──────────────┐          │
                    │ ST_TURNING   │          │
                    │ IMU 转 90°   │──────────┘
                    │ 完成 → 回直行  │
                    └──────────────┘
```

---

## 七、关键修复清单

| 问题 | 修复 |
|------|------|
| 编码器速度跳动 | 累积降采样（20ms）+ EMA 低通滤波 |
| 位置环传感器响应慢 | 消抖计数器加锁死上下限 |
| 位置环低频摇摆 | dt 归一化微分项 |
| 双环模式振动 | 位置环 KP 从 120 降到 30，KD 调至 14 |
| 空白区 GoStraight 锁死错误航向 | 删掉空白区的 `GoStraight_Start` |
| 第二段空白区偏航 | 转弯后用 `GoStraight_StartAt(Turn_GetTarget())` 替代快照 |
| 转弯精度 | TOLERANCE 从 5° 降到 2° |
| 角度环力度不足 | STRAIGHT_KP 从 15 提到 40 |

---

## 八、调参速查

| 现象 | 调哪里 | 文件 |
|------|------|------|
| 黑线蛇形 | 位置环 KP↓ / KD↑ | `follow.c` |
| 空白区偏航 | 角度环 STRAIGHT_KP↑ | `turn.h` |
| 转弯冲过头 | TURN_TOLERANCE↓ / TURN_KD↑ | `turn.h` |
| 转弯转不动 | TURN_KP↑ / TURN_BASE_SPEED↑ | `turn.h` |
| 速度不稳 | 速度环 SPEED_KP↑ / SPEED_KI↑ | `speed_control.h` |
| 三环打架 | 位置环 KP↓（角度环已兜底） | `follow.c` |
