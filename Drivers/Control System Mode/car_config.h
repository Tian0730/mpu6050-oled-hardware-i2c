#ifndef _CAR_CONFIG_H_
#define _CAR_CONFIG_H_

/* ================================================================
 *  小车参数配置开关
 *
 *  两台小车因电机和编码器不同，PID参数及硬件参数均不同
 *  切换 CAR_SELECT 后重新编译即可：
 *    0 = 小车A（小车）
 *    1 = 小车B（大车）
 *
 *  注意：修改此值后需重新编译整个工程
 * ================================================================ */
#define CAR_SELECT  1

/* ================================================================
 *  小车A 参数（小车）
 * ================================================================ */
#if CAR_SELECT == 0

/* ---- 编码器/电机参数 (encoder.h) ---- */
#define CAR_ENCODER_RESOLUTION          11
#define CAR_MOTOR_GEAR_RATIO            21.3f
#define CAR_WHEEL_DIAMETER_MM           65
#define CAR_ENCODER_DECIMATION          6
#define CAR_ENCODER_SPEED_FILTER_ALPHA  0.15f

/* ---- 速度环 PID (speed_control.h) ---- */
#define CAR_SPEED_KP             1.2f
#define CAR_SPEED_KI             0.0f
#define CAR_SPEED_KD             0.5f
#define CAR_SPEED_INTEGRAL_LIMIT 300.0f
#define CAR_SPEED_OUTPUT_LIMIT   400.0f

/* ---- 转弯角度环 PID (turn.h) ---- */
#define CAR_TURN_KP             0.5f
#define CAR_TURN_KD             0.15f
#define CAR_TURN_LIMIT          800.0f
#define CAR_TURN_TOLERANCE      1.5f
#define CAR_TURN_BASE_SPEED     300
#define CAR_TURN_MIN_SPEED      0
#define CAR_TURN_NEAR_ZONE      10.0f
#define CAR_TURN_TIMEOUT_MS     3000

/* ---- 直线角度环 PID (turn.h) ---- */
#define CAR_STRAIGHT_KP         35.0f
#define CAR_STRAIGHT_KD         0.5f
#define CAR_STRAIGHT_LIMIT      150.0f
#define CAR_STRAIGHT_SPEED      400

/* ---- 循线位置环 PID (follow.c) ---- */
#define CAR_FOLLOW_KP           30.0f
#define CAR_FOLLOW_KI           0.0f
#define CAR_FOLLOW_KD           14.0f
#define CAR_FOLLOW_MAX_CORRECTION   200
#define CAR_FOLLOW_MAX_INTEGRAL     100
#define CAR_FOLLOW_BIAS_LPF_ALPHA    0.85f   /* 偏差低通（0.55→0.85，入弯响应快55%） */
#define CAR_FOLLOW_DTERM_LPF_ALPHA   0.35f
#define CAR_FOLLOW_CORR_SLEW_PER_SEC 260.0f

/* ---- 循线预转/冷却 (main.c) ---- */
#define CAR_PRE_TURN_MS         300
#define CAR_TURN_COOLDOWN       500

/* ---- 半圆形循线参数 (main.c) ---- */
#define CAR_SEMICIRCLE_SPEED            250     /* 半圆循线基础速度 */
#define CAR_SEMICIRCLE_FOLLOW_KP        35.0f   /* 半圆循线位置环 KP */
#define CAR_SEMICIRCLE_FOLLOW_KI        2.0f    /* 半圆循线位置环 KI（消除弯道稳态误差） */
#define CAR_SEMICIRCLE_FOLLOW_KD        14.0f   /* 半圆循线位置环 KD */
#define CAR_SEMICIRCLE_ANGLE_SUPPRESS   0.3f    /* 角度环抑制系数（0=完全关闭，1=全开） */
#define CAR_SEMICIRCLE_LOST_TIMEOUT_MS  500     /* 丢线超时停止(ms) */
#define CAR_SEMICIRCLE_INTEGRAL_DECAY   0.85f   /* 直道积分衰减系数(每帧, 0=瞬清, 1=不清) */
#define CAR_SEMICIRCLE_STOP_MIN_BLACK   6       /* 启停线检测最少黑点数(6=8路中至少6路见黑) */
#define CAR_SEMICIRCLE_STOP_COOLDOWN_MS 2000    /* 启停线冷却时间(ms)，防止起步误触发 */
#define CAR_SEMICIRCLE_CURVE_STARTUP_IGNORE_MS 5000 /* 启动后屏蔽入弯判定时间(ms) */
#define CAR_SEMICIRCLE_CURVE_ENTER_BIAS 0.38f   /* 进入弯道阈值（PID切换） */
#define CAR_SEMICIRCLE_CURVE_EXIT_BIAS  0.18f   /* 退出弯道阈值（PID切换） */
#define CAR_SEMICIRCLE_CURVE_REENTRY_COOLDOWN_MS 800 /* 出弯后禁止重新入弯的冷却时间(ms) */
#define CAR_SEMICIRCLE_ANGLE_RELEASE_START 0.06f /* 角度环开始松手阈值（低于此偏差角度环100%） */
#define CAR_SEMICIRCLE_ANGLE_RELEASE_END   0.22f /* 角度环完全松手阈值（高于此偏差角度环0%） */
#define CAR_SEMICIRCLE_BIAS_DERIV_GAIN    0.5f  /* 偏差导数抑制系数 */
#define CAR_SEMICIRCLE_DEEP_CURVE_BIAS    1.5f  /* 深弯道阈值 */
#define CAR_SEMICIRCLE_DEEP_CURVE_SCALE   0.35f /* 深弯道衰减系数 */
#define CAR_SEMICIRCLE_SPEED_MIN        190     /* 弯道最低基速 */
#define CAR_SEMICIRCLE_SPEED_MAX        250     /* 直道最高基速 */
#define CAR_SEMICIRCLE_SPEED_RAMP_UP    120.0f  /* 允许升速斜率(单位: speed/s) */
#define CAR_SEMICIRCLE_SPEED_RAMP_DOWN  300.0f  /* 允许降速斜率(单位: speed/s) */

/* ---- 角速度环 PID (gyro_rate_control.h) ---- */
#define CAR_GYRO_RATE_KP             8.0f
#define CAR_GYRO_RATE_KI             0.05f
#define CAR_GYRO_RATE_KD             2.0f
#define CAR_GYRO_RATE_INTEGRAL_LIMIT 50.0f
#define CAR_GYRO_RATE_OUTPUT_LIMIT   120.0f

/* ---- 串级 PID 参数 (semicircle_follow_plus.h) ---- */
#define CAR_CASCADE_CURVE_POS_KP         12.0f   /* 外环弯道位置PID KP */
#define CAR_CASCADE_CURVE_POS_KI         0.02f    /* 外环弯道位置PID KI */
#define CAR_CASCADE_CURVE_POS_KD         0.0f    /* 外环弯道位置PID KD */
#define CAR_CASCADE_STRAIGHT_POS_KP      6.0f   /* 外环直道位置PID KP */
#define CAR_CASCADE_STRAIGHT_POS_KI      0.02f    /* 外环直道位置PID KI */
#define CAR_CASCADE_STRAIGHT_POS_KD      0.0f    /* 外环直道位置PID KD */
#define CAR_CASCADE_INNER_KP             15.0f   /* 内环角速度PID KP */
#define CAR_CASCADE_INNER_KI             0.0f   /* 内环角速度PID KI */
#define CAR_CASCADE_INNER_KD             0.0f    /* 内环角速度PID KD */
#define CAR_CASCADE_INNER_INTEGRAL_LIMIT 80.0f   /* 内环积分上限 */
#define CAR_CASCADE_INNER_OUTPUT_LIMIT   200.0f  /* 内环输出上限(差速) */
#define CAR_CASCADE_W_TARGET_MAX         150.0f  /* 期望角速度上限(°/s) */

/* ---- 默认基础速度 ---- */
#define CAR_DEFAULT_BASE_SPEED  300

/* ================================================================
 *  小车B 参数（大车）
 * ================================================================ */
#elif CAR_SELECT == 1

/* ---- 编码器/电机参数 (encoder.h) ---- */
#define CAR_ENCODER_RESOLUTION          13
#define CAR_MOTOR_GEAR_RATIO            28
#define CAR_WHEEL_DIAMETER_MM           65
#define CAR_ENCODER_DECIMATION          6
#define CAR_ENCODER_SPEED_FILTER_ALPHA  0.15f

/* ---- 速度环 PID (speed_control.h) ---- */
#define CAR_SPEED_KP             1.2f
#define CAR_SPEED_KI             0.0f
#define CAR_SPEED_KD             0.5f
#define CAR_SPEED_INTEGRAL_LIMIT 100.0f
#define CAR_SPEED_OUTPUT_LIMIT   130.0f

/* ---- 转弯角度环 PID (turn.h) ---- */
#define CAR_TURN_KP             0.5f
#define CAR_TURN_KD             0.15f
#define CAR_TURN_LIMIT          800.0f
#define CAR_TURN_TOLERANCE      1.5f
#define CAR_TURN_BASE_SPEED     200
#define CAR_TURN_MIN_SPEED      0
#define CAR_TURN_NEAR_ZONE      10.0f
#define CAR_TURN_TIMEOUT_MS     3000

/* ---- 直线角度环 PID (turn.h) ---- */
#define CAR_STRAIGHT_KP         24.0f
#define CAR_STRAIGHT_KD         0.5f
#define CAR_STRAIGHT_LIMIT      150.0f
#define CAR_STRAIGHT_SPEED      120

/* ---- 循线位置环 PID (follow.c) ---- */
#define CAR_FOLLOW_KP           30.0f
#define CAR_FOLLOW_KI           0.0f
#define CAR_FOLLOW_KD           14.0f
#define CAR_FOLLOW_MAX_CORRECTION   200
#define CAR_FOLLOW_MAX_INTEGRAL     100
#define CAR_FOLLOW_BIAS_LPF_ALPHA    0.85f
#define CAR_FOLLOW_DTERM_LPF_ALPHA   0.50f
#define CAR_FOLLOW_CORR_SLEW_PER_SEC 600.0f

/* ---- 循线预转/冷却 (main.c) ---- */
#define CAR_PRE_TURN_MS         150
#define CAR_TURN_COOLDOWN       500

/* ---- 半圆形循线参数 (main.c) ---- */
#define CAR_SEMICIRCLE_LOST_TIMEOUT_MS  1000    /* 丢线超时停止(ms) */
#define CAR_SEMICIRCLE_INTEGRAL_DECAY   0.85f   /* 直道积分衰减系数(每帧, 0=瞬清, 1=不清) */
#define CAR_SEMICIRCLE_STOP_MIN_BLACK   6       /* 启停线检测最少黑点数(6=8路中至少6路见黑) */
#define CAR_SEMICIRCLE_STOP_COOLDOWN_MS 5000    /* 启停线冷却时间(ms)，防止起步误触发 */
#define CAR_SEMICIRCLE_STOP_STARTUP_PROTECT_MS 5000 /* 启动后5秒内屏蔽启停线停机 */
#define CAR_SEMICIRCLE_CURVE_STARTUP_IGNORE_MS 5000 /* 启动后屏蔽入弯判定时间(ms) */
#define CAR_SEMICIRCLE_CURVE_ENTER_BIAS 0.3f    /* 进入弯道阈值（PID切换） */
#define CAR_SEMICIRCLE_CURVE_EXIT_BIAS  0.16f   /* 退出弯道阈值（PID切换） */
#define CAR_SEMICIRCLE_CURVE_REENTRY_COOLDOWN_MS 3000 /* 出弯后禁止重新入弯的冷却时间(ms) */
#define CAR_SEMICIRCLE_ANGLE_RELEASE_START 0.0f  /* 角度环开始松手阈值（0=立刻松） */
#define CAR_SEMICIRCLE_ANGLE_RELEASE_END   0.28f /* 角度环完全松手阈值 */
#define CAR_SEMICIRCLE_BIAS_DERIV_GAIN    0.6f  /* 偏差导数抑制系数 */
#define CAR_SEMICIRCLE_SHALLOW_BOOST_BIAS 0.5f  /* 浅弯增强阈值（偏差低于此值放大修正） */
#define CAR_SEMICIRCLE_SHALLOW_BOOST_MAX  2.5f  /* 浅弯最大增强倍数（bias=0时倍数为1+MAX） */
#define CAR_SEMICIRCLE_SPEED_RAMP_UP    90.0f   /* 允许升速斜率(单位: speed/s) */
#define CAR_SEMICIRCLE_SPEED_RAMP_DOWN  260.0f
#define CAR_SEMICIRCLE_OUTER_BOOST      0.00f  /* 外轮加成（0=关） */
#define CAR_SEMICIRCLE_DIFF_SCALE       1.00f  /* 差速缩放（<1 外轮减力，>1 外轮加力） */

/* ---- 默认基础速度 ---- */
#define CAR_DEFAULT_BASE_SPEED  360

/* ---- Key1 半圆循迹 PID（独立调参） ---- */
#define CAR_KEY1_SPEED              450             /* Key1 基础速度400 */
#define CAR_KEY1_SPEED_MIN          360             /* Key1 弯道最低速度 */
#define CAR_KEY1_SPEED_MAX          450             /* Key1 直道最高速度 */
#define CAR_KEY1_CURVE_FOLLOW_KP    45.0f           /* Key1 弯道位置环 KP */
#define CAR_KEY1_CURVE_FOLLOW_KI    0.00f           /* Key1 弯道位置环 KI */
#define CAR_KEY1_CURVE_FOLLOW_KD    0.0f            /* Key1 弯道位置环 KD */
#define CAR_KEY1_STRAIGHT_FOLLOW_KP 2.0f            /* Key1 直道位置环 KP */
#define CAR_KEY1_STRAIGHT_FOLLOW_KI 0.0f            /* Key1 直道位置环 KI */
#define CAR_KEY1_STRAIGHT_FOLLOW_KD 0.0f            /* Key1 直道位置环 KD */
#define CAR_KEY1_ANGLE_KP           0.0f           /* Key1 角度环 KP */
#define CAR_KEY1_ANGLE_KD           0.0f            /* Key1 角度环 KD */
#define CAR_KEY1_CASCADE_INNER_KP           13.0f   /* Key1 内环角速度 KP */
#define CAR_KEY1_CASCADE_INNER_KI           0.0f    /* Key1 内环角速度 KI */
#define CAR_KEY1_CASCADE_INNER_KD           0.0f    /* Key1 内环角速度 KD */
#define CAR_KEY1_CASCADE_CURVE_POS_KP       15.0f   /* Key1 外环弯道位置 KP */
#define CAR_KEY1_CASCADE_CURVE_POS_KI       0.02f   /* Key1 外环弯道位置 KI */
#define CAR_KEY1_CASCADE_CURVE_POS_KD       1.0f    /* Key1 外环弯道位置 KD */
#define CAR_KEY1_CASCADE_STRAIGHT_POS_KP    8.0f    /* Key1 外环直道位置 KP */
#define CAR_KEY1_CASCADE_STRAIGHT_POS_KI    0.02f   /* Key1 外环直道位置 KI */
#define CAR_KEY1_CASCADE_STRAIGHT_POS_KD    1.0f    /* Key1 外环直道位置 KD */
#define CAR_KEY1_CASCADE_W_TARGET_MAX       160.0f  /* Key1 期望角速度上限 */
#define CAR_KEY1_CASCADE2_GYRO_DPS          -45.0f  /* Key1 弯道目标角速度 */
#define CAR_KEY1_CASCADE2_EXIT_ANGLE        175.0f  /* Key1 弯道退出角度 */
#define CAR_KEY1_CASCADE2_F6_ENTRY_FRAMES   3       /* Key1 f6连续帧数 */
#define CAR_KEY1_CASCADE2_SPEED_ADJUST      5       /* Key1 每路调速量 */
#define CAR_KEY1_CASCADE2_SPEED_MIN         300     /* Key1 弯道调速下限 */
#define CAR_KEY1_CASCADE2_SPEED_MAX         400     /* Key1 弯道调速上限 */
#define CAR_KEY1_CASCADE2_LOST_TIMEOUT_MS   500     /* Key1 丢线超时 */

/* ---- Key3 半圆循迹 PID（独立调参） ---- */
#define CAR_KEY3_SPEED              270     /* Key3 基础速度 */
#define CAR_KEY3_SPEED_MIN          270     /* Key3 弯道最低速度 */
#define CAR_KEY3_SPEED_MAX          270     /* Key3 直道最高速度 */
#define CAR_KEY3_STARTUP_SPEED      50      /* Key3 起步速度 */
#define CAR_KEY3_STARTUP_RAMP_MS    600     /* Key3 匀加速时长(ms) */
#define CAR_KEY3_CURVE_FOLLOW_KP    27.0f   /* Key3 弯道位置环 KP 24.0*/
#define CAR_KEY3_CURVE_FOLLOW_KI    0.0f    /* Key3 弯道位置环 KI 0.06*/
#define CAR_KEY3_CURVE_FOLLOW_KD    0.0f    /* Key3 弯道位置环 KD */
#define CAR_KEY3_STRAIGHT_FOLLOW_KP 8.0f    /* Key3 直道位置环 KP */
#define CAR_KEY3_STRAIGHT_FOLLOW_KI 0.0f    /* Key3 直道位置环 KI */
#define CAR_KEY3_STRAIGHT_FOLLOW_KD 0.0f    /* Key3 直道位置环 KD */
#define CAR_KEY3_ANGLE_KP           0.0f   /* Key3 角度环 KP */
#define CAR_KEY3_ANGLE_KD           0.0f    /* Key3 角度环 KD */
#define CAR_KEY3_CASCADE_INNER_KP           15.0f  /* Key3 内环角速度 KP */
#define CAR_KEY3_CASCADE_INNER_KI           0.0f  /* Key3 内环角速度 KI */
#define CAR_KEY3_CASCADE_INNER_KD           0.0f  /* Key3 内环角速度 KD */
#define CAR_KEY3_CASCADE_CURVE_POS_KP       4.0f  /* Key3 外环弯道位置 KP */
#define CAR_KEY3_CASCADE_CURVE_POS_KI       0.0f   /* Key3 外环弯道位置 KI */
#define CAR_KEY3_CASCADE_CURVE_POS_KD       0.0f   /* Key3 外环弯道位置 KD */
#define CAR_KEY3_CASCADE_STRAIGHT_POS_KP    3.0f  /* Key3 外环直道位置 KP */
#define CAR_KEY3_CASCADE_STRAIGHT_POS_KI    0.0f  /* Key3 外环直道位置 KI */
#define CAR_KEY3_CASCADE_STRAIGHT_POS_KD    0.0f   /* Key3 外环直道位置 KD */
#define CAR_KEY3_CASCADE_W_TARGET_MAX       120.0f /* Key3 期望角速度上限 */
#define CAR_KEY3_CASCADE2_GYRO_DPS          -34.0f /* Key3 弯道目标角速度 */
#define CAR_KEY3_CASCADE2_EXIT_ANGLE        180.0f /* Key3 弯道退出角度 */
#define CAR_KEY3_CASCADE2_F6_ENTRY_FRAMES   3      /* Key3 f6连续帧数 */
#define CAR_KEY3_CASCADE2_SPEED_ADJUST      0     /* Key3 每路调速量 */
#define CAR_KEY3_CASCADE2_SPEED_MIN         250    /* Key3 弯道调速下限 */
#define CAR_KEY3_CASCADE2_SPEED_MAX         310    /* Key3 弯道调速上限 */
#define CAR_KEY3_CASCADE2_LOST_TIMEOUT_MS   500    /* Key3 丢线超时 */

/* ---- Key2 半圆循迹 PID（独立调参，7s超时） ---- */
#define CAR_KEY2_SPEED              250     /* Key2 基础速度 */
#define CAR_KEY2_SPEED_MIN          250     /* Key2 弯道最低速度 */
#define CAR_KEY2_SPEED_MAX          250     /* Key2 直道最高速度 */
#define CAR_KEY2_STARTUP_SPEED      20     /* Key2 起步速度 */
#define CAR_KEY2_STARTUP_RAMP_MS    800    /* Key2 匀加速时长(ms) */
#define CAR_KEY2_CURVE_FOLLOW_KP    0.0f   /* Key2 弯道位置环 KP */
#define CAR_KEY2_CURVE_FOLLOW_KI    0.0f   /* Key2 弯道位置环 KI */
#define CAR_KEY2_CURVE_FOLLOW_KD    0.0f    /* Key2 弯道位置环 KD */
#define CAR_KEY2_STRAIGHT_FOLLOW_KP 8.0f   /* Key2 直道位置环 KP */
#define CAR_KEY2_STRAIGHT_FOLLOW_KI 0.0f    /* Key2 直道位置环 KI */
#define CAR_KEY2_STRAIGHT_FOLLOW_KD 0.0f    /* Key2 直道位置环 KD */
#define CAR_KEY2_ANGLE_KP           10.0f   /* Key2 角度环 KP */
#define CAR_KEY2_ANGLE_KD           4.0f    /* Key2 角度环 KD */
#define CAR_KEY2_CASCADE_INNER_KP           15.0f  /* Key2 内环角速度 KP */
#define CAR_KEY2_CASCADE_INNER_KI           0.0f  /* Key2 内环角速度 KI */
#define CAR_KEY2_CASCADE_INNER_KD           0.0f   /* Key2 内环角速度 KD */
#define CAR_KEY2_CASCADE_CURVE_POS_KP       12.0f  /* Key2 外环弯道位置 KP */
#define CAR_KEY2_CASCADE_CURVE_POS_KI       0.02f   /* Key2 外环弯道位置 KI */
#define CAR_KEY2_CASCADE_CURVE_POS_KD       0.0f   /* Key2 外环弯道位置 KD */
#define CAR_KEY2_CASCADE_STRAIGHT_POS_KP    6.0f  /* Key2 外环直道位置 KP */
#define CAR_KEY2_CASCADE_STRAIGHT_POS_KI    0.02f  /* Key2 外环直道位置 KI */
#define CAR_KEY2_CASCADE_STRAIGHT_POS_KD    0.0f   /* Key2 外环直道位置 KD */
#define CAR_KEY2_CASCADE_W_TARGET_MAX       130.0f /* Key2 期望角速度上限 */
#define CAR_KEY2_CASCADE2_GYRO_DPS          -30.0f /* Key2 弯道目标角速度 */
#define CAR_KEY2_CASCADE2_EXIT_ANGLE        175.0f /* Key2 弯道退出角度 */
#define CAR_KEY2_CASCADE2_F6_ENTRY_FRAMES   3      /* Key2 f6连续帧数 */
#define CAR_KEY2_CASCADE2_SPEED_ADJUST      5      /* Key2 每路调速量 */
#define CAR_KEY2_CASCADE2_SPEED_MIN         230    /* Key2 弯道调速下限 */
#define CAR_KEY2_CASCADE2_SPEED_MAX         310    /* Key2 弯道调速上限 */
#define CAR_KEY2_CASCADE2_LOST_TIMEOUT_MS   500    /* Key2 丢线超时 */
#define CAR_KEY2_TIMEOUT_MS         8000    /* Key2 超时 8s */

/* ---- 角速度环 PID (gyro_rate_control.h / gyro_rate_test.c) ---- */
#define CAR_GYRO_RATE_KP             15.0f
#define CAR_GYRO_RATE_KI             0.0f
#define CAR_GYRO_RATE_KD             0.0f
#define CAR_GYRO_RATE_INTEGRAL_LIMIT 60.0f
#define CAR_GYRO_RATE_OUTPUT_LIMIT   150.0f

/* ---- 角速度环测试参数 (gyro_rate_test.c) ---- */
#define GYRO_TEST_TARGET_DPS        -30.0f   /* 目标角速度 (°/s) */
#define GYRO_TEST_BASE_SPEED        280     /* 基础速度 */
#define GYRO_TEST_TOTAL_ANGLE       180.0f  /* 累计角度阈值 (°) */
#define GYRO_TEST_TIMEOUT_MS        7000    /* 超时保护 (ms) */

/* ---- 串级 PID 参数 (semicircle_follow_plus.h) ---- */
#define CAR_CASCADE_CURVE_POS_KP         12.0f   /* 外环弯道位置PID KP */
#define CAR_CASCADE_CURVE_POS_KI         0.02f    /* 外环弯道位置PID KI */
#define CAR_CASCADE_CURVE_POS_KD         0.0f    /* 外环弯道位置PID KD */
#define CAR_CASCADE_STRAIGHT_POS_KP      6.0f   /* 外环直道位置PID KP */
#define CAR_CASCADE_STRAIGHT_POS_KI      0.02f    /* 外环直道位置PID KI */
#define CAR_CASCADE_STRAIGHT_POS_KD      0.0f    /* 外环直道位置PID KD */
#define CAR_CASCADE_INNER_KP             12.0f   /* 内环角速度PID KP */
#define CAR_CASCADE_INNER_KI             0.0f   /* 内环角速度PID KI */
#define CAR_CASCADE_INNER_KD             0.0f    /* 内环角速度PID KD */
#define CAR_CASCADE_INNER_INTEGRAL_LIMIT 70.0f   /* 内环积分上限 */
#define CAR_CASCADE_INNER_OUTPUT_LIMIT   180.0f  /* 内环输出上限(差速) */
#define CAR_CASCADE_W_TARGET_MAX         130.0f  /* 期望角速度上限(°/s) */

#else
#error "CAR_SELECT must be 0 or 1"
#endif

#endif  /* _CAR_CONFIG_H_ */