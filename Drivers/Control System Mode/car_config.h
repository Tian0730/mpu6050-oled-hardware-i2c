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
#define CAR_SEMICIRCLE_CURVE_ENTER_BIAS 0.38f   /* 进入弯道阈值（PID切换） */
#define CAR_SEMICIRCLE_CURVE_EXIT_BIAS  0.18f   /* 退出弯道阈值（PID切换） */
#define CAR_SEMICIRCLE_ANGLE_RELEASE_START 0.06f /* 角度环开始松手阈值（低于此偏差角度环100%） */
#define CAR_SEMICIRCLE_ANGLE_RELEASE_END   0.22f /* 角度环完全松手阈值（高于此偏差角度环0%） */
#define CAR_SEMICIRCLE_BIAS_DERIV_GAIN    0.5f  /* 偏差导数抑制系数 */
#define CAR_SEMICIRCLE_DEEP_CURVE_BIAS    1.5f  /* 深弯道阈值 */
#define CAR_SEMICIRCLE_DEEP_CURVE_SCALE   0.35f /* 深弯道衰减系数 */
#define CAR_SEMICIRCLE_SPEED_MIN        190     /* 弯道最低基速 */
#define CAR_SEMICIRCLE_SPEED_MAX        250     /* 直道最高基速 */
#define CAR_SEMICIRCLE_SPEED_RAMP_UP    120.0f  /* 允许升速斜率(单位: speed/s) */
#define CAR_SEMICIRCLE_SPEED_RAMP_DOWN  300.0f  /* 允许降速斜率(单位: speed/s) */

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
#define CAR_STRAIGHT_KP         35.0f
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
#define CAR_SEMICIRCLE_SPEED            300     /* 半圆循线基础速度 */
#define CAR_SEMICIRCLE_CURVE_FOLLOW_KP  39.0f   /* 弯道位置环 KP */
#define CAR_SEMICIRCLE_CURVE_FOLLOW_KI  0.01f   /* 弯道位置环 KI*/
#define CAR_SEMICIRCLE_CURVE_FOLLOW_KD  0.0f    /* 弯道位置环 KD（量化传感器导数=噪声，必须为0） */
#define CAR_SEMICIRCLE_STRAIGHT_FOLLOW_KP  30.0f   /* 直道位置环 KP（与弯道接近，消除切换延迟） */
#define CAR_SEMICIRCLE_STRAIGHT_FOLLOW_KI  0.0f   /* 直道位置环 KI（入弯处提前积累积分）5 */
#define CAR_SEMICIRCLE_STRAIGHT_FOLLOW_KD  0.0f    /* 直道位置环 KD（同上） */
#define CAR_SEMICIRCLE_ANGLE_SUPPRESS   0.3f    /* 角度环抑制系数（0=完全关闭，1=全开） */
#define CAR_SEMICIRCLE_LOST_TIMEOUT_MS  1000    /* 丢线超时停止(ms) */
#define CAR_SEMICIRCLE_INTEGRAL_DECAY   0.85f   /* 直道积分衰减系数(每帧, 0=瞬清, 1=不清) */
#define CAR_SEMICIRCLE_STOP_MIN_BLACK   6       /* 启停线检测最少黑点数(6=8路中至少6路见黑) */
#define CAR_SEMICIRCLE_STOP_COOLDOWN_MS 5000    /* 启停线冷却时间(ms)，防止起步误触发 */
#define CAR_SEMICIRCLE_CURVE_ENTER_BIAS 0.3f    /* 进入弯道阈值（PID切换） */
#define CAR_SEMICIRCLE_CURVE_EXIT_BIAS  0.16f   /* 退出弯道阈值（PID切换） */
#define CAR_SEMICIRCLE_ANGLE_RELEASE_START 0.0f  /* 角度环开始松手阈值（0=立刻松） */
#define CAR_SEMICIRCLE_ANGLE_RELEASE_END   0.18f /* 角度环完全松手阈值 */
#define CAR_SEMICIRCLE_BIAS_DERIV_GAIN    0.6f  /* 偏差导数抑制系数 */
#define CAR_SEMICIRCLE_SHALLOW_BOOST_BIAS 0.5f  /* 浅弯增强阈值（偏差低于此值放大修正） */
#define CAR_SEMICIRCLE_SHALLOW_BOOST_MAX  2.5f  /* 浅弯最大增强倍数（bias=0时倍数为1+MAX） */
#define CAR_SEMICIRCLE_DEEP_CURVE_BIAS    1.5f  /* 深弯道阈值（偏差超此值开始衰减修正量） */
#define CAR_SEMICIRCLE_DEEP_CURVE_SCALE   0.35f /* 深弯道衰减系数（每单位偏差衰减35%，最低保留25%） */
#define CAR_SEMICIRCLE_SPEED_MIN        280     /* 弯道最低基速 */
#define CAR_SEMICIRCLE_SPEED_MAX        310     /* 直道最高基速 */
#define CAR_SEMICIRCLE_SPEED_RAMP_UP    90.0f   /* 允许升速斜率(单位: speed/s) */
#define CAR_SEMICIRCLE_SPEED_RAMP_DOWN  260.0f
#define CAR_SEMICIRCLE_OUTER_BOOST      0.30f  /* 外侧轮子额外加速比例 */

/* ---- 默认基础速度 ---- */
#define CAR_DEFAULT_BASE_SPEED  300

#else
#error "CAR_SELECT must be 0 or 1"
#endif

#endif  /* _CAR_CONFIG_H_ */