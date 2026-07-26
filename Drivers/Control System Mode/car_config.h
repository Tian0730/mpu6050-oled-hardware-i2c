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
#define CAR_SELECT  0

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

/* ---- 循线预转/冷却 (main.c) ---- */
#define CAR_PRE_TURN_MS         300
#define CAR_TURN_COOLDOWN       500

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
#define CAR_TURN_BASE_SPEED     100
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

/* ---- 循线预转/冷却 (main.c) ---- */
#define CAR_PRE_TURN_MS         300
#define CAR_TURN_COOLDOWN       500

/* ---- 默认基础速度 ---- */
#define CAR_DEFAULT_BASE_SPEED  100

#else
#error "CAR_SELECT must be 0 or 1"
#endif

#endif  /* _CAR_CONFIG_H_ */