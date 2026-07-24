#ifndef _CONTROL_CONFIG_H_
#define _CONTROL_CONFIG_H_

/* ================================================================
 *  控制环配置开关
 *  1 = 启用, 0 = 禁用
 *  可独立启用角度环/位置环/速度环，也可同时启用（多环控制）
 *
 *  调试建议：
 *    单独调试速度环 → USE_SPEED_CONTROL=1, 其余=0
 *    单独调试角度环 → USE_ANGLE_CONTROL=1,  其余=0
 *    单独调试位置环 → USE_FOLLOW_CONTROL=1, 其余=0
 *    双环(角度+位置) → 两个都=1
 * ================================================================ */
#define USE_ANGLE_CONTROL   1   // 角度环控制 (MPU6050 + turn)
#define USE_FOLLOW_CONTROL  1   // 位置环控制 (灰度传感器 + follow)
#define USE_SPEED_CONTROL   1   // 速度环控制 (编码器 + speed_control)

/* ================================================================
 * 超声波模块启用开关
 * 1 = 避障模式, 0 = 循迹模式
 * ================================================================ */
#define USE_OBSTACLE_AVOIDANCE  0   // 模式切换

#define USE_VOFA_DEBUG          0   // VOFA+串口在线调参（暂时关闭排查传感器）

#endif  /* _CONTROL_CONFIG_H_ */