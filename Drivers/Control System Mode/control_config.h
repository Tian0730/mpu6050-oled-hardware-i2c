#ifndef _CONTROL_CONFIG_H_
#define _CONTROL_CONFIG_H_

/* ================================================================
 *  控制环配置开关
 *  1 = 启用, 0 = 禁用
 *  可独立启用角度环或位置环，也可同时启用（双环控制）
 *  目前只有位置环、角度环
 * ================================================================ */
#define USE_ANGLE_CONTROL   0   // 角度环控制 (MPU6050 + turn)
#define USE_FOLLOW_CONTROL  1   // 位置环控制 (灰度传感器 + follow)

#endif  /* _CONTROL_CONFIG_H_ */