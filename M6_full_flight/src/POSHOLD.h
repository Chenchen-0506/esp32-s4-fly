/*
 * ============================================================================
 * POSHOLD 功能模块 —— 悬停（Position Hold）
 * ----------------------------------------------------------------------------
 * 参考开源：ArduPilot Loiter 控制结构（简化）
 *   位置误差（GPS）--P--> 倾角指令（死区防 GPS 噪声抖动，限幅 15°）
 *   摇杆 = 移动悬停目标点（速度积分），释放后停在最新位置保持
 *   航向锁定：进入时记录偏航，悬停中缓慢修正回目标航向
 *
 * 简化点：跳过速度环（无 EKF 速度估计），位置误差直接映射倾角；
 *         GPS ±2-3m 噪声已用 1.5m 死区压制，风大时漂移无法完全消除。
 * ============================================================================
 */
#ifndef POSHOLD_H
#define POSHOLD_H

#include <Arduino.h>

/* ---- 悬停参数（保守默认，试飞调）---- */
#define PH_DEADBAND_M  1.5f     // 位置死区（m），压制 GPS 噪声
#define PH_POS_TILT    3.5f     // 位置误差→倾角（°/m）
#define PH_ANG_MAX     15.0f    // 倾角限幅（°）
#define PH_STICK_VEL   0.0025f  // 摇杆→目标速度（m/s per µs）
#define PH_STICK_MAX   1.2f     // 摇杆搬移速度限幅（m/s）
#define PH_YAW_P       2.0f     // 航向误差→偏航率（°/s per °）
#define PH_YAW_RATE_MAX 60.0f   // 偏航率限幅（°/s）

void poshold_begin();
void poshold_start(float lat, float lon, float heading);
bool poshold_active();
void poshold_stop();
void poshold_update(float lat, float lon, float heading,
                    float pitch_stick, float roll_stick, float dt,
                    float &pitch_cmd, float &roll_cmd, float &yaw_rate_cmd);

#endif
