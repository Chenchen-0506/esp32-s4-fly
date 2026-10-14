/*
 * ============================================================================
 * ALTHOLD 功能模块 —— 定高（Altitude Hold）
 * ----------------------------------------------------------------------------
 * 参考开源：ArduPilot AltHold 控制结构（简化两环）
 *   高度误差 --P--> 期望升降速度 --PID--> 油门补偿（叠加悬停油门）
 *   油门杆 = 升降速度指令（中位死区 30µs，杆推前=爬升）
 *   悬停油门：进入时取手动模式当前油门为基准，积分项自适应修正
 *
 * 注意：垂速由气压高度差分 + 低通估计（无 EKF），20ms 节流更新，
 *       试飞时若上下抖动先降 VEL_P，反应慢则升 ALT_P。
 * ============================================================================
 */
#ifndef ALTHOLD_H
#define ALTHOLD_H

#include <Arduino.h>

/* ---- 定高参数（保守默认，试飞调）---- */
#define ALT_P        0.8f      // 高度误差→垂速（/s）：1m 误差 → 0.8 m/s
#define ALT_STICK_K  0.20f     // 油门杆→垂速（m/s per µs，死区外）
#define ALT_STICK_DB 30.0f     // 油门杆死区（µs）
#define ALT_VS_MAX   150.0f    // 目标垂速限幅（cm/s）
#define VEL_P        120.0f    // 垂速误差→油门（µs per m/s）
#define VEL_I        40.0f     // 垂速积分→油门（µs per (m/s)/s）
#define THR_I_LIMIT  250.0f    // 积分限幅（µs）
#define VS_LP        0.10f     // 垂速低通系数（越小越平滑但越延迟）
#define ALT_STEP_MS  20UL      // 控制更新节流（与气压差分同步）

void     althold_begin();
void     althold_start(float rel_alt, uint16_t hover_thr_us);
bool     althold_active();
void     althold_stop();
uint16_t althold_update(float rel_alt, float stick_thr);

#endif
