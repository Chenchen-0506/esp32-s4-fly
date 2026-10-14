/*
 * ============================================================================
 * ALTHOLD 功能模块 —— 定高（Altitude Hold）
 * ----------------------------------------------------------------------------
 * 参考开源：ArduPilot AltHold 控制结构（简化两环）
 *   高度误差 --P--> 期望升降速度 --PID--> 油门补偿（叠加悬停油门）
 *   油门杆 = 升降速度指令（中位死区 30µs，杆推前=爬升）
 *   悬停油门：进入时取手动模式当前油门为基准，积分项自适应修正
 *
 * 注意：垂速由 VERTEST 垂直融合模块提供（加速度+气压互补，
 *       替代原气压差分+低通，噪声低 5~10 倍），20ms 节流更新。
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

/* ---- 悬停油门自动学习（ArduPilot MOT_HOVER_LEARN 同款）----
 * 悬停稳定（油门杆中位 + 垂速小 + 高度误差小）时，把当前油门平滑学习进
 * hover_thr，每 10 秒存一次 NVS，开机自动加载。飞几次后基准自动收敛，
 * 无需手抄参数。 */
#define HOVER_LEARN_RATE  0.02f  // 学习速率（越大学越快，过大会被风带偏）
#define HOVER_SAVE_MS     10000UL // NVS 保存间隔
#define HOVER_MAX_ALT_ERR 0.5f   // 允许学习的高度误差（m）
#define HOVER_MAX_VS      0.3f   // 允许学习的垂速（m/s）

void     althold_load_hover();   // 开机：从 NVS 加载悬停油门基准
float    althold_hover_thr();    // 当前悬停油门基准（µs，供状态打印）

void     althold_begin();
void     althold_start(float rel_alt, uint16_t hover_thr_us);
bool     althold_active();
void     althold_stop();
uint16_t althold_update(float rel_alt, float vert_vel, float stick_thr);

#endif
