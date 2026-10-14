/*
 * ============================================================================
 * VERTEST 功能模块 —— 垂直状态估计（高度 + 垂速融合）
 * ----------------------------------------------------------------------------
 * 参考开源：ArduPilot/PX4 垂直通道融合思路（简化 1D 互补/低通卡尔曼）
 *   预测（1kHz）：垂直加速度积分 -> 垂速 -> 高度
 *   修正（10Hz） ：BMP280 气压高度观测，按增益拉回（低通气压噪声、
 *                  拉住加速度积分的长期漂移）
 *
 * 假设：小倾角飞行（≤15°，定高/悬停场景成立），机体 z 轴近似重力方向，
 *       垂直加速度 = (acc_z_g - 1) * 9.80665（向上为正，与高度符号一致）。
 *
 * 输出：
 *   vertest_alt()   融合高度（m，正=高于起飞点）
 *   vertest_vel()   融合垂速（m/s，正=上升）
 * ============================================================================
 */
#ifndef VERTEST_H
#define VERTEST_H

#include <Arduino.h>

#define VERT_ACC_MAX   15.0f    // 垂直加速度限幅（m/s²，防异常值）
#define VERT_VEL_MAX   5.0f     // 垂速限幅（m/s，防积分漂移爆量）
#define VERT_K_ALT     0.10f    // 高度修正增益（越大越信气压）
#define VERT_K_VEL     0.04f    // 垂速修正增益（小：垂速主要靠加速度积分）

void  vertest_begin();
void  vertest_reset(float alt0);       // 以当前高度初始化（BMP 地面定高后）
void  vertest_predict(float acc_z_g, float dt);  // 每环（1kHz）
void  vertest_correct(float bmp_alt);  // 气压更新时（10Hz）
float vertest_alt();
float vertest_vel();

#endif
