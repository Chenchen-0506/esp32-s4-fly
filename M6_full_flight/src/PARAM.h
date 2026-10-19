/*
 * ============================================================================
 * PARAM 功能模块 —— 参数在线调整（PARAM:SET，数传/串口下行）
 * ----------------------------------------------------------------------------
 * 支持：地面站/串口发送 `PARAM:SET key=value`（立即生效 + NVS 持久化）、
 *       `PARAM:GET`（返回全部参数 JSON）、`PARAM:RESET`（恢复默认）。
 * 参数变量全部为全局 float，各功能模块（PID/ALTHOLD/RTL/主程序）extern 引用，
 * 无需重新编译烧录即可在飞行前/飞行中微调 PID、返航高度、定高增益等。
 * 默认值来源：各模块头文件的 #define（PARAM.cpp include 后取宏初值）。
 * ============================================================================
 */
#ifndef PARAM_H
#define PARAM_H

#include <Arduino.h>

/* ---- 默认值（原宏集中定义，避免两处漂移）---- */
#define ANGLE_P_ROLL    4.0f
#define ANGLE_P_PITCH   4.0f
#define ANGLE_P_YAW     2.0f
#define MANU_MAX_TILT   25.0f    // 自稳模式最大倾斜角（度）
#define MANU_YAW_RATE   100.0f   // 自稳模式最大偏航速率（度/秒）

/* ---- 可调参数（全局变量，各模块 extern 引用）---- */
extern float rate_p_roll,  rate_i_roll,  rate_d_roll;
extern float rate_p_pitch, rate_i_pitch, rate_d_pitch;
extern float rate_p_yaw,   rate_i_yaw,   rate_d_yaw;
extern float rate_i_limit;                 // 内环积分限幅
extern float angle_p_roll, angle_p_pitch, angle_p_yaw;   // 角度外环 P
extern float alt_p, alt_stick_k, alt_stick_db, alt_vs_max;
extern float vel_p, vel_i, thr_i_limit;    // 垂速内环 + 油门积分
extern float rtl_alt, rtl_near;
extern float rtl_climb_thr, rtl_cruise_thr, rtl_descend_thr;
extern float rtl_max_tilt, rtl_pitch_p, rtl_yaw_p;
extern float max_tilt_manu, yaw_rate_manu;

void   param_begin();                       // setup：NVS 加载全部参数
float  param_get(const char *key);          // 读当前值（未注册返回 NAN）
bool   param_set(const char *key, float v); // 写值 + NVS 保存
void   param_reset();                       // 全部恢复默认并清除 NVS
String param_json();                        // 所有参数 JSON {"key":val,...}

#endif
