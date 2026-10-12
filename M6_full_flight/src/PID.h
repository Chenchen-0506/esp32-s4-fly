/*
 * ============================================================================
 * PID 功能模块 —— 串级 PID 控制器（角度外环 -> 角速度内环）
 * ----------------------------------------------------------------------------
 * 来源：M6_full_flight 已验证代码，原样提取
 * 说明：本模块实现通用的 PID 结构体与计算；外环/内环实例由主程序创建，
 *       参数宏集中在此文件便于统一调参。
 * ============================================================================
 */
#ifndef PID_H
#define PID_H

#include <Arduino.h>

/* ---- 角速度内环 PID 参数（保守默认，飞起来再微调）---- */
#define RATE_P_ROLL    8.0f
#define RATE_I_ROLL    0.1f
#define RATE_D_ROLL    0.05f
#define RATE_P_PITCH   8.0f
#define RATE_I_PITCH   0.1f
#define RATE_D_PITCH   0.05f
#define RATE_P_YAW     6.0f
#define RATE_I_YAW     0.05f
#define RATE_D_YAW     0.0f

#define RATE_I_LIMIT   50.0f    // 积分限幅，防饱和

struct PID {
  float kp, ki, kd;
  float i_term, prev_err;
};

void  pid_init(PID *p, float kp, float ki, float kd);
float pid_calc(PID *p, float err, float dt);

#endif
