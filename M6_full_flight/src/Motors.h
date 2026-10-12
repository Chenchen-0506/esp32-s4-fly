/*
 * ============================================================================
 * Motors 功能模块 —— 4 路电调 PWM 输出
 * ----------------------------------------------------------------------------
 * 来源：M4_esc_test / M6_full_flight 已验证代码，原样提取
 * 引脚：M1=4  M2=5  M3=6  M4=7（50Hz，1000-2000µs）
 * 接口：
 *   void motors_begin();                初始化 4 路 PWM，默认油门 1000（停转）
 *   void set_motor_us(idx, us);         设置单个电机油门（自动限幅 1000~2000）
 *   void set_all_motors(us);            设置全部电机油门
 * 全局输出：motor_us[4]（当前油门，供状态打印）
 * ============================================================================
 */
#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>

#define M1_PIN 4
#define M2_PIN 5
#define M3_PIN 6
#define M4_PIN 7

#define MIN_US        1000
#define MAX_US        2000
#define PWM_RES_BITS  14
#define PERIOD_US     20000

extern uint16_t motor_us[4];

void motors_begin();
void set_motor_us(uint8_t idx, uint16_t us);
void set_all_motors(uint16_t us);

#endif
