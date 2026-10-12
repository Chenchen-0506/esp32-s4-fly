/*
 * ============================================================================
 * SBUS 功能模块 —— 遥控器信号解析（UART 软件反相）
 * ----------------------------------------------------------------------------
 * 来源：M3_remote_sbus / M6_full_flight 已验证代码，原样提取
 * 引脚：RX=GPIO18；100000 baud，8E2，软件反相（无需反相器）
 * 接口：
 *   void sbus_begin();         初始化 Serial2（GPIO18）
 *   void sbus_poll();          在主循环调用：收帧并解析，内部维护 rc_ch[]
 *   bool sbus_get_lost();      丢信号/失控保护标志（>1s 无帧或 failsafe）
 * 全局输出：rc_ch[16]（SBUS 原始值 1000~2000 附近，中位约 1500）
 * ============================================================================
 */
#ifndef SBUS_H
#define SBUS_H

#include <Arduino.h>

#define SBUS_RX_PIN    18
#define SBUS_BAUD      100000
#define SBUS_FRAME_LEN 25

extern uint16_t rc_ch[16];   // 16 通道原始值
extern bool rc_failsafe;

void sbus_begin();
void sbus_poll();
bool sbus_get_lost();
bool sbus_parse(const uint8_t *b);

#endif
