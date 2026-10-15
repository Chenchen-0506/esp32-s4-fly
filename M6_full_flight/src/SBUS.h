/*
 * ============================================================================
 * 遥控接收模块 —— FlySky iBus（FS-iA6B）解析（原 SBUS，接口兼容保留）
 * ----------------------------------------------------------------------------
 * 引脚：RX=GPIO18；19200 baud，8N1（iBus 协议，无需反相）
 * 接口（与旧 SBUS 调用完全一致）：
 *   void sbus_begin();         初始化 Serial2（GPIO18）
 *   void sbus_poll();          在主循环调用：收帧并解析，内部维护 rc_ch[]
 *   bool sbus_get_lost();      丢信号/失控保护标志（>1s 无帧）
 * 全局输出：rc_ch[16]（iBus 通道值 1000~2000，中位约 1500；无效通道保持原值）
 * ============================================================================
 */
#ifndef SBUS_H
#define SBUS_H

#include <Arduino.h>

#define SBUS_RX_PIN    18
#define SBUS_BAUD      19200
#define SBUS_FRAME_LEN 32

extern uint16_t rc_ch[16];   // 16 通道原始值
extern bool rc_failsafe;

void sbus_begin();
void sbus_poll();
bool sbus_get_lost();
bool sbus_parse(const uint8_t *b);

#endif
