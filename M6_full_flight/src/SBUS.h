/*
 * ============================================================================
 * 遥控接收模块 —— FlySky iBus（FS-iA6B）解析（原 SBUS，接口兼容保留）
 * ----------------------------------------------------------------------------
 * 引脚：RX=GPIO44（UART0；注意 GPS 已占用 UART2/Serial2(16/17)，iBus 改走 UART0
 *       ——GPIO43/44 为 ESP32-S3 UART0 物理引脚，与 USB-CDC(GPIO19/20)独立，互不冲突）
 * 波特率：19200 baud，8N1（iBus 协议，无需反相）
 * 接口（与旧 SBUS 调用完全一致）：
 *   void sbus_begin();         初始化 UART0（GPIO44）
 *   void sbus_poll();          在主循环调用：收帧并解析，内部维护 rc_ch[]
 *   bool sbus_get_lost();      丢信号/失控保护标志（>1s 无帧）
 * 全局输出：rc_ch[16]（iBus 通道值 1000~2000，中位约 1500；无效通道保持原值）
 * ============================================================================
 */
#ifndef SBUS_H
#define SBUS_H

#include <Arduino.h>

#define SBUS_RX_PIN    44
#define SBUS_BAUD      19200
#define SBUS_FRAME_LEN 32

extern uint16_t rc_ch[16];   // 16 通道原始值

void sbus_begin();
void sbus_poll();
bool sbus_get_lost();
bool sbus_parse(const uint8_t *b);
uint8_t sbus_signal_pct(void);   // 信号质量 0~100（帧接收率）

#endif
