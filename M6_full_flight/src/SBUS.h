/*
 * ============================================================================
 * 遥控接收模块 —— FlySky iBus（FS-iA6B）解析（原 SBUS，接口兼容保留）
 * ----------------------------------------------------------------------------
 * 【v0.25.1 重要变更】iBus 改用 RMT 硬件解码，不占用任何 UART 串口：
 *   - 接收机信号线接 GPIO18（与早期版本相同，不用改线）
 *   - 三个 UART 均已被占用：UART0=USB 串口(CP2102N→GPIO43/44，Serial 打印)、
 *     UART1=数传 LINK(GPIO14/15)、UART2=GPS(GPIO16/17)；
 *     原 v0.25 曾改 UART0/GPIO44 方案，实测会与板载 USB 串口抢外设，已废弃
 *   - RMT 硬件采集 19200 波特串口波形（1 tick=1µs，DMA 缓冲 512 symbols），
 *     在 sbus_poll() 中软件解析帧，与 Serial/GPS/数传零冲突
 * 接口（与旧 SBUS 调用完全一致）：
 *   void sbus_begin();         初始化 RMT RX（GPIO18）
 *   void sbus_poll();          在主循环调用：若有新帧则解析，内部维护 rc_ch[]
 *   bool sbus_get_lost();      丢信号/失控保护标志（>1s 无帧）
 * 全局输出：rc_ch[16]（iBus 通道值 1000~2000，中位约 1500；无效通道保持原值）
 * ============================================================================
 */
#ifndef SBUS_H
#define SBUS_H

#include <Arduino.h>

#define SBUS_RX_PIN    18        // iBus 信号（RMT RX，无需反相）
#define SBUS_BAUD      19200     // iBus 波特率
#define SBUS_FRAME_LEN 32        // iBus 帧长（2 帧头 + 28 通道 + 2 校验）

extern uint16_t rc_ch[16];   // 16 通道原始值

void sbus_begin();
void sbus_poll();
bool sbus_get_lost();
bool sbus_parse(const uint8_t *b);
uint8_t sbus_signal_pct(void);   // 信号质量 0~100（帧接收率）

#endif
