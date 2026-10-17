/*
 * ============================================================================
 * LINK 功能模块 —— 433MHz 数传链路（UART1）
 * ----------------------------------------------------------------------------
 * 上行：10Hz 遥测 JSON 行 → 数传模块 TX → 地面端（电脑/手机地面站）
 * 下行：地面站命令（WP:ADD/CLEAR/START/STOP、D 日志请求）→ 数传 RX
 *
 * 引脚：TX=GPIO14  RX=GPIO15；波特率 57600 8N1（与地面站默认一致）
 * 接线：数传模块 RX ← 主控 GPIO14(TX)；数传模块 TX → 主控 GPIO15(RX)
 *       （数传 5V 供电，逻辑电平 3.3V 兼容）
 * ============================================================================
 */
#ifndef LINK_H
#define LINK_H

#include <Arduino.h>

#define LINK_BAUD       57600
#define LINK_TX_PIN     14
#define LINK_RX_PIN     15

void link_begin();                 // setup 调用一次
void link_send(const String &line); // 发一行（自动补换行）
bool link_read_line(String &out);   // 非阻塞收一行命令，收到返回 true

#endif
