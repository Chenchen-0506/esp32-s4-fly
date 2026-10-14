/*
 * ============================================================================
 * GPS 功能模块 —— BN-880 NMEA 解析（UART）
 * ----------------------------------------------------------------------------
 * 来源：M5_gps_test 已验证代码，抽成模块。仅解析 GGA 语句。
 * 引脚：RX=GPIO16  TX=GPIO17；波特率自动扫描（9600/38400/57600/115200）
 * 接口：
 *   void gps_begin();              初始化并自动锁定波特率
 *   void gps_poll();               主循环调用：收数据 + 解析 GGA
 *   bool gps_have_fix();           是否已定位（fix_quality > 0）
 *   全局输出：gps_lat/gps_lon/gps_alt/gps_fix/gps_sats
 * 说明：室内搜不到星属正常；到窗边/室外等 1-2 分钟看 gps_fix 变 1。
 * ============================================================================
 */
#ifndef GPS_H
#define GPS_H

#include <Arduino.h>

#define GPS_RX_PIN  16
#define GPS_TX_PIN  17

extern float    gps_lat, gps_lon, gps_alt;   // 度 / 度 / 米
extern bool     gps_fix;                     // 是否定位
extern uint8_t  gps_sats;                    // 卫星数

void gps_begin();
void gps_poll();

#endif
