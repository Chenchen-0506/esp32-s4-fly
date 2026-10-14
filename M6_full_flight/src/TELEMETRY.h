/*
 * ============================================================================
 * TELEMETRY 功能模块 —— WiFi 实时数据看板（方案 A）
 * ----------------------------------------------------------------------------
 * 飞行时无需 USB：ESP32-S3 开热点，手机/电脑连上后浏览器打开
 *   http://192.168.4.1
 *   实时滚动显示姿态/高度/GPS/返航状态（每秒刷新），可一键下载记录。
 *
 * 路由：
 *   /        看板页面
 *   /feed?since=N  返回行号>N 的新日志行（增量，纯文本 seq\t行）
 *   /download 下载黑匣子完整记录（text/plain）
 * ============================================================================
 */
#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <Arduino.h>

#define TEL_SSID      "S4Fly-Telemetry"
#define TEL_PASS      "12345678"

void telemetry_begin();   // 开热点 + 启动 Web 服务（setup 调用一次）
void telemetry_poll();    // 主循环调用（内部处理 HTTP，开销小）

#endif
