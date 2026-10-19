/*
 * ============================================================================
 * TELEMETRY 功能模块 —— WiFi 实时数据看板（仪表盘版）
 * ----------------------------------------------------------------------------
 * 飞行时无需 USB：ESP32-S3 开热点，手机/电脑连上后浏览器打开
 *   http://192.168.4.1
 *   仪表盘实时显示姿态/高度/航向/GPS/RC/电机（每秒刷新），日志滚动保留。
 *
 * 路由：
 *   /        仪表盘页面
 *   /api     最新状态 JSON（结构化，仪表盘轮询用）
 *   /feed?since=N  返回行号>N 的新日志行（增量，纯文本 seq\t行）
 *   /download 下载黑匣子完整记录（text/plain）
 * ============================================================================
 */
#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <Arduino.h>

#define TEL_SSID      "S4Fly-Telemetry"
#define TEL_PASS      "12345678"

/* 最新状态快照（主程序 10Hz 填充，/api 读取） */
typedef struct {
  uint8_t  armed;        // 0/1 解锁
  char     mode[12];     // MANU / RTL-* / POSHOLD / MAGCAL
  float    roll;         // deg
  float    pitch;        // deg
  float    yaw;          // deg
  float    alt;          // m（相对起飞点）
  float    vspd;         // 垂直速度 m/s（+上升，垂直融合）
  float    spd;          // 水平地速 m/s（GPS 位置差分）
  uint16_t rc[7];        // 通道 1~7
  uint8_t  gps_fix;      // 0/1
  uint8_t  gps_sats;
  float    gps_lat;
  float    gps_lon;
  uint8_t  home_valid;   // 0/1
  float    home_lat;     // home 纬度
  float    home_lon;     // home 经度
  float    hdg;          // 罗盘航向 deg
  float    mag_decl;     // 磁偏角（自动识别，度）
  uint16_t mot[4];       // 电机油门 µs
  float    wp_dist;      // 航点剩余距离（米，-1 = 未激活）
} tel_status_t;

extern tel_status_t tel_status;

void telemetry_begin();   // 开热点 + 启动 Web 服务（setup 调用一次）
void telemetry_poll();    // 主循环调用（内部处理 HTTP，开销小）

/* ---- 数传/串口共用接口（USB 与 UART1 复用同一份 JSON 与命令） ---- */
String tel_build_json(void);             // 组装最新遥测 JSON 行（/api、USB、数传三路共用）
bool   tel_wp_cmd(const String &line);   // 解析下行命令（WP:ADD/CLEAR/START/STOP），已处理返回 true

#endif
