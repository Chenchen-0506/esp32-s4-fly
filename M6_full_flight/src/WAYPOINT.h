/*
 * ============================================================================
 * WAYPOINT 航点飞行功能模块 —— WiFi 网页下发目标坐标，飞机自动飞抵
 * ----------------------------------------------------------------------------
 * 依赖外部数据：GPS（gps_lat/lon）、罗盘航向（mag_heading）、气压高度（rel_alt）
 * 输出指令：目标姿态角 / 目标偏航速率 / 目标油门，由主控制环执行。
 *
 * 状态机：
 *   WP_IDLE    空闲
 *   WP_CLIMB   爬升到目标高度 WP_TARGET_ALT
 *   WP_FLY     机头对准目标点 + 倾斜前进，距离 < WP_NEAR 后进入 ARRIVED
 *   WP_ARRIVED 已到点：姿态回中悬停（油门巡航保持），等待遥控接管/取消
 *
 * 安全设计：
 *   - 启动条件由主程序校验（需已解锁 + GPS 定位）
 *   - 主程序在 WP 模式下检测到遥控摇杆偏离中位 → 立即 wp_stop 交还手动
 *   - 返航（RTL）激活时主程序强制 wp_stop（RTL 优先级更高）
 * ============================================================================
 */
#ifndef WAYPOINT_H
#define WAYPOINT_H

#include <Arduino.h>

/* ---- 航点参数（首次试飞建议保守）---- */
#define WP_TARGET_ALT   30.0f    // 默认航点高度（米），网页可覆盖
#define WP_NEAR         5.0f     // 距目标点多近算到达（米）
#define WP_CLIMB_THR    1500     // 爬升油门 µs
#define WP_CRUISE_THR   1500     // 巡航油门 µs
#define WP_DESCEND_THR  1280     // 高度偏高时收油 µs
#define WP_MAX_TILT     15.0f    // 最大倾斜角（度）
#define WP_YAW_RATE_MAX 120.0f   // 最大偏航速率（度/秒）
#define WP_PITCH_P      0.35f    // 前进倾斜比例（目标倾斜 = 距离误差 * P）
#define WP_YAW_P        2.5f     // 偏航对齐比例

/* ---- 航点状态 ---- */
typedef enum {
  WP_IDLE = 0,
  WP_CLIMB,
  WP_FLY,
  WP_ARRIVED
} WP_MODE;

/* ---- 对外输出：主控制环直接使用 ---- */
typedef struct {
  WP_MODE   mode;
  float     target_roll;     // 度
  float     target_pitch;    // 度
  float     target_yaw_rate; // 度/秒
  float     throttle_us;     // µs
  float     dist;            // 距目标点距离（米）
  float     tgt_lat;         // 目标纬度
  float     tgt_lon;         // 目标经度
  float     tgt_alt;         // 目标高度（米）
} WP_Output;

void          wp_begin();    // 初始化（清状态）
bool          wp_start(float lat, float lon, float alt); // 设定目标并启动（主程序校验后调用）
void          wp_stop();     // 停止，回到 IDLE
bool          wp_active();   // 是否处于航点飞行
WP_Output     wp_update(bool armed, float lat, float lon, float heading, float rel_alt);

#endif
