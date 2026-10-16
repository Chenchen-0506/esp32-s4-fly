/*
 * ============================================================================
 * WAYPOINT 多航点航线飞行功能模块 —— WiFi 网页下发航线，飞机按顺序飞抵
 * ----------------------------------------------------------------------------
 * 依赖外部数据：GPS（gps_lat/lon）、罗盘航向（mag_heading）、气压高度（rel_alt）
 * 输出指令：目标姿态角 / 目标偏航速率 / 目标油门，由主控制环执行。
 *
 * 状态机（沿航线逐点推进）：
 *   WP_IDLE    空闲
 *   WP_CLIMB   爬升到当前航点高度
 *   WP_FLY     机头对准当前航点 + 倾斜前进，距离 < WP_NEAR 后推进下一航点
 *   WP_ARRIVED 全部航点飞完：姿态回中悬停（不自动返航，等待遥控接管/取消）
 *
 * 安全设计：
 *   - 启动条件由主程序校验（需已解锁 + GPS 定位）
 *   - 主程序在 WP 模式下检测到遥控摇杆偏离中位 → 立即 wp_stop 交还手动
 *   - 返航（RTL）激活时主程序强制 wp_stop（RTL 优先级更高）
 *   - 上锁 / 失控自动取消
 * ============================================================================
 */
#ifndef WAYPOINT_H
#define WAYPOINT_H

#include <Arduino.h>

#define WP_MAX        8        // 最大航点数
#define WP_NEAR       5.0f     // 距目标点多近算到达（米）
#define WP_CLIMB_THR  1500     // 爬升油门 µs
#define WP_CRUISE_THR 1500     // 巡航油门 µs
#define WP_DESCEND_THR 1280    // 高度偏高时收油 µs
#define WP_MAX_TILT   15.0f    // 最大倾斜角（度）
#define WP_YAW_RATE_MAX 120.0f // 最大偏航速率（度/秒）
#define WP_PITCH_P    0.35f    // 前进倾斜比例（目标倾斜 = 距离误差 * P）
#define WP_YAW_P      2.5f     // 偏航对齐比例

/* ---- 航点状态 ---- */
typedef enum {
  WP_IDLE = 0,
  WP_CLIMB,
  WP_FLY,
  WP_ARRIVED
} WP_MODE;

/* ---- 单个航点 ---- */
typedef struct {
  float lat, lon, alt;
} WP_Point;

/* ---- 对外输出：主控制环直接使用 ---- */
typedef struct {
  WP_MODE mode;
  float   target_roll;     // 度
  float   target_pitch;    // 度
  float   target_yaw_rate; // 度/秒
  float   throttle_us;     // µs
  float   dist;            // 距当前航点距离（米）
  int     index;           // 当前航点序号（0 起）
  int     total;           // 航点总数
} WP_Output;

void      wp_begin();                                  // 初始化
bool      wp_add_point(float lat, float lon, float alt); // 添加航点（最多 WP_MAX 个）
void      wp_clear();                                  // 清空航线（含停止）
bool      wp_start();                                  // 开始执行航线（列表非空才成功）
void      wp_stop();                                   // 停止，回到 IDLE
bool      wp_active();                                 // 是否处于航点飞行
int       wp_index_now();                              // 当前航点序号（-1 = 未激活）
int       wp_count_now();                              // 航线航点总数
WP_Point  wp_point(int i);                             // 取第 i 个航点
WP_Output wp_update(bool armed, float lat, float lon, float heading, float rel_alt);

#endif
