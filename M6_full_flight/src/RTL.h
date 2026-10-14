/*
 * ============================================================================
 * RTL 返航功能模块 —— 手动返航（CH6）+ 遥控丢失自动返航
 * ----------------------------------------------------------------------------
 * 依赖外部数据：GPS（gps_lat/lon/fix）、罗盘（mag_read）、气压高度（bmp_rel_alt_m）
 * 输出指令：目标姿态角 / 目标偏航速率 / 目标油门，由主控制环执行。
 *
 * 返航状态机：
 *   RTL_IDLE    空闲（不返航）
 *   RTL_CLIMB   爬升到安全高度 RTL_ALT
 *   RTL_FLY     机头对准 home 方向 + 倾斜前进，距离 < RTL_NEAR 后进入降落
 *   RTL_DESCEND 缓降，触地后自动上锁
 *   RTL_LANDED  已降落（主程序上锁停转）
 *
 * 安全设计：
 *   - 无 GPS 定位时不进入 FLY，直接缓降上锁（由主程序判断）
 *   - 返航中油门低于安全值自动终止（主程序兜底）
 *   - 所有参数可用宏调节，首次试飞建议保守值
 * ============================================================================
 */
#ifndef RTL_H
#define RTL_H

#include <Arduino.h>

/* ---- 返航参数（首次试飞建议保守）---- */
#define RTL_ALT          120.0f  // 返航巡航高度（米）
#define RTL_NEAR         5.0f    // 距 home 多近开始降落（米）
#define RTL_CLIMB_THR    1500    // 爬升油门 µs
#define RTL_CRUISE_THR   1500    // 巡航油门 µs
#define RTL_DESCEND_THR  1280    // 缓降油门 µs（低于怠速即可缓降）
#define RTL_MAX_TILT     15.0f   // 返航最大倾斜角（度）
#define RTL_YAW_RATE_MAX 120.0f  // 最大偏航速率（度/秒）
#define RTL_PITCH_P      0.35f   // 前进倾斜比例（目标倾斜 = 距离误差 * P）
#define RTL_YAW_P        2.5f    // 偏航对齐比例

/* ---- 返航状态 ---- */
typedef enum {
  RTL_IDLE = 0,
  RTL_CLIMB,
  RTL_FLY,
  RTL_DESCEND,
  RTL_LANDED
} RTL_MODE;

/* ---- 对外输出：主控制环直接使用 ---- */
typedef struct {
  RTL_MODE mode;
  float    target_roll;     // 度
  float    target_pitch;    // 度
  float    target_yaw_rate; // 度/秒
  float    throttle_us;     // µs
  float    home_dist;       // 距 home 距离（米）
} RTL_Output;

void    rtl_begin();                    // 初始化（清状态）
void    rtl_set_home(float lat, float lon);  // 记录起飞点（解锁时调用）
bool    rtl_home_valid();               // home 点是否已记录
float   rtl_dist_to_home(float lat, float lon); // 距 home 距离（米）
void    rtl_reset();                    // 复位到 IDLE
RTL_Output rtl_update(bool armed, bool rc_lost,
                      float lat, float lon, float heading,
                      float rel_alt);

#endif
