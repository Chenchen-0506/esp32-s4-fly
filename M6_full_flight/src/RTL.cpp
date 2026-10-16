/*
 * ============================================================================
 * RTL 返航功能模块实现
 * ============================================================================
 */
#include "RTL.h"
#include <math.h>

static float home_lat = 0, home_lon = 0;
static bool  home_valid = false;
static RTL_Output out;

static uint32_t descend_start_ms = 0;   // 触地判定计时
static float    last_alt = 0;

/* ---- 度 -> 弧度 ---- */
static float rad(float deg) { return deg * PI / 180.0f; }

/* ---- 距离（米，haversine）---- */
float rtl_dist_to_home(float lat, float lon) {
  if (!home_valid) return 0.0f;
  const float R = 6371000.0f;
  float dLat = rad(lat - home_lat);
  float dLon = rad(lon - home_lon);
  float a = sinf(dLat/2)*sinf(dLat/2) +
            cosf(rad(home_lat))*cosf(rad(lat))*sinf(dLon/2)*sinf(dLon/2);
  float c = 2.0f * atan2f(sqrtf(a), sqrtf(1.0f - a));
  return R * c;
}

/* ---- home 方位角（度，正北=0 顺时针）---- */
static float bearing_to_home(float lat, float lon) {
  if (!home_valid) return 0.0f;
  float dLon = rad(home_lon - lon);
  float y = sinf(dLon) * cosf(rad(home_lat));
  float x = cosf(rad(lat)) * sinf(rad(home_lat)) -
            sinf(rad(lat)) * cosf(rad(home_lat)) * cosf(dLon);
  float b = atan2f(y, x) * 180.0f / PI;
  if (b < 0) b += 360.0f;
  return b;
}

/* ---- 角度差归一化到 -180~180 ---- */
static float wrap180(float a) {
  while (a > 180.0f)  a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

void rtl_begin() {
  home_valid = false;
  home_lat = home_lon = 0;
  rtl_reset();
}

void rtl_set_home(float lat, float lon) {
  if (lat != 0.0f || lon != 0.0f) {
    home_lat = lat;
    home_lon = lon;
    home_valid = true;
  }
}

float rtl_home_lat() { return home_lat; }
float rtl_home_lon() { return home_lon; }

bool rtl_home_valid() {
  return home_valid;
}

void rtl_reset() {
  out.mode = RTL_IDLE;
  out.target_roll = 0;
  out.target_pitch = 0;
  out.target_yaw_rate = 0;
  out.throttle_us = 1000;
  out.home_dist = 0;
  descend_start_ms = 0;
  last_alt = 0;
}

/* ============================================================================
 * rtl_update：每控制环调用一次，产出当前指令
 *   进入返航：主程序检测到 rc_lost 或 CH6 拨杆后调用本函数（armed 状态下）
 *   状态推进：CLIMB -> FLY -> DESCEND -> LANDED
 * ========================================================================== */
RTL_Output rtl_update(bool armed, bool rc_lost,
                      float lat, float lon, float heading,
                      float rel_alt) {
  if (!armed) { rtl_reset(); return out; }
  if (!home_valid) { rtl_reset(); return out; }

  out.home_dist = rtl_dist_to_home(lat, lon);

  // ---- 上一轮退出返航（解锁后未进入，或遥控恢复且不在返航中）----
  // 状态机只在首次调用时从 IDLE 进入（主程序保证触发条件）

  if (out.mode == RTL_IDLE) {
    // 进入：先爬升
    out.mode = RTL_CLIMB;
    last_alt = rel_alt;
  }

  switch (out.mode) {
    case RTL_CLIMB: {
      // 爬到巡航高度
      out.target_roll = 0;
      out.target_pitch = 0;
      out.target_yaw_rate = 0;
      if (rel_alt < RTL_ALT - 1.0f) {
        out.throttle_us = RTL_CLIMB_THR;
      } else {
        out.mode = RTL_FLY;
        out.throttle_us = RTL_CRUISE_THR;
      }
      break;
    }
    case RTL_FLY: {
      // 高度保持：低于巡航高度就补油门，高于则收
      if (rel_alt < RTL_ALT - 1.5f)      out.throttle_us = RTL_CLIMB_THR;
      else if (rel_alt > RTL_ALT + 1.5f) out.throttle_us = RTL_DESCEND_THR;
      else                               out.throttle_us = RTL_CRUISE_THR;

      // 机头对准 home 方向（用罗盘航向）
      float b = bearing_to_home(lat, lon);
      float yaw_err = wrap180(b - heading);
      out.target_yaw_rate = RTL_YAW_P * yaw_err;
      if (out.target_yaw_rate >  RTL_YAW_RATE_MAX) out.target_yaw_rate =  RTL_YAW_RATE_MAX;
      if (out.target_yaw_rate < -RTL_YAW_RATE_MAX) out.target_yaw_rate = -RTL_YAW_RATE_MAX;

      // 机头对准后给前进倾斜（P 控制距离）
      float pitch_cmd = RTL_PITCH_P * (out.home_dist - RTL_NEAR);
      if (pitch_cmd >  RTL_MAX_TILT) pitch_cmd =  RTL_MAX_TILT;
      if (pitch_cmd < -RTL_MAX_TILT) pitch_cmd = -RTL_MAX_TILT;
      out.target_pitch = pitch_cmd;
      out.target_roll  = 0;

      // 接近 home 进入降落（水平修正由 DESCEND 分支保留）
      if (out.home_dist < RTL_NEAR) {
        out.mode = RTL_DESCEND;
        descend_start_ms = millis();
      }
      break;
    }
    case RTL_DESCEND: {
      // 缓降 + 保留水平位置修正（机头对准 home、距离 P 控制，限幅更小）
      // —— 边缓降边回到 home 正上方，落点不再被风推偏
      out.target_roll = 0;
      out.throttle_us = RTL_DESCEND_THR;

      float b = bearing_to_home(lat, lon);
      float yaw_err = wrap180(b - heading);
      out.target_yaw_rate = RTL_YAW_P * yaw_err;
      if (out.target_yaw_rate >  RTL_YAW_RATE_MAX) out.target_yaw_rate =  RTL_YAW_RATE_MAX;
      if (out.target_yaw_rate < -RTL_YAW_RATE_MAX) out.target_yaw_rate = -RTL_YAW_RATE_MAX;

      float pitch_cmd = RTL_PITCH_P * (out.home_dist - RTL_NEAR);
      if (pitch_cmd >  RTL_DESCEND_MAX_TILT) pitch_cmd =  RTL_DESCEND_MAX_TILT;
      if (pitch_cmd < -RTL_DESCEND_MAX_TILT) pitch_cmd = -RTL_DESCEND_MAX_TILT;
      out.target_pitch = pitch_cmd;

      // 触地判定（不变）：高度不再下降且很低，持续 2 秒 -> 上锁
      bool near_ground = (rel_alt < 1.0f);
      bool no_change   = (fabsf(rel_alt - last_alt) < 0.15f);
      if (near_ground && no_change) {
        if (millis() - descend_start_ms > 2000) {
          out.mode = RTL_LANDED;
        }
      } else {
        descend_start_ms = millis();
      }
      last_alt = rel_alt;
      break;
    }
    case RTL_LANDED:
    default:
      out.throttle_us = 1000;
      out.target_roll = 0;
      out.target_pitch = 0;
      out.target_yaw_rate = 0;
      break;
  }

  return out;
}
