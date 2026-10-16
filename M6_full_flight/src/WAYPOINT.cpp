/*
 * ============================================================================
 * WAYPOINT 航点飞行功能模块实现
 * 距离/方位计算与 RTL 同构（haversine + 方位角），目标点参数化
 * ============================================================================
 */
#include "WAYPOINT.h"
#include <math.h>

static WP_Output out;

static float rad(float deg) { return deg * PI / 180.0f; }

/* ---- 两点距离（米，haversine）---- */
static float dist_xy(float lat1, float lon1, float lat2, float lon2) {
  const float R = 6371000.0f;
  float dLat = rad(lat2 - lat1);
  float dLon = rad(lon2 - lon1);
  float a = sinf(dLat/2)*sinf(dLat/2) +
            cosf(rad(lat1))*cosf(rad(lat2))*sinf(dLon/2)*sinf(dLon/2);
  float c = 2.0f * atan2f(sqrtf(a), sqrtf(1.0f - a));
  return R * c;
}

/* ---- 从 (lat1,lon1) 到 (lat2,lon2) 的方位角（度，正北=0 顺时针）---- */
static float bearing_xy(float lat1, float lon1, float lat2, float lon2) {
  float dLon = rad(lon2 - lon1);
  float y = sinf(dLon) * cosf(rad(lat2));
  float x = cosf(rad(lat1)) * sinf(rad(lat2)) -
            sinf(rad(lat1)) * cosf(rad(lat2)) * cosf(dLon);
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

void wp_begin() {
  wp_stop();
}

bool wp_start(float lat, float lon, float alt) {
  // 参数合法性：经纬度范围 + 高度范围
  if (fabsf(lat) > 90.0f || fabsf(lon) > 180.0f) return false;
  if (alt < 1.0f || alt > 120.0f) return false;
  out.mode = WP_IDLE;
  out.tgt_lat = lat;
  out.tgt_lon = lon;
  out.tgt_alt = alt;
  out.dist = 0;
  out.target_roll = 0;
  out.target_pitch = 0;
  out.target_yaw_rate = 0;
  out.throttle_us = 1000;
  return true;   // 模式由 wp_update 在 armed 下自动进入 CLIMB
}

void wp_stop() {
  out.mode = WP_IDLE;
  out.target_roll = 0;
  out.target_pitch = 0;
  out.target_yaw_rate = 0;
  out.throttle_us = 1000;
  out.dist = 0;
}

bool wp_active() {
  return (out.mode != WP_IDLE);
}

/* ============================================================================
 * wp_update：每控制环调用一次，产出当前指令
 *   首次调用（IDLE + armed）自动进入 CLIMB；CLIMB -> FLY -> ARRIVED
 * ========================================================================== */
WP_Output wp_update(bool armed, float lat, float lon, float heading, float rel_alt) {
  if (!armed) { wp_stop(); return out; }

  out.dist = dist_xy(lat, lon, out.tgt_lat, out.tgt_lon);

  if (out.mode == WP_IDLE) out.mode = WP_CLIMB;

  switch (out.mode) {
    case WP_CLIMB: {
      // 先爬升到目标高度
      out.target_roll = 0;
      out.target_pitch = 0;
      out.target_yaw_rate = 0;
      if (rel_alt < out.tgt_alt - 1.0f) {
        out.throttle_us = WP_CLIMB_THR;
      } else {
        out.mode = WP_FLY;
        out.throttle_us = WP_CRUISE_THR;
      }
      break;
    }
    case WP_FLY: {
      // 高度保持（同 RTL 三段油门）
      if (rel_alt < out.tgt_alt - 1.5f)      out.throttle_us = WP_CLIMB_THR;
      else if (rel_alt > out.tgt_alt + 1.5f) out.throttle_us = WP_DESCEND_THR;
      else                                   out.throttle_us = WP_CRUISE_THR;

      // 机头对准目标方向
      float b = bearing_xy(lat, lon, out.tgt_lat, out.tgt_lon);
      float yaw_err = wrap180(b - heading);
      out.target_yaw_rate = WP_YAW_P * yaw_err;
      if (out.target_yaw_rate >  WP_YAW_RATE_MAX) out.target_yaw_rate =  WP_YAW_RATE_MAX;
      if (out.target_yaw_rate < -WP_YAW_RATE_MAX) out.target_yaw_rate = -WP_YAW_RATE_MAX;

      // 前进倾斜（P 控制距离）
      float pitch_cmd = WP_PITCH_P * (out.dist - WP_NEAR);
      if (pitch_cmd >  WP_MAX_TILT) pitch_cmd =  WP_MAX_TILT;
      if (pitch_cmd < -WP_MAX_TILT) pitch_cmd = -WP_MAX_TILT;
      out.target_pitch = pitch_cmd;
      out.target_roll  = 0;

      if (out.dist < WP_NEAR) out.mode = WP_ARRIVED;
      break;
    }
    case WP_ARRIVED: {
      // 到点悬停：姿态回中，油门巡航保持（等遥控接管/取消）
      out.target_roll = 0;
      out.target_pitch = 0;
      out.target_yaw_rate = 0;
      if (rel_alt < out.tgt_alt - 1.5f)      out.throttle_us = WP_CLIMB_THR;
      else if (rel_alt > out.tgt_alt + 1.5f) out.throttle_us = WP_DESCEND_THR;
      else                                   out.throttle_us = WP_CRUISE_THR;
      break;
    }
    default:
      out.throttle_us = 1000;
      out.target_roll = 0;
      out.target_pitch = 0;
      out.target_yaw_rate = 0;
      break;
  }

  return out;
}
