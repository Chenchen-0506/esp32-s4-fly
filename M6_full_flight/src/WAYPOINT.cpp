/*
 * ============================================================================
 * WAYPOINT 多航点航线飞行功能模块实现
 * 距离/方位计算与 RTL 同构（haversine + 方位角），目标点参数化
 * ============================================================================
 */
#include "WAYPOINT.h"
#include <math.h>

static WP_Point wp_list[WP_MAX];
static int     wp_count = 0;
static int     wp_index = 0;
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

bool wp_add_point(float lat, float lon, float alt) {
  if (wp_count >= WP_MAX) return false;
  if (fabsf(lat) > 90.0f || fabsf(lon) > 180.0f) return false;
  if (alt < 1.0f || alt > 120.0f) return false;
  wp_list[wp_count].lat = lat;
  wp_list[wp_count].lon = lon;
  wp_list[wp_count].alt = alt;
  wp_count++;
  return true;
}

void wp_clear() {
  wp_count = 0;
  wp_index = 0;
  wp_stop();
}

bool wp_start() {
  if (wp_count == 0) return false;
  out.mode = WP_IDLE;       // 由 wp_update 在 armed 下自动进入 CLIMB
  wp_index = 0;
  out.dist = 0;
  out.index = 0;
  out.total = wp_count;
  return true;
}

void wp_stop() {
  out.mode = WP_IDLE;
  out.target_roll = 0;
  out.target_pitch = 0;
  out.target_yaw_rate = 0;
  out.throttle_us = 1000;
  out.dist = 0;
  out.index = -1;
  out.total = 0;
}

bool wp_active() {
  return (out.mode != WP_IDLE);
}

int wp_index_now() {
  return wp_active() ? out.index : -1;
}

int wp_count_now() {
  return wp_count;
}

WP_Point wp_point(int i) {
  WP_Point p = {0, 0, 0};
  if (i >= 0 && i < wp_count) p = wp_list[i];
  return p;
}

/* ============================================================================
 * wp_update：每控制环调用一次，产出当前指令
 *   首次调用（IDLE + armed）自动进入 CLIMB；沿航线逐点推进
 *   CLIMB -> FLY -> (到点) -> 下一航点 CLIMB ... -> 最后一个 -> ARRIVED 悬停
 * ========================================================================== */
WP_Output wp_update(bool armed, float lat, float lon, float heading, float rel_alt) {
  if (!armed) { wp_stop(); return out; }
  if (wp_count == 0) { wp_stop(); return out; }

  WP_Point tgt = wp_list[wp_index];
  out.index = wp_index;
  out.total = wp_count;
  out.dist = dist_xy(lat, lon, tgt.lat, tgt.lon);

  if (out.mode == WP_IDLE) out.mode = WP_CLIMB;

  switch (out.mode) {
    case WP_CLIMB: {
      // 先爬升到当前航点高度
      out.target_roll = 0;
      out.target_pitch = 0;
      out.target_yaw_rate = 0;
      if (rel_alt < tgt.alt - 1.0f) {
        out.throttle_us = WP_CLIMB_THR;
      } else {
        out.mode = WP_FLY;
        out.throttle_us = WP_CRUISE_THR;
      }
      break;
    }
    case WP_FLY: {
      // 高度保持（同 RTL 三段油门）
      if (rel_alt < tgt.alt - 1.5f)      out.throttle_us = WP_CLIMB_THR;
      else if (rel_alt > tgt.alt + 1.5f) out.throttle_us = WP_DESCEND_THR;
      else                               out.throttle_us = WP_CRUISE_THR;

      // 机头对准目标方向
      float b = bearing_xy(lat, lon, tgt.lat, tgt.lon);
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

      if (out.dist < WP_NEAR) {
        // 到点：推进下一个航点；全部飞完进入悬停
        if (wp_index < wp_count - 1) {
          wp_index++;
          out.mode = WP_CLIMB;
          Serial.printf("[WP] 到达航点 %d/%d，前往下一航点\n", out.index + 1, out.total);
        } else {
          out.mode = WP_ARRIVED;
          Serial.printf("[WP] 全部航点飞完（%d 个），悬停等待\n", out.total);
        }
      }
      break;
    }
    case WP_ARRIVED: {
      // 到点悬停：姿态回中，油门巡航保持（等遥控接管/取消）
      out.target_roll = 0;
      out.target_pitch = 0;
      out.target_yaw_rate = 0;
      if (rel_alt < tgt.alt - 1.5f)      out.throttle_us = WP_CLIMB_THR;
      else if (rel_alt > tgt.alt + 1.5f) out.throttle_us = WP_DESCEND_THR;
      else                               out.throttle_us = WP_CRUISE_THR;
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
