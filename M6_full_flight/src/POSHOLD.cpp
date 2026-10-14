/*
 * ============================================================================
 * POSHOLD 功能模块实现 —— 悬停控制（参考 ArduPilot Loiter 结构）
 * ============================================================================
 */
#include "POSHOLD.h"
#include <math.h>

static bool     active = false;
static double   target_lat = 0, target_lon = 0;
static float    target_yaw = 0;

void poshold_begin() {
  poshold_stop();
}

void poshold_start(float lat, float lon, float heading) {
  target_lat = lat;
  target_lon = lon;
  target_yaw = heading;
  active = true;
}

bool poshold_active() { return active; }

void poshold_stop() { active = false; }

/* ---- 角度差归一化到 -180~180 ---- */
static float wrap180(float a) {
  while (a > 180.0f)  a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

void poshold_update(float lat, float lon, float heading,
                    float pitch_stick, float roll_stick, float dt,
                    float &pitch_cmd, float &roll_cmd, float &yaw_rate_cmd) {
  pitch_cmd = 0; roll_cmd = 0; yaw_rate_cmd = 0;
  if (!active) return;

  const float rad = PI / 180.0f;
  float h = heading * rad;
  float lat_rad = lat * rad;

  // 位置误差（NED：北/东，米）
  float dxn = (float)(target_lat - lat) * 111320.0f;
  float dxe = (float)(target_lon - lon) * 111320.0f * cosf(lat_rad);

  // 旋转到机身系：ef=机头方向误差，er=右侧误差
  float ef =  dxn * cosf(h) + dxe * sinf(h);
  float er = -dxn * sinf(h) + dxe * cosf(h);

  // 摇杆 → 目标速度（机身系，积分移动悬停目标点）
  float sp_f = (pitch_stick - 1500.0f) * PH_STICK_VEL;
  float sp_r = (roll_stick  - 1500.0f) * PH_STICK_VEL;
  if (sp_f >  PH_STICK_MAX) sp_f =  PH_STICK_MAX;
  if (sp_f < -PH_STICK_MAX) sp_f = -PH_STICK_MAX;
  if (sp_r >  PH_STICK_MAX) sp_r =  PH_STICK_MAX;
  if (sp_r < -PH_STICK_MAX) sp_r = -PH_STICK_MAX;
  // 目标点按摇杆速度积分（机身系 → NED）
  float vN = sp_f * cosf(h) - sp_r * sinf(h);
  float vE = sp_f * sinf(h) + sp_r * cosf(h);
  double dLat = vN * dt / 111320.0;
  double dLon = vE * dt / (111320.0 * cosf(lat_rad));
  target_lat += dLat;
  target_lon += dLon;

  // 位置误差 → 倾角（死区 + 限幅）：目标在前 → 前倾（pitch+），在右 → 右倾（roll+）
  float tilt_f = 0, tilt_r = 0;
  if (ef >  PH_DEADBAND_M) tilt_f =  PH_POS_TILT * (ef - PH_DEADBAND_M);
  if (ef < -PH_DEADBAND_M) tilt_f =  PH_POS_TILT * (ef + PH_DEADBAND_M);
  if (er >  PH_DEADBAND_M) tilt_r =  PH_POS_TILT * (er - PH_DEADBAND_M);
  if (er < -PH_DEADBAND_M) tilt_r =  PH_POS_TILT * (er + PH_DEADBAND_M);

  pitch_cmd = constrain(tilt_f, -PH_ANG_MAX, PH_ANG_MAX);
  roll_cmd  = constrain(tilt_r, -PH_ANG_MAX, PH_ANG_MAX);

  // 航向锁定：缓慢修正回目标航向
  float yaw_err = wrap180(target_yaw - heading);
  yaw_rate_cmd = constrain(PH_YAW_P * yaw_err, -PH_YAW_RATE_MAX, PH_YAW_RATE_MAX);
}
