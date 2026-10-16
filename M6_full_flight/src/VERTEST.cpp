/*
 * ============================================================================
 * VERTEST.cpp —— 垂直状态估计实现
 * 1D 互补/低通卡尔曼：加速度积分（高频真实）+ 气压观测（低频稳定）
 * ============================================================================
 */
#include "VERTEST.h"

static float vert_alt_f = 0.0f;   // 融合高度（m，向上为正）
static float vert_vel_f = 0.0f;   // 融合垂速（m/s，向上为正）
static bool  vert_inited = false;

void vertest_begin() {
  vert_alt_f = 0.0f;
  vert_vel_f = 0.0f;
  vert_inited = false;
}

void vertest_reset(float alt0) {
  vert_alt_f = alt0;
  vert_vel_f = 0.0f;
  vert_inited = true;
}

/* 预测：垂直加速度 -> 垂速 -> 高度（每环 1kHz 调用） */
void vertest_predict(float ax, float ay, float az, const float q[4], float dt) {
  if (!vert_inited) return;

  /* 机体加速度（g）→ 地理系（ENU）向上分量（含重力，静止水平 ≈ +1g）
   * body->ENU 旋转矩阵第三行：[ 2(xz-wy)  2(yz+wx)  (w²+z²-x²-y²) ] */
  float w = q[0], x = q[1], y = q[2], z = q[3];
  float up = (w*w + z*z - x*x - y*y) * az
           + 2.0f*(x*z - w*y) * ax
           + 2.0f*(y*z + w*x) * ay;

  float a = (up - 1.0f) * 9.80665f;   // 去重力后为垂直加速度（向上为正）
  if (a >  VERT_ACC_MAX) a =  VERT_ACC_MAX;
  if (a < -VERT_ACC_MAX) a = -VERT_ACC_MAX;

  vert_vel_f += a * dt;
  if (vert_vel_f >  VERT_VEL_MAX) vert_vel_f =  VERT_VEL_MAX;
  if (vert_vel_f < -VERT_VEL_MAX) vert_vel_f = -VERT_VEL_MAX;

  vert_alt_f += vert_vel_f * dt;
}

/* 修正：BMP280 气压高度观测（10Hz 调用） */
void vertest_correct(float bmp_alt) {
  if (!vert_inited) return;

  float err = bmp_alt - vert_alt_f;   // 气压高度 - 融合高度
  vert_alt_f += VERT_K_ALT * err;     // 高度被气压缓慢拉回（低通）
  vert_vel_f += VERT_K_VEL * err;     // 垂速被轻微修正（抑制积分漂移）
}

float vertest_alt() { return vert_alt_f; }
float vertest_vel() { return vert_vel_f; }
