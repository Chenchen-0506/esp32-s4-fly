/*
 * ============================================================================
 * ALTHOLD 功能模块实现 —— 定高控制（参考 ArduPilot AltHold 结构）
 * 含悬停油门自动学习（ArduPilot MOT_HOVER_LEARN 同款，NVS 持久化）
 * 垂速来源：VERTEST 垂直融合（加速度+气压互补），替代原气压差分+低通
 * ============================================================================
 */
#include "ALTHOLD.h"
#include "PARAM.h"
#include <Preferences.h>

static bool      active = false;
static float     target_alt = 0;
static float     hover_thr = 1500;
static float     vert_spd = 0;        // 垂速（m/s，由 VERTEST 融合提供）
static float     thr_i = 0;           // 油门积分修正（µs）
static uint32_t  last_ms = 0;         // 节流计时
static uint32_t  last_save_ms = 0;    // NVS 保存计时
static uint16_t  last_out = 1000;     // 最近一次输出

/* ---- NVS：悬停油门基准持久化 ---- */
static void hover_save() {
  Preferences p;
  p.begin("althold", false);
  p.putFloat("hover", hover_thr);
  p.end();
}

void althold_load_hover() {
  Preferences p;
  p.begin("althold", true);
  hover_thr = p.getFloat("hover", 1500.0f);
  p.end();
  if (hover_thr < 1100.0f || hover_thr > 1900.0f) hover_thr = 1500.0f; // 脏数据兜底
}

void althold_begin() {
  althold_stop();
}

float althold_hover_thr() { return hover_thr; }

void althold_start(float rel_alt, uint16_t hover_thr_us) {
  target_alt = rel_alt;
  hover_thr  = (float)hover_thr_us;   // 进入时以当前油门为基准（若已学习过则近悬停值）
  vert_spd   = 0;
  thr_i      = 0;
  last_ms    = millis();
  last_save_ms = millis();
  last_out   = hover_thr_us;
  active = true;
}

bool althold_active() { return active; }

void althold_stop() { active = false; }

/* ---- 每 20ms 更新一次（其余时间返回上次输出）---- */
uint16_t althold_update(float rel_alt, float vert_vel, float stick_thr) {
  if (!active) return last_out;

  uint32_t now = millis();
  uint32_t dms = now - last_ms;
  if (dms < ALT_STEP_MS) return last_out;
  float dt = (float)dms / 1000.0f;
  last_ms = now;

  // 垂速：直接用 VERTEST 融合值（加速度+气压互补，噪声低 5~10 倍）
  vert_spd = vert_vel;

  // 油门杆 → 目标垂速（死区 + 限幅）
  float stick_d = stick_thr - 1500.0f;
  float stick_vs = 0;
  if (stick_d >  alt_stick_db) stick_vs =  (stick_d - alt_stick_db) * alt_stick_k;
  if (stick_d < -alt_stick_db) stick_vs =  (stick_d + alt_stick_db) * alt_stick_k;
  if (stick_vs >  alt_vs_max) stick_vs =  alt_vs_max;
  if (stick_vs < -alt_vs_max) stick_vs = -alt_vs_max;

  // 高度误差 → 修正垂速（ArduPilot AltHold P）
  float alt_err = target_alt - rel_alt;
  float target_vs = stick_vs + alt_p * alt_err;
  if (target_vs >  200.0f) target_vs =  200.0f;
  if (target_vs < -200.0f) target_vs = -200.0f;

  // 垂速误差 → PID → 油门
  float vs_err = target_vs - vert_spd;
  thr_i += vs_err * dt * vel_i;
  if (thr_i >  thr_i_limit) thr_i =  thr_i_limit;
  if (thr_i < -thr_i_limit) thr_i = -thr_i_limit;

  float thr = hover_thr + vel_p * vs_err + thr_i;
  last_out = (uint16_t)constrain(thr, 1000.0f, 2000.0f);

  // ---- 悬停油门自动学习（悬停稳定时向当前油门收敛）----
  if (fabsf(stick_d) < alt_stick_db &&   // 油门杆在中位（无主动升降意图）
      fabsf(alt_err) < HOVER_MAX_ALT_ERR &&  // 高度误差小
      fabsf(vert_spd) < HOVER_MAX_VS) {      // 垂速小（近似悬停）
    hover_thr += ((float)last_out - hover_thr) * HOVER_LEARN_RATE;
    if (millis() - last_save_ms >= HOVER_SAVE_MS) {
      last_save_ms = millis();
      hover_save();
    }
  }

  return last_out;
}
