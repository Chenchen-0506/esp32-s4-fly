/*
 * ============================================================================
 * PARAM.cpp —— 参数表 + NVS 持久化 + JSON 输出
 * ============================================================================
 */
#include "PARAM.h"
#include "PID.h"       // RATE_* 默认宏
#include "ALTHOLD.h"   // ALT_*/VEL_*/THR_I_LIMIT 默认宏
#include "RTL.h"       // RTL_* 默认宏
#include <Preferences.h>

/* ---- 参数变量定义（默认值 = 各模块宏初值）---- */
float rate_p_roll=RATE_P_ROLL,  rate_i_roll=RATE_I_ROLL,  rate_d_roll=RATE_D_ROLL;
float rate_p_pitch=RATE_P_PITCH, rate_i_pitch=RATE_I_PITCH, rate_d_pitch=RATE_D_PITCH;
float rate_p_yaw=RATE_P_YAW,    rate_i_yaw=RATE_I_YAW,    rate_d_yaw=RATE_D_YAW;
float rate_i_limit=RATE_I_LIMIT;
float angle_p_roll=ANGLE_P_ROLL, angle_p_pitch=ANGLE_P_PITCH, angle_p_yaw=ANGLE_P_YAW;
float alt_p=ALT_P, alt_stick_k=ALT_STICK_K, alt_stick_db=ALT_STICK_DB, alt_vs_max=ALT_VS_MAX;
float vel_p=VEL_P, vel_i=VEL_I, thr_i_limit=THR_I_LIMIT;
float rtl_alt=RTL_ALT, rtl_near=RTL_NEAR;
float rtl_climb_thr=RTL_CLIMB_THR, rtl_cruise_thr=RTL_CRUISE_THR, rtl_descend_thr=RTL_DESCEND_THR;
float rtl_max_tilt=RTL_MAX_TILT, rtl_pitch_p=RTL_PITCH_P, rtl_yaw_p=RTL_YAW_P;
float max_tilt_manu=MANU_MAX_TILT, yaw_rate_manu=MANU_YAW_RATE;

/* ---- 参数注册表（顺序即 PARAM:GET 输出顺序）---- */
typedef struct { const char *key; float *ptr; } ParamDef;
static const ParamDef PARAMS[] = {
  {"rate_p_roll",  &rate_p_roll},  {"rate_i_roll",  &rate_i_roll},  {"rate_d_roll",  &rate_d_roll},
  {"rate_p_pitch", &rate_p_pitch}, {"rate_i_pitch", &rate_i_pitch}, {"rate_d_pitch", &rate_d_pitch},
  {"rate_p_yaw",   &rate_p_yaw},   {"rate_i_yaw",   &rate_i_yaw},   {"rate_d_yaw",   &rate_d_yaw},
  {"rate_i_limit", &rate_i_limit},
  {"angle_p_roll", &angle_p_roll}, {"angle_p_pitch",&angle_p_pitch},{"angle_p_yaw", &angle_p_yaw},
  {"alt_p",        &alt_p},        {"alt_stick_k",  &alt_stick_k},  {"alt_stick_db", &alt_stick_db},
  {"alt_vs_max",   &alt_vs_max},
  {"vel_p",        &vel_p},        {"vel_i",        &vel_i},        {"thr_i_limit",  &thr_i_limit},
  {"rtl_alt",      &rtl_alt},      {"rtl_near",     &rtl_near},
  {"rtl_climb_thr",&rtl_climb_thr},{"rtl_cruise_thr",&rtl_cruise_thr},{"rtl_descend_thr",&rtl_descend_thr},
  {"rtl_max_tilt", &rtl_max_tilt}, {"rtl_pitch_p",  &rtl_pitch_p},  {"rtl_yaw_p",    &rtl_yaw_p},
  {"max_tilt_manu",&max_tilt_manu},{"yaw_rate_manu",&yaw_rate_manu},
};
#define PARAM_COUNT (sizeof(PARAMS) / sizeof(PARAMS[0]))

static const char *NVS_NS = "param";

void param_begin() {
  Preferences p;
  p.begin(NVS_NS, true);
  for (uint16_t i = 0; i < PARAM_COUNT; i++) {
    float def = *PARAMS[i].ptr;              // 当前值即默认（编译初值）
    *PARAMS[i].ptr = p.getFloat(PARAMS[i].key, def);
  }
  p.end();
}

float param_get(const char *key) {
  for (uint16_t i = 0; i < PARAM_COUNT; i++) {
    if (strcmp(PARAMS[i].key, key) == 0) return *PARAMS[i].ptr;
  }
  return NAN;
}

bool param_set(const char *key, float v) {
  for (uint16_t i = 0; i < PARAM_COUNT; i++) {
    if (strcmp(PARAMS[i].key, key) == 0) {
      *PARAMS[i].ptr = v;
      Preferences p;
      p.begin(NVS_NS, false);
      p.putFloat(PARAMS[i].key, v);
      p.end();
      return true;
    }
  }
  return false;
}

void param_reset() {
  Preferences p;
  p.begin(NVS_NS, false);
  p.clear();
  p.end();
  for (uint16_t i = 0; i < PARAM_COUNT; i++) {
    *PARAMS[i].ptr = *PARAMS[i].ptr;   // 占位（默认值在复位后由各自模块宏重建）
  }
  /* 恢复编译初值：需要重新读宏——直接重新赋值（宏在头文件里，再次引用即可） */
  rate_p_roll=RATE_P_ROLL;  rate_i_roll=RATE_I_ROLL;  rate_d_roll=RATE_D_ROLL;
  rate_p_pitch=RATE_P_PITCH; rate_i_pitch=RATE_I_PITCH; rate_d_pitch=RATE_D_PITCH;
  rate_p_yaw=RATE_P_YAW;    rate_i_yaw=RATE_I_YAW;    rate_d_yaw=RATE_D_YAW;
  rate_i_limit=RATE_I_LIMIT;
  angle_p_roll=ANGLE_P_ROLL; angle_p_pitch=ANGLE_P_PITCH; angle_p_yaw=ANGLE_P_YAW;
  alt_p=ALT_P; alt_stick_k=ALT_STICK_K; alt_stick_db=ALT_STICK_DB; alt_vs_max=ALT_VS_MAX;
  vel_p=VEL_P; vel_i=VEL_I; thr_i_limit=THR_I_LIMIT;
  rtl_alt=RTL_ALT; rtl_near=RTL_NEAR;
  rtl_climb_thr=RTL_CLIMB_THR; rtl_cruise_thr=RTL_CRUISE_THR; rtl_descend_thr=RTL_DESCEND_THR;
  rtl_max_tilt=RTL_MAX_TILT; rtl_pitch_p=RTL_PITCH_P; rtl_yaw_p=RTL_YAW_P;
  max_tilt_manu=MANU_MAX_TILT; yaw_rate_manu=MANU_YAW_RATE;
}

String param_json() {
  String j;
  j.reserve(512);
  j = "{";
  for (uint16_t i = 0; i < PARAM_COUNT; i++) {
    if (i) j += ',';
    j += '"'; j += PARAMS[i].key; j += "\":";
    j += String(*PARAMS[i].ptr, 4);
  }
  j += '}';
  return j;
}
