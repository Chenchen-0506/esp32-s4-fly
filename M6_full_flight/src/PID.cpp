/*
 * ============================================================================
 * PID 功能模块 —— 实现文件
 * ============================================================================
 */
#include "PID.h"

void pid_init(PID *p, float kp, float ki, float kd) {
  p->kp = kp; p->ki = ki; p->kd = kd;
  p->i_term = 0; p->prev_err = 0;
}

float pid_calc(PID *p, float err, float dt) {
  p->i_term += err * dt;
  if (p->i_term > RATE_I_LIMIT) p->i_term = RATE_I_LIMIT;
  if (p->i_term < -RATE_I_LIMIT) p->i_term = -RATE_I_LIMIT;
  float deriv = (err - p->prev_err) / dt;
  p->prev_err = err;
  return p->kp * err + p->ki * p->i_term + p->kd * deriv;
}
