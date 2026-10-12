/*
 * ============================================================================
 * Motors 功能模块 —— 实现文件
 * ============================================================================
 */
#include "Motors.h"

static const uint8_t MOTOR_PINS[4] = {M1_PIN, M2_PIN, M3_PIN, M4_PIN};
uint16_t motor_us[4] = {MIN_US, MIN_US, MIN_US, MIN_US};

void motors_begin() {
  for (int i = 0; i < 4; i++) {
    pinMode(MOTOR_PINS[i], OUTPUT);
    analogWriteResolution(MOTOR_PINS[i], PWM_RES_BITS);
    analogWriteFrequency(MOTOR_PINS[i], 50);   // 电调标准 50Hz
  }
  set_all_motors(MIN_US);                       // 上电默认油门最低，电机不转
}

void set_motor_us(uint8_t idx, uint16_t us) {
  if (us < MIN_US) us = MIN_US;
  if (us > MAX_US) us = MAX_US;
  motor_us[idx] = us;
  uint32_t duty = (uint32_t)us * (1UL << PWM_RES_BITS) / PERIOD_US;
  analogWrite(MOTOR_PINS[idx], duty);
}

void set_all_motors(uint16_t us) {
  for (int i = 0; i < 4; i++) set_motor_us(i, us);
}
