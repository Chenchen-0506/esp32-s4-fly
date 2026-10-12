/*
 * ============================================================================
 * ICM20948 功能模块 —— 实现文件
 * 原 M1/M6 已验证驱动 + Mahony 6 轴，行为完全一致，仅拆分文件
 * ============================================================================
 */
#include "ICM20948.h"

SPIClass icmSPI(FSPI);

static inline void cs_low()  { digitalWrite(ICM_CS, LOW);  }
static inline void cs_high() { digitalWrite(ICM_CS, HIGH); }

static uint8_t icm_read_reg(uint8_t reg) {
  cs_low();
  icmSPI.transfer(reg | 0x80);
  uint8_t v = icmSPI.transfer(0x00);
  cs_high();
  return v;
}
static void icm_write_reg(uint8_t reg, uint8_t val) {
  cs_low();
  icmSPI.transfer(reg & 0x7F);
  icmSPI.transfer(val);
  cs_high();
}
static void icm_read_burst(uint8_t reg, uint8_t *buf, uint8_t len) {
  cs_low();
  icmSPI.transfer(reg | 0x80);
  for (uint8_t i = 0; i < len; i++) buf[i] = icmSPI.transfer(0x00);
  cs_high();
}
static void icm_bank(uint8_t bank) {
  icm_write_reg(REG_BANK_SEL, bank << 4);
}

float acc_g[3], gyr_dps[3];

static void icm_read_motion_impl() {
  uint8_t b[14];
  icm_read_burst(REG_ACCEL_XOUT_H, b, 14);
  int16_t ax = (int16_t)((b[0] << 8) | b[1]);
  int16_t ay = (int16_t)((b[2] << 8) | b[3]);
  int16_t az = (int16_t)((b[4] << 8) | b[5]);
  int16_t gx = (int16_t)((b[6] << 8) | b[7]);
  int16_t gy = (int16_t)((b[8] << 8) | b[9]);
  int16_t gz = (int16_t)((b[10] << 8) | b[11]);
  acc_g[0] = ax / ACC_LSB_G; acc_g[1] = ay / ACC_LSB_G; acc_g[2] = az / ACC_LSB_G;
  gyr_dps[0] = gx / GYRO_LSB_DPS; gyr_dps[1] = gy / GYRO_LSB_DPS; gyr_dps[2] = gz / GYRO_LSB_DPS;
}

uint8_t icm_init() {
  pinMode(ICM_CS, OUTPUT);
  cs_high();
  icmSPI.begin(ICM_SCK, ICM_MISO, ICM_MOSI, ICM_CS);
  icmSPI.beginTransaction(SPISettings(ICM_SPI_HZ, MSBFIRST, SPI_MODE3));
  cs_high();
  delay(10);
  icm_bank(0);
  icm_write_reg(REG_PWR_MGMT_1, 0x80);
  delay(120);
  icm_write_reg(REG_PWR_MGMT_1, 0x01);
  delay(10);
  icm_write_reg(REG_PWR_MGMT_2, 0x00);
  icm_write_reg(REG_INT_PIN_CFG, 0x00);
  icm_bank(2);
  icm_write_reg(B2_GYRO_SMPLRT_DIV, 0x00);
  icm_write_reg(B2_GYRO_CONFIG_1, 0x1F);
  icm_write_reg(B2_GYRO_CONFIG_2, 0x00);
  icm_write_reg(B2_ACCEL_SMPLRT_DIV_2, 0x00);
  icm_write_reg(B2_ACCEL_CONFIG, 0x1D);
  icm_bank(0);
  delay(50);
  return icm_read_reg(REG_WHO_AM_I);
}

void icm_read_motion() { icm_read_motion_impl(); }

/* ---- 姿态四元数 / 欧拉角 ---- */
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
static float eInt[3] = {0, 0, 0};
float roll_deg, pitch_deg, yaw_deg;
float gyr_bias[3] = {0, 0, 0};

static void q_init_from_accel(float ax, float ay, float az) {
  (void)az;
  float pitch = asinf(constrain(ax, -1.0f, 1.0f));
  float roll  = asinf(constrain(-ay, -1.0f, 1.0f));
  float cr = cosf(roll * 0.5f), sr = sinf(roll * 0.5f);
  float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
  q0 = cr*cp; q1 = sr*cp; q2 = cr*sp; q3 = -sr*sp;
  eInt[0] = eInt[1] = eInt[2] = 0.0f;
}

void imu_calibrate(uint16_t samples) {
  float gs[3] = {0, 0, 0}, as[3] = {0, 0, 0};
  for (uint16_t i = 0; i < samples; i++) {
    icm_read_motion_impl();
    gs[0] += gyr_dps[0]; gs[1] += gyr_dps[1]; gs[2] += gyr_dps[2];
    as[0] += acc_g[0];   as[1] += acc_g[1];   as[2] += acc_g[2];
    delayMicroseconds(1000);
  }
  for (int i = 0; i < 3; i++) gyr_bias[i] = gs[i] / samples;
  float ax = as[0] / samples, ay = as[1] / samples, az = as[2] / samples;
  float n = sqrtf(ax*ax + ay*ay + az*az);
  if (n > 1e-6f) { ax /= n; ay /= n; az /= n; }
  q_init_from_accel(ax, ay, az);
}

void mahony_update6(float ax, float ay, float az,
                    float gx, float gy, float gz, float dt) {
  float vx = 2.0f*(q1*q3 - q0*q2);
  float vy = 2.0f*(q0*q1 + q2*q3);
  float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
  float ex = ay*vz - az*vy;
  float ey = az*vx - ax*vz;
  float ez = ax*vy - ay*vx;
  eInt[0] += ex * Ki_MAHONY * dt;
  eInt[1] += ey * Ki_MAHONY * dt;
  eInt[2] += ez * Ki_MAHONY * dt;
  gx += Kp_MAHONY*ex + eInt[0];
  gy += Kp_MAHONY*ey + eInt[1];
  gz += Kp_MAHONY*ez + eInt[2];
  float dq0 = 0.5f*(-q1*gx - q2*gy - q3*gz);
  float dq1 = 0.5f*( q0*gx + q2*gz - q3*gy);
  float dq2 = 0.5f*( q0*gy - q1*gz + q3*gx);
  float dq3 = 0.5f*( q0*gz + q1*gy - q2*gx);
  q0 += dq0*dt; q1 += dq1*dt; q2 += dq2*dt; q3 += dq3*dt;
  float n = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  if (n > 1e-6f) { q0/=n; q1/=n; q2/=n; q3/=n; }
}

void quat_to_euler_deg() {
  float roll_rad  = atan2f(2.0f*(q0*q1 + q2*q3), 1.0f - 2.0f*(q1*q1 + q2*q2));
  float pitch_rad = asinf(constrain(2.0f*(q0*q2 - q3*q1), -1.0f, 1.0f));
  float yaw_rad   = atan2f(2.0f*(q0*q3 + q1*q2), 1.0f - 2.0f*(q2*q2 + q3*q3));
  roll_deg  = roll_rad * RAD2DEG;
  pitch_deg = pitch_rad * RAD2DEG;
  yaw_deg   = yaw_rad * RAD2DEG;
}
