/*
 * ============================================================================
 * 罗盘功能模块实现 —— QMC5883L / HMC5883 自动识别
 * ============================================================================
 */
#include "MAG.h"
#include <Wire.h>

#define MAG_SDA  8
#define MAG_SCL  9
#define HMC_I2C_ADDR 0x1E
#define QMC_I2C_ADDR 0x0D
#define QMC_REG_X_LSB 0x00
#define QMC_REG_CTRL1 0x09

static uint8_t mag_addr = 0;       // 检测到的地址（0=未找到）
static bool    mag_is_hmc = false; // true=HMC5883, false=QMC5883L

bool mag_begin() {
  // Wire 已由主程序 setup 统一初始化（Wire.begin(8,9)），此处不再重复调用

  // 先试 HMC5883 (0x1E)
  Wire.beginTransmission(HMC_I2C_ADDR);
  if (Wire.endTransmission() == 0) {
    mag_addr = HMC_I2C_ADDR;
    mag_is_hmc = true;
    Wire.beginTransmission(HMC_I2C_ADDR);
    Wire.write(0x02);              // 模式寄存器
    Wire.write(0x00);              // 连续测量模式
    Wire.endTransmission();
    return true;
  }
  // 再试 QMC5883L (0x0D)
  Wire.beginTransmission(QMC_I2C_ADDR);
  if (Wire.endTransmission() == 0) {
    mag_addr = QMC_I2C_ADDR;
    mag_is_hmc = false;
    Wire.beginTransmission(QMC_I2C_ADDR);
    Wire.write(QMC_REG_CTRL1);
    Wire.write(0x01);              // 连续测量, 50Hz, 8x 过采样
    Wire.endTransmission();
    return true;
  }
  return false;
}

static bool mag_read_raw(int16_t &x, int16_t &y, int16_t &z) {
  uint8_t base = mag_is_hmc ? 0x03 : QMC_REG_X_LSB;
  Wire.beginTransmission(mag_addr);
  Wire.write(base);
  Wire.endTransmission();
  Wire.requestFrom(mag_addr, (uint8_t)6);
  if (Wire.available() < 6) return false;

  if (mag_is_hmc) {
    // HMC5883：X MSB/LSB, Z MSB/LSB, Y MSB/LSB
    uint8_t xh = Wire.read(), xl = Wire.read();
    uint8_t zh = Wire.read(), zl = Wire.read();
    uint8_t yh = Wire.read(), yl = Wire.read();
    x = (int16_t)((xh << 8) | xl);
    y = (int16_t)((yh << 8) | yl);
    z = (int16_t)((zh << 8) | zl);
  } else {
    // QMC5883L：X LSB/MSB, Y LSB/MSB, Z LSB/MSB
    uint8_t xl = Wire.read(), xh = Wire.read();
    uint8_t yl = Wire.read(), yh = Wire.read();
    uint8_t zl = Wire.read(), zh = Wire.read();
    x = (int16_t)((xh << 8) | xl);
    y = (int16_t)((yh << 8) | yl);
    z = (int16_t)((zh << 8) | zl);
  }
  return true;
}

void mag_read(float &headingDeg) {
  int16_t mx, my, mz;
  if (!mag_read_raw(mx, my, mz)) { headingDeg = -1.0f; return; }
  float x = (float)mx + MAG_X_OFF;
  float y = (float)my + MAG_Y_OFF;
  float h = atan2f(y, x) * 180.0f / PI + MAG_DECL;
  if (h < 0) h += 360.0f;
  if (h >= 360.0f) h -= 360.0f;
  headingDeg = h;
}
