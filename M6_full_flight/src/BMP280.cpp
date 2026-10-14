/*
 * ============================================================================
 * BMP280 气压计模块实现（I2C，寄存器级驱动）
 * 移植自 esp32_sensor_check 自检工程，抽成 M6 的 src 功能模块。
 * 只使用 <Wire.h>，零第三方依赖。
 * ============================================================================
 */
#include "BMP280.h"

// ---- 内部状态 ----
static uint8_t  bmpAddr = 0;          // 探测到的 I2C 地址（0x76 或 0x77）
static uint16_t dig_T1, dig_P1;       // 出厂校准系数（来自芯片 OTP）
static int16_t  dig_T2, dig_T3;
static int16_t  dig_P2, dig_P3, dig_P4, dig_P5, dig_P6;
static int16_t  dig_P7, dig_P8, dig_P9;
static int32_t  t_fine = 0;

// 起飞参考气压（由 bmp_set_ground 写入，主程序可在解锁前调用）
float bmp_ground_pa = 0.0f;

// ---- 底层 I2C 读写 ----
static uint8_t bmpRead8(uint8_t reg) {
  Wire.beginTransmission(bmpAddr);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom((int)bmpAddr, 1);
  return Wire.available() ? Wire.read() : 0xFF;
}

static void bmpWrite8(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(bmpAddr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

static uint16_t bmpRead16LE(uint8_t reg) {
  return (uint16_t)bmpRead8(reg) | ((uint16_t)bmpRead8(reg + 1) << 8);
}

// ---- 初始化 ----
bool bmp_begin() {
  // Wire 已由主程序 setup 统一初始化（Wire.begin(8,9)），此处不再重复调用

  // 自动探测地址：0x76 / 0x77
  bmpAddr = 0;
  for (uint8_t a = 0x76; a <= 0x77 && bmpAddr == 0; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) bmpAddr = a;
  }
  if (bmpAddr == 0) return false;

  if (bmpRead8(0xD0) != 0x58) return false;   // CHIP_ID 必须是 0x58

  bmpWrite8(0xE0, 0xB6);   // 软复位
  delay(20);

  // 读出厂校准系数（0x88 起 24 字节，小端）
  dig_T1 = bmpRead16LE(0x88);
  dig_T2 = (int16_t)bmpRead16LE(0x8A);
  dig_T3 = (int16_t)bmpRead16LE(0x8C);
  dig_P1 = bmpRead16LE(0x8E);
  dig_P2 = (int16_t)bmpRead16LE(0x90);
  dig_P3 = (int16_t)bmpRead16LE(0x92);
  dig_P4 = (int16_t)bmpRead16LE(0x94);
  dig_P5 = (int16_t)bmpRead16LE(0x96);
  dig_P6 = (int16_t)bmpRead16LE(0x98);
  dig_P7 = (int16_t)bmpRead16LE(0x9A);
  dig_P8 = (int16_t)bmpRead16LE(0x9C);
  dig_P9 = (int16_t)bmpRead16LE(0x9E);

  // config：IIR 滤波 x16，待机 0.5ms
  bmpWrite8(0xF5, 0x10);
  // ctrl_meas：温度 x2、气压 x16、normal 模式 -> 0x57
  bmpWrite8(0xF4, 0x57);
  delay(20);
  return true;
}

// ---- 温度补偿，返回 0.01℃ ----
static int32_t bmpCompensateT(int32_t adcT) {
  int32_t v1 = ((((adcT >> 3) - ((int32_t)dig_T1 << 1))) * (int32_t)dig_T2) >> 11;
  int32_t v2 = (((((adcT >> 4) - (int32_t)dig_T1) *
                  ((adcT >> 4) - (int32_t)dig_T1)) >> 12) *
                (int32_t)dig_T3) >> 14;
  t_fine = v1 + v2;
  return (t_fine * 5 + 128) >> 8;
}

// ---- 气压补偿，返回 Pa（int64 官方算法，结果 /256）----
static uint32_t bmpCompensateP(int32_t adcP) {
  int64_t v1, v2, p;
  v1 = (int64_t)t_fine - 128000;
  v2 = v1 * v1 * (int64_t)dig_P6;
  v2 = v2 + ((v1 * (int64_t)dig_P5) << 17);
  v2 = v2 + (((int64_t)dig_P4) << 35);
  v1 = ((v1 * v1 * (int64_t)dig_P3) >> 8) + ((v1 * (int64_t)dig_P2) << 12);
  v1 = (((((int64_t)1) << 47) + v1) * (int64_t)dig_P1) >> 33;
  if (v1 == 0) return 0;
  p = 1048576 - adcP;
  p = (((p << 31) - v2) * 3125) / v1;
  v1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
  v2 = (((int64_t)dig_P8) * p) >> 19;
  p = ((p + v1 + v2) >> 8) + (((int64_t)dig_P7) << 4);
  return (uint32_t)(p / 256);
}

// ---- 读一次温度/气压 ----
void bmp_read(float &tempC, float &pressPa) {
  uint8_t raw[6];
  Wire.beginTransmission(bmpAddr);
  Wire.write(0xF7);
  if (Wire.endTransmission() != 0) { pressPa = 0; return; }
  Wire.requestFrom((int)bmpAddr, 6);
  if (Wire.available() != 6) { pressPa = 0; return; }
  for (uint8_t i = 0; i < 6; i++) raw[i] = Wire.read();

  int32_t adcP = ((int32_t)raw[0] << 12) | ((int32_t)raw[1] << 4) | (raw[2] >> 4);
  int32_t adcT = ((int32_t)raw[3] << 12) | ((int32_t)raw[4] << 4) | (raw[5] >> 4);

  int32_t t10 = bmpCompensateT(adcT);
  uint32_t p  = bmpCompensateP(adcP);
  if (p == 0) { pressPa = 0; return; }
  tempC  = t10 / 100.0f;   // t10 单位是 0.01℃
  pressPa = (float)p;
}

// ---- 气压 -> 海拔（m，标准大气模型）----
float bmp_alt_m(float pressPa) {
  if (pressPa <= 0.0f) return 0.0f;
  return 44330.0f * (1.0f - powf(pressPa / BMP_SEA_LEVEL_PA, 1.0f / 5.255f));
}

// ---- 记录起飞参考气压 ----
void bmp_set_ground(float pressPa) {
  if (pressPa > 0.0f) bmp_ground_pa = pressPa;
}

// ---- 相对起飞点高度（m，正=高于起飞点）----
float bmp_rel_alt_m(float pressPa) {
  if (pressPa <= 0.0f || bmp_ground_pa <= 0.0f) return 0.0f;
  return 44330.0f * (1.0f - powf(pressPa / bmp_ground_pa, 1.0f / 5.255f));
}
