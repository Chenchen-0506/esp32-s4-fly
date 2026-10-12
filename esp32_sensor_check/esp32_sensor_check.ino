/*
 * ============================================================
 *  第 2 步：BMP280 气压计 + BN-880 GPS 到货联合自检
 *  零第三方库：I2C 用核心库 Wire，GPS 用硬件串口 Serial2，
 *             BMP280 为寄存器级驱动，不用安装任何库。
 *
 *  ---- 接线（杜邦线，模块全部先不焊，面包板/对插即可）----
 *  BMP280（GY-BMP280，3.3V 版）：
 *    VCC  -> 3V3      GND -> GND
 *    SDA  -> GPIO8    SCL -> GPIO9
 *    CSB  -> 3V3（拉高=I2C 模式；板载已上拉时可悬空，建议接上）
 *    SDO  -> GND（I2C 地址 0x76；悬空多数模块也是 0x76）
 *
 *  BN-880 GPS（自带 6Pin 转杜邦线）：
 *    VCC  -> 5V / VIN（模块 3.3~5V 兼容，以丝印为准）
 *    GND  -> GND
 *    TX   -> GPIO16（GPS 发出 -> 飞控 RX2）
 *    RX   -> GPIO17（飞控 TX2 -> GPS，自检可只接 TX 这一根信号线）
 *    SDA/SCL（内置罗盘）本步不接
 *
 *  现象判读：
 *    - I2C 扫描到 0x76/0x77 = BMP280 接对
 *    - GPS “语句数”在涨 = 接线/波特率正常；放窗边或室外等定位
 *    - GPS 语句数一直为 0 = TX/RX 接反或波特率不符（程序会自动试 4 种波特率）
 * ============================================================
 */
#include <Arduino.h>
#include <Wire.h>

// ---------------- 引脚 ----------------
#define I2C_SDA 8
#define I2C_SCL 9
#define GPS_RX  16   // 飞控 RX2，接 GPS 模块 TX
#define GPS_TX  17   // 飞控 TX2，接 GPS 模块 RX

// ============================================================
//  BMP280 部分
// ============================================================
uint8_t bmpAddr = 0;
uint16_t dig_T1, dig_P1;
int16_t  dig_T2, dig_T3;
int16_t  dig_P2, dig_P3, dig_P4, dig_P5, dig_P6;
int16_t  dig_P7, dig_P8, dig_P9;
int32_t  t_fine = 0;

uint8_t bmpRead8(uint8_t reg) {
  Wire.beginTransmission(bmpAddr);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom((int)bmpAddr, 1);
  return Wire.available() ? Wire.read() : 0xFF;
}

void bmpWrite8(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(bmpAddr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint16_t bmpRead16LE(uint8_t reg) {
  return (uint16_t)bmpRead8(reg) | ((uint16_t)bmpRead8(reg + 1) << 8);
}

bool bmpBegin() {
  // 自动找 0x76 / 0x77
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

// 温度，返回 0.1℃
int32_t bmpCompensateT(int32_t adcT) {
  int32_t v1 = ((((adcT >> 3) - ((int32_t)dig_T1 << 1))) * (int32_t)dig_T2) >> 11;
  int32_t v2 = (((((adcT >> 4) - (int32_t)dig_T1) *
                  ((adcT >> 4) - (int32_t)dig_T1)) >> 12) *
                (int32_t)dig_T3) >> 14;
  t_fine = v1 + v2;
  return (t_fine * 5 + 128) >> 8;
}

// 气压，返回 Pa（int64 官方算法，结果 /256）
uint32_t bmpCompensateP(int32_t adcP) {
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

bool bmpRead(float &tempC, float &pressPa) {
  uint8_t raw[6];
  Wire.beginTransmission(bmpAddr);
  Wire.write(0xF7);
  if (Wire.endTransmission() != 0) return false;
  Wire.requestFrom((int)bmpAddr, 6);
  if (Wire.available() != 6) return false;
  for (uint8_t i = 0; i < 6; i++) raw[i] = Wire.read();

  int32_t adcP = ((int32_t)raw[0] << 12) | ((int32_t)raw[1] << 4) | (raw[2] >> 4);
  int32_t adcT = ((int32_t)raw[3] << 12) | ((int32_t)raw[4] << 4) | (raw[5] >> 4);

  int32_t t10 = bmpCompensateT(adcT);
  uint32_t p  = bmpCompensateP(adcP);
  if (p == 0) return false;
  tempC  = t10 / 100.0f;   // t10 单位是 0.01℃
  pressPa = (float)p;
  return true;
}

// ============================================================
//  GPS（NMEA）部分
// ============================================================
char     gpsLine[140];
uint8_t  gpsLineLen = 0;
uint32_t nmeaGood = 0;          // 校验通过的语句数
bool     gpsFix = false;
uint8_t  gpsSats = 0;
float    gpsLat = 0, gpsLon = 0, gpsAlt = 0;
long     gpsBaud = 0;

bool nmeaChecksumOK(const char *s) {
  const char *star = strchr(s, '*');
  if (!star || star - s < 2) return false;
  uint8_t cs = 0;
  for (const char *p = s + 1; p < star; p++) cs ^= (uint8_t)(*p);
  uint8_t got = (uint8_t)strtol(star + 1, NULL, 16);
  return cs == got;
}

float nmeaToDeg(const char *s) {
  float v = atof(s);
  int deg = (int)(v / 100.0f);
  float minu = v - deg * 100.0f;
  return deg + minu / 60.0f;
}

// 把一行 NMEA 按逗号切成字段
void parseNMEA(char *line) {
  char fields[16][18];
  uint8_t nf = 0;
  char *tok = strtok(line, ",");
  while (tok && nf < 16) {
    strncpy(fields[nf], tok, 17);
    fields[nf][17] = 0;
    nf++;
    tok = strtok(NULL, ",");
  }
  if (nf < 3) return;

  if (strstr(fields[0], "GGA")) {
    if (nf > 9 && fields[6][0]) {
      gpsFix = atoi(fields[6]) > 0;
      gpsSats = (uint8_t)atoi(fields[7]);
      gpsAlt = atof(fields[9]);
    }
  } else if (strstr(fields[0], "RMC")) {
    if (nf > 6) {
      bool valid = fields[2][0] == 'A';
      if (valid && fields[3][0] && fields[5][0]) {
        float lat = nmeaToDeg(fields[3]);
        float lon = nmeaToDeg(fields[5]);
        if (fields[4][0] == 'S') lat = -lat;
        if (fields[6][0] == 'W') lon = -lon;
        gpsLat = lat; gpsLon = lon; gpsFix = true;
      } else if (fields[2][0] == 'V') {
        gpsFix = false;
      }
    }
  }
}

void feedGPS(char c) {
  if (c == '\n') {
    gpsLine[gpsLineLen] = 0;
    if (gpsLineLen > 6 && gpsLine[0] == '$' && nmeaChecksumOK(gpsLine)) {
      nmeaGood++;
      parseNMEA(gpsLine);
    }
    gpsLineLen = 0;
  } else if (c != '\r' && gpsLineLen < sizeof(gpsLine) - 1) {
    gpsLine[gpsLineLen++] = c;
  }
}

// 自动探测 GPS 波特率：9600 / 38400 / 57600 / 115200
long detectGPSBaud() {
  const long cand[] = {9600, 38400, 57600, 115200};
  for (uint8_t i = 0; i < 4; i++) {
    Serial2.updateBaudRate(cand[i]);
    uint32_t t0 = millis();
    uint32_t start = nmeaGood;
    while (millis() - t0 < 1500) {
      while (Serial2.available()) feedGPS((char)Serial2.read());
      if (nmeaGood > start) return cand[i];
    }
  }
  return 0;
}

// ============================================================
//  setup / loop
// ============================================================
float pressRef = 0;   // 上电气压基准，用于相对高度

void setup() {
  Serial.begin(115200);   // 走 UART0 -> CP210x -> COM4
  delay(1500);

  Serial.println();
  Serial.println("================================================");
  Serial.println("   第2步自检：BMP280 气压计 + BN-880 GPS");
  Serial.println("================================================");

  // ---- I2C 扫描 ----
  Wire.begin(I2C_SDA, I2C_SCL);
  Serial.printf("I2C 扫描（SDA=%d SCL=%d）：", I2C_SDA, I2C_SCL);
  uint8_t found = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.printf("0x%02X ", a);
      found++;
    }
  }
  Serial.println(found ? "" : "（没发现任何设备）");

  // ---- BMP280 ----
  if (bmpBegin()) {
    Serial.printf("BMP280：在线，地址 0x%02X，ID=0x58 OK\n", bmpAddr);
    // 等首次转换稳定后，取 10 次平均作气压基准，避免取到坏值
    delay(500);
    float t, p, psum = 0;
    int n = 0;
    for (int i = 0; i < 10; i++) {
      if (bmpRead(t, p)) { psum += p; n++; }
      delay(100);
    }
    if (n > 0) {
      pressRef = psum / n;
      Serial.printf("  上电基准气压 %.1f hPa（%d 次平均）\n", pressRef / 100.0f, n);
    }
  } else {
    Serial.println("BMP280：未找到！检查 VCC=3V3 / SDA=8 / SCL=9 / CSB 接 3V3");
  }

  // ---- GPS ----
  Serial2.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  Serial.println("GPS：正在自动探测波特率（最多 6 秒）...");
  gpsBaud = detectGPSBaud();
  if (gpsBaud) {
    Serial.printf("GPS：在线，锁定波特率 %ld\n", gpsBaud);
  } else {
    Serial.println("GPS：没收到数据。检查 VCC/GND、模块 TX 是否接 GPIO16");
    Serial2.updateBaudRate(9600);
  }
  Serial.println("================================================");
}

void loop() {
  // 持续吃 GPS 数据
  while (Serial2.available()) feedGPS((char)Serial2.read());

  static uint32_t lastPrint = 0, lastRead = 0;
  uint32_t now = millis();

  static float tempC = 0, pressPa = 0;
  if (now - lastRead > 500) {
    lastRead = now;
    if (bmpAddr) bmpRead(tempC, pressPa);
  }

  if (now - lastPrint > 1000) {
    lastPrint = now;
    if (bmpAddr && pressPa > 0) {
      float alt = 44330.0f * (1.0f - powf(pressPa / pressRef, 1.0f / 5.255f));
      Serial.printf("[BMP] %.1f C | %.1f hPa | 相对高度 %+.1f m\n",
                    tempC, pressPa / 100.0f, alt);
    } else {
      Serial.println("[BMP] 无数据");
    }

    if (gpsBaud) {
      Serial.printf("[GPS] 语句%lu | %s | 卫星 %u | %.6f, %.6f | 海拔 %.1f m\n",
                    (unsigned long)nmeaGood,
                    gpsFix ? "已定位" : "未定位(去窗边/室外)",
                    gpsSats, gpsLat, gpsLon, gpsAlt);
    } else {
      Serial.printf("[GPS] 无信号（语句 %lu）\n", (unsigned long)nmeaGood);
    }
    Serial.println();
  }
}
