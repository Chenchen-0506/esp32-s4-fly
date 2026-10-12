/*
 * [M5] GPS + 罗盘测试 —— ESP32-S3 自研四旋翼
 * =====================================================
 * 功能：读取 BN-880 GPS 模块（NMEA）数据 + 板上罗盘（QMC5883L）航向
 *
 * 接线（GPS 转接线到货后）：
 *   BN-880  VCC  ──► 5V
 *   BN-880  GND  ──► GND
 *   BN-880  TX   ──► GPIO16（ESP32 UART2 RX）
 *   BN-880  RX   ──► GPIO17（ESP32 UART2 TX）
 *   BN-880  SCL  ──► GPIO9   （罗盘 I2C，与 BMP280 共用总线）
 *   BN-880  SDA  ──► GPIO8
 *
 * 串口命令（115200）：
 *   b    重新扫描 GPS 波特率（9600/38400/57600/115200 自动识别）
 *
 * 说明：
 *   - 室内搜不到星属正常，拿到窗边/室外等 1-2 分钟看"定位"变 1 且卫星数上升
 *   - 罗盘显示的是 raw 值 + 简易航向，正式装机前需要做磁校准（届时再给代码）
 */

#include <Arduino.h>
#include <Wire.h>

#define GPS_RX 16
#define GPS_TX 17

// 罗盘：HMC5883 (0x1E) / QMC5883L (0x0D) 自动识别
// （BN-880 参数表标 HMC5883；部分版本为 QMC5883L，两者地址与数据顺序不同）
#define HMC_I2C_ADDR 0x1E
#define QMC_I2C_ADDR 0x0D
#define QMC_REG_X_LSB 0x00
#define QMC_REG_CTRL1 0x09
#define QMC_REG_STAT  0x06

static uint8_t mag_addr = 0;       // 检测到的罗盘地址（0=未找到）
static bool mag_is_hmc = false;    // true=HMC5883, false=QMC5883L

static HardwareSerial gps(2);
static int gps_baud = 0;                 // 0=未锁定
static const int BAUD_LIST[4] = {9600, 38400, 57600, 115200};

static int sat_count = 0;
static int fix_quality = 0;
static float gps_lat = 0.0, gps_lon = 0.0, gps_alt = 0.0;
static String gps_time = "";

/* ---------- 罗盘初始化（HMC5883 / QMC5883L 自动识别） ---------- */
bool qmc_init() {
  Wire.begin(8, 9);                      // SDA=8, SCL=9（与 BMP280 共用）

  // 先试 HMC5883 (0x1E)
  Wire.beginTransmission(HMC_I2C_ADDR);
  if (Wire.endTransmission() == 0) {
    mag_addr = HMC_I2C_ADDR;
    mag_is_hmc = true;
    Wire.beginTransmission(HMC_I2C_ADDR);
    Wire.write(0x02);                    // 模式寄存器
    Wire.write(0x00);                    // 连续测量模式
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
    Wire.write(0x01);                    // 连续测量模式, 50Hz, 8 倍过采样
    Wire.endTransmission();
    return true;
  }
  return false;
}

bool qmc_read(int16_t &x, int16_t &y, int16_t &z) {
  uint8_t base = mag_is_hmc ? 0x03 : QMC_REG_X_LSB;
  Wire.beginTransmission(mag_addr);
  Wire.write(base);
  Wire.endTransmission();
  Wire.requestFrom(mag_addr, (uint8_t)6);
  if (Wire.available() < 6) return false;

  if (mag_is_hmc) {
    // HMC5883 数据顺序：X MSB/LSB, Z MSB/LSB, Y MSB/LSB
    uint8_t xh = Wire.read(), xl = Wire.read();
    uint8_t zh = Wire.read(), zl = Wire.read();
    uint8_t yh = Wire.read(), yl = Wire.read();
    x = (int16_t)((xh << 8) | xl);
    y = (int16_t)((yh << 8) | yl);
    z = (int16_t)((zh << 8) | zl);
  } else {
    // QMC5883L 数据顺序：X LSB/MSB, Y LSB/MSB, Z LSB/MSB
    uint8_t xl = Wire.read(), xh = Wire.read();
    uint8_t yl = Wire.read(), yh = Wire.read();
    uint8_t zl = Wire.read(), zh = Wire.read();
    x = (int16_t)((xh << 8) | xl);
    y = (int16_t)((yh << 8) | yl);
    z = (int16_t)((zh << 8) | zl);
  }
  return true;
}

/* ---------- NMEA 解析（GGA） ---------- */
void parse_gga(String line) {
  // 字段：0=$GNGGA 1=时间 2=纬度 3=N/S 4=经度 5=E/W 6=定位 7=卫星数 8=HDOP 9=海拔
  int f = 0, start = 0;
  String fields[12];
  for (int i = 0; i < (int)line.length() && f < 12; i++) {
    if (line[i] == ',' || i == (int)line.length() - 1) {
      fields[f++] = line.substring(start, i + 1);
      start = i + 1;
    }
  }
  if (f < 10) return;

  gps_time = fields[1];
  fix_quality = fields[6].toInt();
  sat_count = fields[7].toInt();

  // 纬度 ddmm.mmmm → 度
  if (fields[2].length() >= 7) {
    float raw = fields[2].toFloat();
    int deg = (int)(raw / 100);
    gps_lat = deg + (raw - deg * 100) / 60.0;
    if (fields[3] == "S") gps_lat = -gps_lat;
  }
  if (fields[4].length() >= 7) {
    float raw = fields[4].toFloat();
    int deg = (int)(raw / 100);
    gps_lon = deg + (raw - deg * 100) / 60.0;
    if (fields[5] == "W") gps_lon = -gps_lon;
  }
  if (fields[9].length() > 0) gps_alt = fields[9].toFloat();
}

/* ---------- GPS 波特率扫描 ---------- */
bool scan_gps_baud() {
  for (int i = 0; i < 4; i++) {
    gps.begin(BAUD_LIST[i], SERIAL_8N1, GPS_RX, GPS_TX);
    unsigned long t0 = millis();
    String buf = "";
    while (millis() - t0 < 1200) {
      while (gps.available()) {
        char c = gps.read();
        if (c == '\n') {
          buf.trim();
          if (buf.startsWith("$G") && buf.length() > 6) {
            gps_baud = BAUD_LIST[i];
            Serial.printf("GPS 波特率锁定：%d\r\n", gps_baud);
            return true;
          }
          buf = "";
        } else {
          buf += c;
        }
      }
    }
  }
  return false;
}

void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println("[M5] GPS + 罗盘测试（BN-880）");
  Serial.println("注意：室内搜不到星正常，请到窗边或室外等待定位。");

  bool mag_ok = qmc_init();
  if (mag_ok) {
    Serial.printf("罗盘: 在线 0x%02X（%s）\r\n",
                  mag_addr, mag_is_hmc ? "HMC5883" : "QMC5883L");
  } else {
    Serial.println("罗盘: 未找到（检查 SCL/SDA 接线）");
  }

  if (scan_gps_baud()) {
    Serial.println("GPS 模块: 在线（正在搜星...）");
  } else {
    Serial.println("GPS 模块: 未收到数据（检查 TX/RX 接线；串口命令 b 可重扫）");
  }
  Serial.println("命令：b=重扫波特率");
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd == "b") {
      Serial.println("重新扫描 GPS 波特率...");
      scan_gps_baud();
    }
  }

  while (gps.available()) {
    String line = gps.readStringUntil('\n');
    line.trim();
    if (line.startsWith("$GNGGA") || line.startsWith("$GPGGA")) {
      parse_gga(line);
    }
  }

  static unsigned long last_print = 0;
  if (millis() - last_print >= 1000) {
    last_print = millis();

    // GPS 状态
    Serial.printf("[GPS] 定位:%d 卫星:%d 时间:%s\r\n",
                  fix_quality, sat_count, gps_time.c_str());
    if (fix_quality > 0) {
      Serial.printf("       纬度:%.6f 经度:%.6f 高度:%.1f m\r\n",
                    gps_lat, gps_lon, gps_alt);
    }

    // 罗盘状态
    int16_t mx, my, mz;
    if (qmc_read(mx, my, mz)) {
      float heading = atan2((float)my, (float)mx) * 180.0 / 3.14159265;
      if (heading < 0) heading += 360;
      Serial.printf("[MAG] X:%6d Y:%6d Z:%6d 航向(未校准):%.1f°\r\n",
                    mx, my, mz, heading);
    }
  }
}
