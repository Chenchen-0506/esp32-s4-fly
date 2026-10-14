/*
 * ============================================================================
 * GPS 功能模块实现 —— 仅 NMEA GGA 解析 + 波特率扫描
 * ============================================================================
 */
#include "GPS.h"

static HardwareSerial gps_ser(2);

float   gps_lat = 0, gps_lon = 0, gps_alt = 0;
bool    gps_fix = false;
uint8_t gps_sats = 0;

static int gps_baud = 0;   // 0=未锁定

/* ---- GGA 解析：$GNGGA/$GPGGA ---- */
static void parse_gga(String line) {
  // 字段：0=$xxGGA 1=时间 2=纬度 3=N/S 4=经度 5=E/W 6=定位 7=卫星数 8=HDOP 9=海拔
  int f = 0, start = 0;
  String fields[12];
  for (int i = 0; i < (int)line.length() && f < 12; i++) {
    if (line[i] == ',' || i == (int)line.length() - 1) {
      fields[f++] = line.substring(start, i + 1);
      start = i + 1;
    }
  }
  if (f < 10) return;

  gps_fix  = (fields[6].toInt() > 0);
  gps_sats = (uint8_t)fields[7].toInt();

  // 纬度 ddmm.mmmm -> 度
  if (fields[2].length() >= 7) {
    float raw = fields[2].toFloat();
    int deg = (int)(raw / 100);
    gps_lat = deg + (raw - deg * 100) / 60.0f;
    if (fields[3] == "S") gps_lat = -gps_lat;
  }
  // 经度 dddmm.mmmm -> 度
  if (fields[4].length() >= 7) {
    float raw = fields[4].toFloat();
    int deg = (int)(raw / 100);
    gps_lon = deg + (raw - deg * 100) / 60.0f;
    if (fields[5] == "W") gps_lon = -gps_lon;
  }
  if (fields[9].length() > 0) gps_alt = fields[9].toFloat();
}

/* ---- 波特率扫描：每档试 1.2s，收到 $G* 开头即锁定 ---- */
static bool scan_baud() {
  static const int BAUD_LIST[4] = {9600, 38400, 57600, 115200};
  for (int i = 0; i < 4; i++) {
    gps_ser.begin(BAUD_LIST[i], SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    unsigned long t0 = millis();
    String buf = "";
    while (millis() - t0 < 1200) {
      while (gps_ser.available()) {
        char c = gps_ser.read();
        if (c == '\n') {
          buf.trim();
          if (buf.startsWith("$G") && buf.length() > 6) {
            gps_baud = BAUD_LIST[i];
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

void gps_begin() {
  gps_baud = 0;
  scan_baud();
}

void gps_poll() {
  while (gps_ser.available()) {
    String line = gps_ser.readStringUntil('\n');
    line.trim();
    if (line.startsWith("$GNGGA") || line.startsWith("$GPGGA")) {
      parse_gga(line);
    }
  }
}
