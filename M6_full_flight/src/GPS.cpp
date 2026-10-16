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
static void parse_gga(const char *line) {
  // 字段：0=$xxGGA 1=时间 2=纬度 3=N/S 4=经度 5=E/W 6=定位 7=卫星数 8=HDOP 9=海拔
  char buf[96];
  strncpy(buf, line, sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = 0;
  char *f[12] = {0};
  int fc = 0;
  char *tok = strtok(buf, ",");
  while (tok && fc < 12) { f[fc++] = tok; tok = strtok(NULL, ","); }
  if (fc < 10) return;

  gps_fix  = (atoi(f[6]) > 0);
  gps_sats = (uint8_t)atoi(f[7]);

  // 纬度 ddmm.mmmm -> 度
  if (strlen(f[2]) >= 7) {
    float raw = atof(f[2]);
    int deg = (int)(raw / 100);
    gps_lat = deg + (raw - deg * 100) / 60.0f;
    if (f[3][0] == 'S') gps_lat = -gps_lat;
  }
  // 经度 dddmm.mmmm -> 度
  if (strlen(f[4]) >= 7) {
    float raw = atof(f[4]);
    int deg = (int)(raw / 100);
    gps_lon = deg + (raw - deg * 100) / 60.0f;
    if (f[5][0] == 'W') gps_lon = -gps_lon;
  }
  if (f[9][0]) gps_alt = atof(f[9]);
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
  // 逐字符解析，绝不阻塞：readStringUntil 在无 '\n' 时会卡到超时（默认 1s），
  // 会让 1kHz 控制环瞬间停摆，飞行中是致命的
  static char linebuf[96];
  static uint8_t li = 0;
  while (gps_ser.available()) {
    char c = (char)gps_ser.read();
    if (c == '\n') {
      linebuf[li] = 0;
      if (li >= 6 &&
          (strncmp(linebuf, "$GNGGA", 6) == 0 ||
           strncmp(linebuf, "$GPGGA", 6) == 0)) {
        parse_gga(linebuf);
      }
      li = 0;
    } else if (li < sizeof(linebuf) - 1) {
      linebuf[li++] = c;   // 限长，防无界增长
    }
  }
}
