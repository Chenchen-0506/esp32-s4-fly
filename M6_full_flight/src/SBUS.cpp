/*
 * ============================================================================
 * SBUS 功能模块 —— 实现文件
 * 原 M3/M6 已验证解析逻辑，仅封装为 begin/poll/get_lost 接口
 * ============================================================================
 */
#include "SBUS.h"

uint16_t rc_ch[16] = {1500, 1500, 1000, 1500, 1500, 1500, 1500, 1500,
                      1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
bool     rc_failsafe = false;

static uint32_t rc_last_frame_ms = 0;

void sbus_begin() {
  // SBUS：100000 波特、8E2、RX=GPIO18、软件反相 invert=true
  Serial2.begin(SBUS_BAUD, SERIAL_8E2, SBUS_RX_PIN, -1, true);
}

bool sbus_parse(const uint8_t *b) {
  if (b[0] != 0x0F) return false;                       // 起始字节校验

  rc_ch[0]  = ((b[1]       | b[2] << 8)                    & 0x07FF);
  rc_ch[1]  = ((b[2] >> 3  | b[3] << 5)                    & 0x07FF);
  rc_ch[2]  = ((b[3] >> 6  | b[4] << 2 | b[5] << 10)       & 0x07FF);
  rc_ch[3]  = ((b[5] >> 1  | b[6] << 7)                    & 0x07FF);
  rc_ch[4]  = ((b[6] >> 4  | b[7] << 4)                    & 0x07FF);
  rc_ch[5]  = ((b[7] >> 7  | b[8] << 1 | b[9] << 9)        & 0x07FF);
  rc_ch[6]  = ((b[9] >> 2  | b[10] << 6)                   & 0x07FF);
  rc_ch[7]  = ((b[10] >> 5 | b[11] << 3)                   & 0x07FF);
  rc_ch[8]  = ((b[12]      | b[13] << 8)                   & 0x07FF);
  rc_ch[9]  = ((b[13] >> 3 | b[14] << 5)                   & 0x07FF);
  rc_ch[10] = ((b[14] >> 6 | b[15] << 2 | b[16] << 10)     & 0x07FF);
  rc_ch[11] = ((b[16] >> 1 | b[17] << 7)                   & 0x07FF);
  rc_ch[12] = ((b[17] >> 4 | b[18] << 4)                   & 0x07FF);
  rc_ch[13] = ((b[18] >> 7 | b[19] << 1 | b[20] << 9)      & 0x07FF);
  rc_ch[14] = ((b[20] >> 2 | b[21] << 6)                   & 0x07FF);
  rc_ch[15] = ((b[21] >> 5 | b[22] << 3)                   & 0x07FF);

  uint8_t flags = b[23];
  rc_failsafe   = (flags & 0x08) != 0;
  return true;
}

void sbus_poll() {
  static uint8_t buf[SBUS_FRAME_LEN];
  static uint8_t idx = 0;

  while (Serial2.available()) {
    uint8_t b = Serial2.read();
    if (idx == 0 && b != 0x0F) continue;   // 未同步时等待帧头
    buf[idx++] = b;
    if (idx >= SBUS_FRAME_LEN) {
      idx = 0;
      if (sbus_parse(buf)) rc_last_frame_ms = millis();
    }
  }
}

bool sbus_get_lost() {
  return (millis() - rc_last_frame_ms > 1000) || rc_failsafe;
}
