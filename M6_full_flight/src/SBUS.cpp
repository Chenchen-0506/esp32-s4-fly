/*
 * ============================================================================
 * 遥控接收模块实现 —— FlySky iBus 协议（FS-iA6B）
 * ----------------------------------------------------------------------------
 * 帧格式（32 字节）：
 *   0x20 0x40 | CH1L CH1H ... CH14L CH14H | CHECK_L CHECK_H
 *   通道值小端 2 字节，1000~2000µs 直接对应
 *   校验和 = 0xFFFF - Σ(前 30 字节)，低字节在前（高字节通常 0x00）
 * 串口：19200 baud 8N1，RX=GPIO18，无需反相
 * ============================================================================
 */
#include "SBUS.h"

uint16_t rc_ch[16] = {1500, 1500, 1000, 1500, 1500, 1500, 1500, 1500,
                      1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
bool     rc_failsafe = false;

static uint32_t rc_last_frame_ms = 0;
/* 信号质量：最近 1 秒实际 iBus 帧数 / 期望帧数（约 130 帧/s，7.7ms 周期） */
#define SBUS_EXPECT_FPS  130
static uint32_t rc_frames_sec = 0;
static uint32_t rc_sec_anchor = 0;
static uint8_t  rc_sig = 100;

void sbus_begin() {
  // iBus：19200 波特、8N1、RX=GPIO18（无需反相）
  Serial2.begin(SBUS_BAUD, SERIAL_8N1, SBUS_RX_PIN, -1);
}

bool sbus_parse(const uint8_t *b) {
  if (b[0] != 0x20 || b[1] != 0x40) return false;        // 帧头校验

  uint16_t sum = 0;
  for (int i = 0; i < 30; i++) sum += b[i];
  uint16_t ck = (uint16_t)(0xFFFF - sum);
  if (b[30] != (ck & 0xFF) || b[31] != (ck >> 8)) return false;  // 校验和

  for (int ch = 0; ch < 14; ch++) {
    uint16_t v = b[2 + ch * 2] | (uint16_t)(b[3 + ch * 2] << 8);
    if (v >= 800 && v <= 2200) rc_ch[ch] = v;   // 只收有效范围；空闲/无效通道保持原值
  }
  return true;
}

void sbus_poll() {
  static uint8_t buf[SBUS_FRAME_LEN];
  static uint8_t idx = 0;

  while (Serial2.available()) {
    uint8_t b = Serial2.read();
    if (idx == 0 && b != 0x20) continue;                  // 未同步：等帧头第 1 字节
    if (idx == 1 && b != 0x40) { idx = 0; continue; }     // 第 2 字节不符：重同步
    buf[idx++] = b;
    if (idx >= SBUS_FRAME_LEN) {
      idx = 0;
      if (sbus_parse(buf)) { rc_last_frame_ms = millis(); rc_frames_sec++; }
    }
  }
}

bool sbus_get_lost() {
  return (millis() - rc_last_frame_ms > 1000);
}

uint8_t sbus_signal_pct() {
  uint32_t now = millis();
  if (now - rc_sec_anchor >= 1000UL) {
    uint32_t got = rc_frames_sec * 100 / SBUS_EXPECT_FPS;
    if (got > 100) got = 100;
    rc_sig = (uint8_t)((rc_sig * 7 + got * 3) / 10);   // 平滑，防抖
    rc_frames_sec = 0;
    rc_sec_anchor = now;
  }
  return rc_sig;
}
