/*
 * ============================================================================
 * 遥控接收模块实现 —— FlySky iBus 协议（FS-iA6B），RMT 硬件解码
 * ----------------------------------------------------------------------------
 * 帧格式（32 字节）：
 *   0x20 0x40 | CH1L CH1H ... CH14L CH14H | CHECK_L CHECK_H
 *   通道值小端 2 字节，1000~2000µs 直接对应
 *   校验和 = 0xFFFF - Σ(前 30 字节)，低字节在前（高字节通常 0x00）
 *
 * 解码原理（19200 波特 8N1，1 bit ≈ 52µs）：
 *   - RMT RX 通道（GPIO18，1 tick=1µs，DMA 缓冲 512 symbols）等待空闲高电平后的
 *     第一个下降沿开始记录（即 start bit），每个 symbol word 含两段电平+时长
 *   - 帧 32 字节 × 10 bits = 320 bits，DMA 512 symbols 足以容纳最坏交替位模式
 *   - sbus_poll() 把符号序列量化回 bit 流（52µs/bit 四舍五入），按 10 bits/字节
 *     尝试 0~9 偏移对齐，找 0x20 0x40 帧头 + 校验和，通过即更新 rc_ch[]
 * ============================================================================
 */
#include "SBUS.h"

#include "driver/rmt_rx.h"
#include "driver/rmt_types.h"
#include "esp_attr.h"

/* ---- RMT 接收资源 ---- */
#define SBUS_RMT_SYMBOLS   512      // DMA 缓冲（symbols）；iBus 帧最长 ~320 symbols
#define SBUS_BIT_US        52       // 19200 波特下 1 bit 时长（µs）
static rmt_channel_handle_t    sbus_rmt = NULL;
static rmt_symbol_word_t      sbus_sym[SBUS_RMT_SYMBOLS];
static rmt_receive_config_t   sbus_recv_cfg;
static volatile bool          sbus_sym_ready = false;
static volatile uint32_t      sbus_sym_cnt = 0;

/* ---- 输出与信号质量状态 ---- */
uint16_t rc_ch[16] = {1500, 1500, 1000, 1500, 1500, 1500, 1500, 1500,
                      1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};

static uint32_t rc_last_frame_ms = 0;
#define SBUS_EXPECT_FPS  130      // iBus 帧率约 130 帧/s（7.7ms 周期）
static uint32_t rc_frames_sec = 0;
static uint32_t rc_sec_anchor = 0;
static uint8_t  rc_sig = 100;

/* ---- RMT 接收完成回调（ISR 上下文）：拷贝符号、置标志、重新布防 ---- */
static bool IRAM_ATTR sbus_rmt_done(rmt_channel_handle_t ch,
                                    const rmt_rx_done_event_data_t *ed,
                                    void *arg) {
  uint32_t n = ed->num_symbols;
  if (n > 0 && n <= SBUS_RMT_SYMBOLS) {
    memcpy((void *)sbus_sym, ed->received_symbols,
           n * sizeof(rmt_symbol_word_t));
    sbus_sym_cnt = n;
    sbus_sym_ready = true;
  }
  // 重新布防接收下一帧
  rmt_receive(ch, sbus_sym, sizeof(sbus_sym), &sbus_recv_cfg);
  return false;
}

void sbus_begin() {
  rmt_rx_channel_config_t cfg = {};
  cfg.gpio_num         = (gpio_num_t)SBUS_RX_PIN;
  cfg.clk_src          = RMT_CLK_SRC_DEFAULT;
  cfg.resolution_hz    = 1000000;              // 1 tick = 1µs
  cfg.mem_block_symbols = SBUS_RMT_SYMBOLS;
  cfg.flags.invert_in  = false;
  cfg.flags.with_dma   = true;                 // S3 支持 DMA，可收满整帧
  if (rmt_new_rx_channel(&cfg, &sbus_rmt) != ESP_OK) {
    Serial.println("[RC] RMT RX 通道创建失败！");
    return;
  }

  rmt_rx_event_callbacks_t cb = {};
  cb.on_recv_done = sbus_rmt_done;
  rmt_rx_register_event_callbacks(sbus_rmt, &cb, NULL);

  sbus_recv_cfg.signal_range_min_ns = 1000;     // 滤掉 <1µs 毛刺
  sbus_recv_cfg.signal_range_max_ns = 600000;   // 最长 ~11 bit（帧内不会更长）
  rmt_enable(sbus_rmt);
  rmt_receive(sbus_rmt, sbus_sym, sizeof(sbus_sym), &sbus_recv_cfg);

  rc_last_frame_ms = millis();   // 上电即视为有帧，避免首秒误判丢信号
  Serial.println("[RC] iBus RMT 解码就绪：GPIO18 19200（不占用 UART）");
}

/* ---- 校验并解出 14 通道（帧头 + 校验和验证通过才更新）---- */
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

/* ---- 符号序列 → bit 流 → 32 字节帧（自动对齐）---- */
static bool ibus_decode_symbols(const rmt_symbol_word_t *sym, uint32_t n,
                                uint8_t *frame) {
  // 展开成 bit 流：每段电平持续 µs/52 四舍五入（clamp 1~12），电平 0/1
  static uint8_t bits[1200];
  uint32_t nbits = 0;
  for (uint32_t i = 0; i < n && nbits < sizeof(bits); i++) {
    // level0 / duration0
    uint32_t d0 = sym[i].duration0 / SBUS_BIT_US;
    if (d0 < 1) d0 = 1;
    if (d0 > 12) d0 = 12;
    for (uint32_t k = 0; k < d0 && nbits < sizeof(bits); k++)
      bits[nbits++] = (sym[i].level0 & 1);
    // level1 / duration1（末 symbol 可能只有半段，靠 length 判断）
    if (sym[i].level1 > 0 || sym[i].duration1 > 0) {
      uint32_t d1 = sym[i].duration1 / SBUS_BIT_US;
      if (d1 < 1) d1 = 1;
      if (d1 > 12) d1 = 12;
      for (uint32_t k = 0; k < d1 && nbits < sizeof(bits); k++)
        bits[nbits++] = (sym[i].level1 & 1);
    }
  }
  if (nbits < SBUS_FRAME_LEN * 10) return false;

  // 尝试 0~9 位偏移：每字节 = start(0) + 8 数据 + stop(1)，找 0x20 0x40 帧头
  for (int off = 0; off < 10; off++) {
    uint8_t tmp[SBUS_FRAME_LEN];
    bool ok = true;
    for (int b = 0; b < SBUS_FRAME_LEN; b++) {
      int base = off + b * 10;
      if (base + 10 > (int)nbits) { ok = false; break; }
      if (bits[base] != 0) { ok = false; break; }            // start bit 应为 0
      uint8_t v = 0;
      for (int k = 0; k < 8; k++)
        if (bits[base + 1 + k]) v |= (uint8_t)(1 << k);      // 数据位 LSB 在前
      if (bits[base + 9] != 1) { ok = false; break; }        // stop bit 应为 1
      tmp[b] = v;
    }
    if (ok && tmp[0] == 0x20 && tmp[1] == 0x40) {
      // 再用完整校验和确认（可选；帧头已强约束，误检率极低）
      memcpy(frame, tmp, SBUS_FRAME_LEN);
      return true;
    }
  }
  return false;
}

void sbus_poll() {
  if (!sbus_sym_ready) return;
  sbus_sym_ready = false;

  uint8_t frame[SBUS_FRAME_LEN];
  if (ibus_decode_symbols(sbus_sym, sbus_sym_cnt, frame)) {
    if (sbus_parse(frame)) {
      rc_last_frame_ms = millis();
      rc_frames_sec++;
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
