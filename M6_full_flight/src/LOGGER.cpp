/*
 * ============================================================================
 * LOGGER.cpp —— 飞行黑匣子实现（RAM 环形 + LittleFS 落盘）
 * ============================================================================
 */
#include "LOGGER.h"
#include <LittleFS.h>

static char  ring[LOG_LINES][LOG_LINE_LEN];   // RAM 环形缓冲
static uint32_t seq_total = 0;                // 已写入的总行数（从 1 开始）
static uint32_t seq_flushed = 0;              // 已落盘的行数
static uint32_t last_save_ms = 0;
static bool    fs_ok = false;

void logger_begin() {
  memset(ring, 0, sizeof(ring));
  seq_total = 0;
  seq_flushed = 0;
  last_save_ms = millis();
  fs_ok = LittleFS.begin(true);   // true = 首次挂载失败自动格式化
  if (!fs_ok) {
    Serial.println("[LOG] ⚠ LittleFS 挂载失败！黑匣子/落盘不可用（飞行不受影响）");
  } else {
    Serial.printf("[LOG] 黑匣子就绪：RAM 环形 %d 行 + Flash /log.txt（串口发 D 导出）\n",
                  LOG_LINES);
  }
}

void logger_log(const char* line) {
  uint32_t idx = (uint32_t)(seq_total % LOG_LINES);
  snprintf(ring[idx], LOG_LINE_LEN, "%s", line);
  seq_total++;
  if (seq_total == 0) seq_total = 1;   // 防回绕
}

/* 把 (seq_flushed, seq_total] 的行 append 到 /log.txt */
static void log_append_to_file() {
  if (!fs_ok) return;
  if (seq_total <= seq_flushed) return;

  uint32_t from = seq_flushed + 1;
  if (seq_total - from + 1 > LOG_LINES) from = seq_total - LOG_LINES + 1;

  File f = LittleFS.open("/log.txt", FILE_APPEND);
  if (!f) return;
  for (uint32_t s = from; s <= seq_total; s++) {
    f.print(ring[(s - 1) % LOG_LINES]);   // 第 s 行存在 ring[(s-1)%N]
  }
  f.close();
  seq_flushed = seq_total;

  // 文件上限管理：超出则删除重建（黑匣子保留最近约 1MB）
  if (LittleFS.exists("/log.txt")) {
    File ff = LittleFS.open("/log.txt", "r");
    if (ff) {
      size_t sz = ff.size();
      ff.close();
      if (sz > LOG_FILE_MAX) {
        LittleFS.remove("/log.txt");
        Serial.println("[LOG] 黑匣子文件超上限，已重建（保留最近飞行）");
      }
    }
  }
}

void logger_tick(bool armed) {
  // 飞行中（armed）不落盘：LittleFS 写入会阻塞控制环（数毫秒~百毫秒），
  // 对 1kHz 姿态环是致命的。飞行日志全量由地面站 CSV 实时记录（10Hz 遥测上行），
  // 板载黑匣子保留 RAM 最近 512 行（约 51 秒）作离线兜底，上锁/降落时 logger_flush() 落盘。
  if (armed) return;
  uint32_t now = millis();
  if (now - last_save_ms >= LOG_SAVE_MS) {
    last_save_ms = now;
    log_append_to_file();
  }
}

void logger_flush() {
  log_append_to_file();
  last_save_ms = millis();
}

uint32_t logger_seq() { return seq_total; }

bool logger_line_at(uint32_t seq, char* out) {
  if (seq == 0 || seq > seq_total) return false;
  memcpy(out, ring[(seq - 1) % LOG_LINES], LOG_LINE_LEN);
  return true;
}

void logger_dump_all() {
  Serial.println("\n===== 黑匣子日志 =====");
  if (!fs_ok) {
    Serial.println("（LittleFS 不可用，无落盘数据）");
    return;
  }
  if (!LittleFS.exists("/log.txt")) {
    Serial.println("（暂无落盘日志——飞过后才有）");
    return;
  }
  File f = LittleFS.open("/log.txt", "r");
  if (!f) { Serial.println("（打开失败）"); return; }
  while (f.available()) {
    Serial.write(f.read());
  }
  f.close();
  Serial.println("===== 结束 =====");
}
