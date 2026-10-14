/*
 * ============================================================================
 * LOGGER 功能模块 —— 飞行黑匣子（方案 B）
 * ----------------------------------------------------------------------------
 * 两级存储：
 *   RAM 环形缓冲：最近 LOG_LINES 行（1Hz 约 8.5 分钟），WiFi 看板实时读取
 *   Flash 落盘   ：每 LOG_SAVE_MS 把新增行 append 到 LittleFS /log.txt，
 *                  上锁/降落后立即 flush。断电最多丢 1 分钟。
 *
 * 导出：
 *   串口监视器发送 'D' → logger_dump_all() 打印黑匣子全部日志
 *   （飞行中无需 USB；飞完连 USB 发 D 即可回放）
 *
 * 文件上限 LOG_FILE_MAX（约 1MB ≈ 100 次飞行），超出删除重建，
 * 黑匣子始终保留最近一次完整飞行。
 * ============================================================================
 */
#ifndef LOGGER_H
#define LOGGER_H

#include <Arduino.h>

#define LOG_LINES     512      // RAM 环形容量（行）
#define LOG_LINE_LEN  176      // 每行最大字符（含换行，现有打印行约 155 字符）
#define LOG_SAVE_MS   60000UL  // Flash 落盘周期（ms）
#define LOG_FILE_MAX  1000000UL // 黑匣子文件上限（字节）

void  logger_begin();                 // 挂载 LittleFS（失败仅警告，不阻塞飞行）
void  logger_log(const char* line);   // 压入环形（覆盖最旧），供 WiFi/落盘
void  logger_tick();                  // 主循环每环调用（内部 60s 落盘节拍）
void  logger_flush();                 // 立即把未落盘的新行写入 Flash（上锁/降落时）
void  logger_dump_all();              // 串口导出黑匣子全部日志（'D' 命令）
uint32_t logger_seq();                // 当前总行数（WiFi 增量拉取用）
bool  logger_line_at(uint32_t seq, char* out); // 取某行（WiFi 用）

#endif
