/*
 * ============================================================================
 * LINK.cpp —— 433MHz 数传链路实现（UART1）
 * ============================================================================
 */
#include "LINK.h"

static HardwareSerial link_ser(1);   // UART1（GPS 用 Serial2、SBUS 用 Serial2，UART1 空闲）
static String link_buf;

void link_begin() {
  link_ser.begin(LINK_BAUD, SERIAL_8N1, LINK_RX_PIN, LINK_TX_PIN);
  link_ser.setTxBufferSize(2048);   // 大 TX 缓冲：10Hz 遥测帧(~400B)写入缓冲即返回，不阻塞 1kHz 控制环
  link_buf.reserve(256);
}

void link_send(const String &line) {
  link_ser.println(line);
}

bool link_read_line(String &out) {
  while (link_ser.available()) {
    char c = (char)link_ser.read();
    if (c == '\n') {
      link_buf.trim();
      if (link_buf.length() > 0) {
        out = link_buf;
        link_buf = "";
        return true;
      }
      link_buf = "";
    } else {
      if (link_buf.length() < 255) link_buf += c;   // 防御超长垃圾行
    }
  }
  return false;
}
