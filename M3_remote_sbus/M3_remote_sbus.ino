/*
 * [M3] SBUS 接收机测试 —— ESP32-S3 自研四旋翼
 * =====================================================
 * 功能：读取 FlySky SBUS 接收机信号，串口打印 6 个通道值
 *
 * 接线（测试阶段）：
 *   FlySky 接收机 SBUS 输出 ──► GPIO18
 *   FlySky 接收机 +5V      ──► ESP32 板 5V 引脚（USB 供电时可用，正式装机改 UBEC）
 *   FlySky 接收机 GND      ──► ESP32 GND
 *
 * 关键点：
 *   1) SBUS 是"反相"信号，无需 74HC14 硬件反相器——
 *      Serial2.begin(..., invert=true) 由软件完成反相
 *   2) SBUS 波特率 100000，数据位 8，偶校验，2 停止位（8E2）
 *   3) 每 25ms 一帧，共 25 字节：0x0F 起始 + 22 数据 + 1 标志字节
 *
 * 测试步骤：
 *   1) 遥控器先开机（绿灯亮）
 *   2) 接收机通电（接 5V + GND），对频成功接收机灯常亮
 *   3) 烧录本程序，打开串口监视器 115200
 *   4) 推动遥控器摇杆，看 CH1~CH6 数值变化：
 *      摇杆中位 ≈ 1500，两端 ≈ 1000 / 2000
 */

#include <Arduino.h>

#define SBUS_RX_PIN  18          // 接收机 SBUS 输出接 GPIO18
#define SBUS_BAUD    100000
#define FRAME_LEN    25          // SBUS 一帧 25 字节

static uint16_t ch[16];          // 16 通道原始值（0~2047）
static bool     failsafe = false;
static bool     lost_frame = false;
static uint32_t last_print = 0;

/* 解析一帧 SBUS 数据（25 字节） */
bool sbus_parse(const uint8_t *b) {
  if (b[0] != 0x0F) return false;                       // 校验起始字节

  ch[0]  = ((b[1]       | b[2] << 8)                    & 0x07FF);
  ch[1]  = ((b[2] >> 3  | b[3] << 5)                    & 0x07FF);
  ch[2]  = ((b[3] >> 6  | b[4] << 2 | b[5] << 10)       & 0x07FF);
  ch[3]  = ((b[5] >> 1  | b[6] << 7)                    & 0x07FF);
  ch[4]  = ((b[6] >> 4  | b[7] << 4)                    & 0x07FF);
  ch[5]  = ((b[7] >> 7  | b[8] << 1 | b[9] << 9)        & 0x07FF);
  ch[6]  = ((b[9] >> 2  | b[10] << 6)                   & 0x07FF);
  ch[7]  = ((b[10] >> 5 | b[11] << 3)                   & 0x07FF);
  ch[8]  = ((b[12]      | b[13] << 8)                   & 0x07FF);
  ch[9]  = ((b[13] >> 3 | b[14] << 5)                   & 0x07FF);
  ch[10] = ((b[14] >> 6 | b[15] << 2 | b[16] << 10)     & 0x07FF);
  ch[11] = ((b[16] >> 1 | b[17] << 7)                   & 0x07FF);
  ch[12] = ((b[17] >> 4 | b[18] << 4)                   & 0x07FF);
  ch[13] = ((b[18] >> 7 | b[19] << 1 | b[20] << 9)      & 0x07FF);
  ch[14] = ((b[20] >> 2 | b[21] << 6)                   & 0x07FF);
  ch[15] = ((b[21] >> 5 | b[22] << 3)                   & 0x07FF);

  uint8_t flags = b[23];
  failsafe  = (flags & 0x08) != 0;
  lost_frame = (flags & 0x10) != 0;
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(200);

  // SBUS：100000 波特、8E2、RX=GPIO18、软件反相 invert=true
  Serial2.begin(SBUS_BAUD, SERIAL_8E2, SBUS_RX_PIN, -1, true);

  Serial.println("[M3] SBUS 接收机测试 / 反相信号由软件处理 / GPIO18");
  Serial.println("等待 SBUS 信号...（先开遥控器再给接收机通电）");
}

void loop() {
  static uint8_t buf[FRAME_LEN];
  static uint8_t idx = 0;

  while (Serial2.available()) {
    uint8_t b = Serial2.read();

    if (idx == 0 && b != 0x0F) continue;   // 未同步时等待帧头
    buf[idx++] = b;

    if (idx >= FRAME_LEN) {                // 一帧收完
      idx = 0;
      if (sbus_parse(buf)) {
        // 每 100ms 打印一次（SBUS 25ms 一帧，4 帧打一次即可）
        if (millis() - last_print >= 100) {
          last_print = millis();
          Serial.printf("CH1:%4u CH2:%4u CH3:%4u CH4:%4u CH5:%4u CH6:%4u",
                        ch[0], ch[1], ch[2], ch[3], ch[4], ch[5]);
          if (failsafe)  Serial.print("  [FS=失控保护]");
          if (lost_frame) Serial.print("  [LF=丢帧]");
          Serial.println();
        }
      }
    }
  }

  // 2 秒没收到帧：提示接收机未通电 / 未对频
  static uint32_t last_rx = 0;
  if (Serial2.available() == 0 && millis() - last_rx > 2000) {
    last_rx = millis();
    Serial.println("[!] 未收到 SBUS 帧：检查接线 / 接收机是否通电 / 遥控器是否开机");
  }
}
