/*
 * ============================================================
 *  ESP32-S3 开发板到货自检（第 1 步）
 *  目的：验证板子真伪（ESP32-S3 / 16MB Flash / 8MB PSRAM）
 *        验证 Arduino IDE 能编译、能烧录、串口能通信
 *  零依赖：只用 ESP32 核心库，不需要安装任何第三方库
 *  接线：只需一根 USB-C 数据线连电脑，先不接任何模块
 * ============================================================
 */
#include <Arduino.h>

void setup() {
  Serial.begin(115200);
  delay(2000);  // 给 USB CDC 一点枚举时间

  Serial.println();
  Serial.println("================================================");
  Serial.println("       ESP32-S3 开发板到货自检");
  Serial.println("================================================");

  Serial.printf("芯片型号 : %s\n", ESP.getChipModel());
  Serial.printf("内核数量 : %d 核\n", ESP.getChipCores());
  Serial.printf("CPU 频率 : %d MHz\n", ESP.getCpuFreqMHz());
  Serial.printf("芯片修订 : %d\n", ESP.getChipRevision());

  uint32_t flashMB = ESP.getFlashChipSize() / (1024UL * 1024UL);
  uint32_t psramMB  = ESP.getPsramSize()    / (1024UL * 1024UL);
  Serial.printf("Flash    : %u MB  (N16 应为 16)\n", (unsigned)flashMB);
  Serial.printf("PSRAM    : %u MB  (R8 应为 8)\n", (unsigned)psramMB);
  Serial.printf("SDK 版本 : %s\n", ESP.getSdkVersion());

  // ---- 自动判定 ----
  bool isS3    = (strcmp(ESP.getChipModel(), "ESP32-S3") == 0);
  bool flashOK = (flashMB >= 16);
  bool psramOK = (psramMB  >= 8);

  Serial.println("------------------------------------------------");
  if (isS3 && flashOK && psramOK) {
    Serial.println(">>> 结论：板子确认 = ESP32-S3 N16R8 真品，环境正常！");
  } else {
    Serial.println(">>> 结论：规格与 N16R8 不符，请把以上整段文字发我：");
    if (!isS3)    Serial.println("    - 芯片不是 ESP32-S3（可能是 C3/S2 冒充）");
    if (!flashOK) Serial.println("    - Flash 不足 16MB（或 Tools 里 Flash Size 没选 16MB）");
    if (!psramOK) Serial.println("    - PSRAM 不是 8MB（Tools 里 PSRAM 要选 OPI PSRAM）");
  }
  Serial.println("================================================");
}

void loop() {
  static uint32_t cnt = 0;
  Serial.printf("心跳 %lu：板子运行正常\n", (unsigned long)(++cnt));
  delay(1000);
}
