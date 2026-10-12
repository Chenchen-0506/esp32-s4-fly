/*
 * [M3] 遥控器 PWM 通道测试 —— ESP32-S3 自研四旋翼
 * =====================================================
 * 功能：读取 FlySky FS-iA6 接收机 6 路 PWM 信号，串口打印脉宽
 *
 * 背景：FS-iA6 接收机只支持 PWM 输出（无 SBUS）。
 *       先用本程序验证"遥控器→接收机→飞控"链路；
 *       后续换 FS-iA6B 接收机后改用 SBUS（代码已备好，见 m3_sbus）。
 *
 * 接线（测试阶段，USB 供电）：
 *   接收机 B/VCC  ──► ESP32 板 5V 引脚（USB 供电时输出 5V）
 *   接收机 GND    ──► ESP32 板 GND
 *   CH1 信号针(S) ──► GPIO14
 *   CH2 信号针(S) ──► GPIO15
 *   CH3 信号针(S) ──► GPIO18
 *   CH4 信号针(S) ──► GPIO21
 *   CH5 信号针(S) ──► GPIO2
 *   CH6 信号针(S) ──► GPIO1
 *
 * 测试步骤：
 *   1) 遥控器先开机（绿灯亮）
 *   2) 接收机通电（B/VCC 接 5V、GND 接地）
 *   3) 烧录本程序，打开串口监视器 115200
 *   4) 摇杆各方向推一遍，看 CH1~CH6 数值：
 *        中位 ≈ 1500，推到底 ≈ 1000 / 2000（单位 µs）
 */

#include <Arduino.h>

#define CH_COUNT 6
const uint8_t CH_PINS[CH_COUNT] = {14, 15, 18, 21, 2, 1};

volatile uint32_t rise_us[CH_COUNT];
volatile uint16_t ch_us[CH_COUNT] = {0, 0, 0, 0, 0, 0};   // 0 = 尚未收到该通道脉冲

/* 每个通道一个中断：上升沿记起点，下降沿算脉宽 */
#define MAKE_ISR(N)                                            \
  void IRAM_ATTR isr_ch##N() {                                 \
    uint32_t t = micros();                                     \
    if (digitalRead(CH_PINS[N])) { rise_us[N] = t; }           \
    else { ch_us[N] = (uint16_t)(t - rise_us[N]); }            \
  }

MAKE_ISR(0)
MAKE_ISR(1)
MAKE_ISR(2)
MAKE_ISR(3)
MAKE_ISR(4)
MAKE_ISR(5)

void setup() {
  Serial.begin(115200);
  delay(200);

  for (int i = 0; i < CH_COUNT; i++) {
    pinMode(CH_PINS[i], INPUT);
  }
  attachInterrupt(digitalPinToInterrupt(CH_PINS[0]), isr_ch0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CH_PINS[1]), isr_ch1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CH_PINS[2]), isr_ch2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CH_PINS[3]), isr_ch3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CH_PINS[4]), isr_ch4, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CH_PINS[5]), isr_ch5, CHANGE);

  Serial.println("[M3] 遥控器 PWM 通道测试 / GPIO 14 15 18 21 2 1");
  Serial.println("等待信号...（遥控器先开机，接收机再接 5V）");
}

void loop() {
  static uint32_t last_print = 0;
  if (millis() - last_print >= 100) {
    last_print = millis();
    for (int i = 0; i < CH_COUNT; i++) {
      Serial.printf("CH%d:", i + 1);
      if (ch_us[i] == 0) Serial.print("----");
      else Serial.printf("%4u", ch_us[i]);
      Serial.print(" ");
    }
    Serial.println();
  }
}
