/*
 * [M4] 电调油门校准 + 单电机测试 —— ESP32-S3 自研四旋翼
 * =====================================================
 * 功能：校准 4 个 BLHeli 电调的油门行程，并逐个电机缓慢试转
 *
 * ⚠️ 安全铁律（每次通电前必做）：
 *   - 必须拆掉所有螺旋桨！严禁带桨通电！
 *   - 桌面测试 → 测试架 → 系留 → 首飞（顺序不可跳）
 *   - 接电池时拔掉 USB；USB 调试时不要接电池
 *   - 电机转动时手/头发/电线远离电机轴
 *
 * 接线：
 *   电调 1 信号线 ──► GPIO4
 *   电调 2 信号线 ──► GPIO5
 *   电调 3 信号线 ──► GPIO6
 *   电调 4 信号线 ──► GPIO7
 *   （电调 BEC 红线全挑出热缩不接！全机唯一 5V = 好盈 UBEC）
 *
 * 串口命令（115200）：
 *   cal   进入油门校准流程
 *   1 2 3 4  选择电机（默认全部）
 *   + / -    当前选中电机油门 +50 / -50 µs
 *   0        全部油门回 1000（急停）
 *   t        当前选中电机自动缓升到 2000 再回落（自动测试）
 *
 * 油门标准：50Hz PWM，1000µs=最低，2000µs=最高
 */

#include <Arduino.h>

#define M1_PIN 4
#define M2_PIN 5
#define M3_PIN 6
#define M4_PIN 7

#define MIN_US 1000
#define MAX_US 2000
#define PWM_RES_BITS 14                      // 分辨率 14bit = 16384 级
#define PERIOD_US 20000                      // 50Hz 周期

static const uint8_t MOTOR_PINS[4] = {M1_PIN, M2_PIN, M3_PIN, M4_PIN};
static uint16_t throttle_us[4] = {MIN_US, MIN_US, MIN_US, MIN_US};

/* 设置某电机油门（µs） */
void set_motor_us(uint8_t idx, uint16_t us) {
  if (us < MIN_US) us = MIN_US;
  if (us > MAX_US) us = MAX_US;
  throttle_us[idx] = us;
  uint32_t duty = (uint32_t)us * (1UL << PWM_RES_BITS) / PERIOD_US;
  analogWrite(MOTOR_PINS[idx], duty);
}

void set_all_us(uint16_t us) {
  for (int i = 0; i < 4; i++) set_motor_us(i, us);
}

void print_status() {
  Serial.printf("油门: M1:%4u M2:%4u M3:%4u M4:%4u\r\n",
                throttle_us[0], throttle_us[1], throttle_us[2], throttle_us[3]);
}

/* 电调油门校准流程 */
void do_calibration() {
  Serial.println("=== 电调油门校准 ===");
  Serial.println("[1/3] 请确认：桨已拆掉？油门会先推到最高。");
  Serial.println("      然后接上电池 → 每个电调应发长哔一声（进入校准）。");
  Serial.println("      听到哔声后，在串口输入任意字符继续。");
  set_all_us(MAX_US);

  while (!Serial.available()) delay(10);
  while (Serial.available()) Serial.read();

  Serial.println("[2/3] 油门拉到最低（1000µs）...");
  set_all_us(MIN_US);
  delay(300);

  Serial.println("[3/3] 电调应发短哔确认（N 声=识别到电池节数）。校准完成！");
  Serial.println("      拔电池重新上电，电调自检哔声正常后即可试转。");
  print_status();
}

/* 当前选中电机自动缓升缓降（0xFF = 全部四路一起） */
void auto_test(uint8_t idx) {
  if (idx == 0xFF) {
    Serial.println("自动测试全部电机：1000→2000→1000");
    for (uint16_t us = MIN_US; us <= MAX_US; us += 20) {
      set_all_us(us);
      delay(30);
    }
    for (uint16_t us = MAX_US; us >= MIN_US; us -= 20) {
      set_all_us(us);
      delay(30);
    }
    set_all_us(MIN_US);
    print_status();
    return;
  }
  Serial.printf("自动测试电机 %d：1000→2000→1000\r\n", idx + 1);
  for (uint16_t us = MIN_US; us <= MAX_US; us += 20) {
    set_motor_us(idx, us);
    delay(30);
  }
  for (uint16_t us = MAX_US; us >= MIN_US; us -= 20) {
    set_motor_us(idx, us);
    delay(30);
  }
  set_motor_us(idx, MIN_US);
  print_status();
}

void setup() {
  Serial.begin(115200);
  delay(300);

  for (int i = 0; i < 4; i++) {
    pinMode(MOTOR_PINS[i], OUTPUT);
    analogWriteResolution(MOTOR_PINS[i], PWM_RES_BITS);
    analogWriteFrequency(MOTOR_PINS[i], 50);   // 电调标准 50Hz
  }
  set_all_us(MIN_US);                           // 上电默认油门最低，电机不转

  Serial.println("[M4] 电调校准 + 单电机测试");
  Serial.println("⚠ 通电前确认：螺旋桨已拆！电池未接！");
  Serial.println("命令：cal=校准  1-4=选电机  +=/ -=油门 0=急停 t=自动测试");
  print_status();
}

void loop() {
  static uint8_t sel = 0xFF;   // 0xFF = 全部

  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "cal") {
      do_calibration();
    }
    else if (cmd == "1" || cmd == "2" || cmd == "3" || cmd == "4") {
      sel = cmd[0] - '1';
      Serial.printf("选中电机 %d\r\n", sel + 1);
      print_status();
    }
    else if (cmd == "0") {
      set_all_us(MIN_US);
      Serial.println("急停：全部油门 1000");
      print_status();
    }
    else if (cmd == "+") {
      for (int i = 0; i < 4; i++) {
        if (sel == 0xFF || sel == i) set_motor_us(i, throttle_us[i] + 50);
      }
      print_status();
    }
    else if (cmd == "-") {
      for (int i = 0; i < 4; i++) {
        if (sel == 0xFF || sel == i) set_motor_us(i, throttle_us[i] - 50);
      }
      print_status();
    }
    else if (cmd == "t") {
      auto_test(sel);   // 0xFF = 全部四路一起
    }
    else if (cmd == "all") {
      sel = 0xFF;
      Serial.println("选中全部电机");
    }
    else {
      Serial.println("未知命令。cal / 1-4 / + / - / 0 / t / all");
    }
  }
}
