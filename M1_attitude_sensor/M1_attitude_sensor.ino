/*
 * ============================================================================
 * ESP32-S3 自研飞控 —— M1 阶段（ICM-20948 SPI 版）
 * ----------------------------------------------------------------------------
 * 本文件对应采购 BOM：
 *   主控 ESP32-S3-DevKitC N16R8 ；IMU = ICM-20948 九轴模块（走 SPI）
 *
 * M1 目标（本文件已完成、到货即可验证）：
 *   1) SPI 寄存器级驱动 ICM-20948，读取陀螺仪 + 加速度计
 *   2) 上电自检 WHO_AM_I（应为 0xEA）
 *   3) 静止陀螺零偏校准
 *   4) Mahony 6 轴姿态解算（1kHz 主环），串口输出 roll,pitch,yaw(度)
 *   5) BMP280 气压计(I2C，GPIO8/9) 温度/气压/相对高度（驱动已并入，随本文件一次验证）
 *
 * 后续阶段（本文件末尾有引脚预留与说明）：
 *   - 磁力计 AK09916（ICM 内部 AUX I2C）+ 9 轴航向融合  -> M1 第二步
 *   - SBUS 遥控解析(UART)                                -> M3
 *   - 电调 PWM/DShot 输出 + 串级 PID                     -> M3/M4
 *
 * ----------------------------------------------------------------------------
 * 【接线（ESP32-S3-DevKitC -> ICM-20948 模块）】
 *   ICM 模块     ESP32-S3 引脚
 *   VCC      ->  3V3
 *   GND      ->  GND
 *   SCK/SCL  ->  GPIO12
 *   MISO/SDO ->  GPIO13
 *   MOSI/SDA ->  GPIO11
 *   CS/NCS   ->  GPIO10
 *   （模块上若有 AD0/SA0、I2C/SPI 选择，按模块丝印切到 SPI；CS 必须接，不能悬空）
 *
 * 【开发环境】
 *   Arduino IDE 2.x ；开发板包 esp32 by Espressif（3.x）
 *   开发板选 "ESP32S3 Dev Module" ；串口监视器 115200
 *
 * 【到货验证步骤】
 *   1) 烧录后看串口：WHO_AM_I 应打印 EA；若打印 00/FF -> 接线/CS/SPI 模式问题
 *   2) 提示校准时保持板子水平静止 2 秒
 *   3) 用手缓慢转动板子，roll/pitch 应跟随且回正后接近 0
 *   4) 6 轴稳定后，再叠加磁力计（9 轴）与 BMP280
 *
 * 安全：本阶段不接电调/电机，或电机一律拆桨。
 * ============================================================================
 */

#include <SPI.h>
#include <math.h>
#include <Wire.h>

/* ============================================================================
 * 引脚定义（ESP32-S3-DevKitC N16R8）
 * ========================================================================== */
// ICM-20948 —— SPI（FSPI / VSPI 兼容引脚）
#define ICM_SCK       12
#define ICM_MISO      13
#define ICM_MOSI      11
#define ICM_CS        10
#define ICM_SPI_HZ    8000000UL      // 8MHz，先求稳；验证后可升到 16~24MHz

// BMP280 气压计（I2C，GPIO8/9，与 ICM 的 FSPI 不冲突）
#define BMP_SDA     8
#define BMP_SCL     9

// 预留：SBUS 接收机（UART，M3）
// #define SBUS_RX     18   // 经反相器后接此脚

// 预留：4 路电调 PWM（M4，测试全程拆桨）
// #define MOTOR1_PIN  4
// #define MOTOR2_PIN  5
// #define MOTOR3_PIN  6
// #define MOTOR4_PIN  7

/* ============================================================================
 * ICM-20948 寄存器（仅列出本驱动用到的）
 * ========================================================================== */
// Bank 0
#define REG_WHO_AM_I          0x00   // 固定返回 0xEA
#define REG_USER_CTRL         0x03
#define REG_PWR_MGMT_1        0x06
#define REG_PWR_MGMT_2        0x07
#define REG_INT_PIN_CFG       0x0F
#define REG_ACCEL_XOUT_H      0x2D   // 连续: accel(6)+gyro(6)+temp(2)=14 字节
#define REG_BANK_SEL          0x7F

// Bank 2（量程 / 滤波 / 采样率）——地址已对照官方 DS-000189 修正
#define B2_GYRO_SMPLRT_DIV    0x00
#define B2_GYRO_CONFIG_1      0x01
#define B2_GYRO_CONFIG_2      0x02
#define B2_ACCEL_SMPLRT_DIV_2 0x11
#define B2_ACCEL_CONFIG       0x14

// USER_CTRL 位
#define BIT_I2C_MST_EN        0x20   // bit5（磁力计阶段才用）
#define BIT_I2C_MST_RST       0x01   // bit0

/* 量程与灵敏度
 * 陀螺 ±2000dps -> 16.384 LSB/(deg/s)
 * 加速度 ±8g     -> 4096 LSB/g
 */
#define GYRO_LSB_DPS          16.384f
#define ACC_LSB_G             4096.0f
#define ICM_WHOAMI_EXPECT     0xEA

/* ============================================================================
 * 控制环 / 解算参数
 * ========================================================================== */
#define LOOP_HZ               1000
#define LOOP_US               (1000000UL / LOOP_HZ)

#define Kp_MAHONY             1.2f   // 加速度计修正强度
#define Ki_MAHONY             0.05f  // 陀螺零偏积分修正
#define RAD2DEG               57.2957795f
#define DEG2RAD               0.01745329f

/* ============================================================================
 * SPI 对象与底层读写
 * ========================================================================== */
SPIClass icmSPI(FSPI);

static inline void cs_low()  { digitalWrite(ICM_CS, LOW);  }
static inline void cs_high() { digitalWrite(ICM_CS, HIGH); }

static uint8_t icm_read_reg(uint8_t reg) {
  cs_low();
  icmSPI.transfer(reg | 0x80);          // 读：最高位=1
  uint8_t v = icmSPI.transfer(0x00);
  cs_high();
  return v;
}

static void icm_write_reg(uint8_t reg, uint8_t val) {
  cs_low();
  icmSPI.transfer(reg & 0x7F);          // 写：最高位=0
  icmSPI.transfer(val);
  cs_high();
}

static void icm_read_burst(uint8_t reg, uint8_t *buf, uint8_t len) {
  cs_low();
  icmSPI.transfer(reg | 0x80);
  for (uint8_t i = 0; i < len; i++) buf[i] = icmSPI.transfer(0x00);
  cs_high();
}

static void icm_bank(uint8_t bank) {
  icm_write_reg(REG_BANK_SEL, bank << 4);   // bank 0/1/2/3 -> 0x00/0x10/0x20/0x30
}

/* ============================================================================
 * ICM-20948 初始化（寄存器级）
 * ========================================================================== */
static uint8_t icm_init() {
  // SPI 总线与 CS
  pinMode(ICM_CS, OUTPUT);
  cs_high();
  icmSPI.begin(ICM_SCK, ICM_MISO, ICM_MOSI, ICM_CS);
  icmSPI.beginTransaction(SPISettings(ICM_SPI_HZ, MSBFIRST, SPI_MODE3)); // ICM 空闲高、上升沿采样(模式3)
  cs_high();
  delay(10);

  icm_bank(0);
  icm_write_reg(REG_PWR_MGMT_1, 0x80);   // DEVICE_RESET
  delay(120);                            // 复位后必须等待
  icm_write_reg(REG_PWR_MGMT_1, 0x01);   // 唤醒(SLEEP=0)，CLKSEL=1 自动选最佳陀螺时钟
  delay(10);
  icm_write_reg(REG_PWR_MGMT_2, 0x00);   // 使能全部陀螺/加速度轴
  icm_write_reg(REG_INT_PIN_CFG, 0x00);

  // Bank2：量程 + 数字低通 + 采样率
  icm_bank(2);
  icm_write_reg(B2_GYRO_SMPLRT_DIV, 0x00);          // 陀螺 ODR = 1.1kHz/(0+1)
  // GYRO_CONFIG_1: DLPFCFG=3(bit5:3) + FS_SEL=3 即 ±2000dps(bit2:1) + FCHOICE=1(bit0)
  icm_write_reg(B2_GYRO_CONFIG_1, 0x1F);
  // GYRO_CONFIG_2: 默认 0，禁止误开 X/Y/Z 陀螺自测（曾写 0x18 到此处致 Y/Z 读数 1000+ dps）
  icm_write_reg(B2_GYRO_CONFIG_2, 0x00);
  icm_write_reg(B2_ACCEL_SMPLRT_DIV_2, 0x00);       // 加速度 ODR 最高
  // ACCEL_CONFIG: DLPFCFG=3(bit5:3) + FS_SEL=2 即 ±8g(bit2:1) + FCHOICE=1(bit0)
  icm_write_reg(B2_ACCEL_CONFIG, 0x1D);

  icm_bank(0);
  delay(50);

  return icm_read_reg(REG_WHO_AM_I);     // 期望 0xEA
}

/* 读取 accel+gyro（+temp 占位），转换为物理单位 */
static float acc_g[3], gyr_dps[3];
static void icm_read_motion() {
  uint8_t b[14];
  icm_read_burst(REG_ACCEL_XOUT_H, b, 14);   // 大端有符号

  int16_t ax = (int16_t)((b[0] << 8) | b[1]);
  int16_t ay = (int16_t)((b[2] << 8) | b[3]);
  int16_t az = (int16_t)((b[4] << 8) | b[5]);
  int16_t gx = (int16_t)((b[6] << 8) | b[7]);
  int16_t gy = (int16_t)((b[8] << 8) | b[9]);
  int16_t gz = (int16_t)((b[10] << 8) | b[11]);

  acc_g[0] = ax / ACC_LSB_G;  acc_g[1] = ay / ACC_LSB_G;  acc_g[2] = az / ACC_LSB_G;
  gyr_dps[0] = gx / GYRO_LSB_DPS;
  gyr_dps[1] = gy / GYRO_LSB_DPS;
  gyr_dps[2] = gz / GYRO_LSB_DPS;
}

/* ============================================================================
 * BMP280 气压计 —— I2C（GPIO8/9），寄存器级驱动
 * 已在第 2 步自检程序（esp32_sensor_check）中实测验证，此处原样并入。
 * 与 ICM-20948 的 FSPI(GPIO11/12/13/10) 无引脚冲突，可同程序运行。
 * ========================================================================== */
static uint8_t bmpAddr = 0;
static uint16_t bmp_dig_T1, bmp_dig_P1;
static int16_t  bmp_dig_T2, bmp_dig_T3;
static int16_t  bmp_dig_P2, bmp_dig_P3, bmp_dig_P4, bmp_dig_P5, bmp_dig_P6;
static int16_t  bmp_dig_P7, bmp_dig_P8, bmp_dig_P9;
static int32_t  bmp_t_fine = 0;
static float    bmp_tempC = 0, bmp_pressPa = 0, bmp_pressRef = 0;
static bool     bmp_ok = false;

static uint8_t bmpRead8(uint8_t reg) {
  Wire.beginTransmission(bmpAddr);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom((int)bmpAddr, 1);
  return Wire.available() ? Wire.read() : 0xFF;
}

static void bmpWrite8(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(bmpAddr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

static uint16_t bmpRead16LE(uint8_t reg) {
  return (uint16_t)bmpRead8(reg) | ((uint16_t)bmpRead8(reg + 1) << 8);
}

static bool bmpInit() {
  // 自动找 0x76 / 0x77（模块 SDO 接 GND 时为 0x76）
  for (uint8_t a = 0x76; a <= 0x77 && bmpAddr == 0; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) bmpAddr = a;
  }
  if (bmpAddr == 0) return false;
  if (bmpRead8(0xD0) != 0x58) return false;   // CHIP_ID 必须 0x58

  bmpWrite8(0xE0, 0xB6);   // 软复位
  delay(20);

  // 出厂校准系数（0x88 起 24 字节，小端）
  bmp_dig_T1 = bmpRead16LE(0x88);
  bmp_dig_T2 = (int16_t)bmpRead16LE(0x8A);
  bmp_dig_T3 = (int16_t)bmpRead16LE(0x8C);
  bmp_dig_P1 = bmpRead16LE(0x8E);
  bmp_dig_P2 = (int16_t)bmpRead16LE(0x90);
  bmp_dig_P3 = (int16_t)bmpRead16LE(0x92);
  bmp_dig_P4 = (int16_t)bmpRead16LE(0x94);
  bmp_dig_P5 = (int16_t)bmpRead16LE(0x96);
  bmp_dig_P6 = (int16_t)bmpRead16LE(0x98);
  bmp_dig_P7 = (int16_t)bmpRead16LE(0x9A);
  bmp_dig_P8 = (int16_t)bmpRead16LE(0x9C);
  bmp_dig_P9 = (int16_t)bmpRead16LE(0x9E);

  // config：IIR 滤波 x16，待机 0.5ms
  bmpWrite8(0xF5, 0x10);
  // ctrl_meas：温度 x2、气压 x16、normal 模式
  bmpWrite8(0xF4, 0x57);
  delay(20);
  return true;
}

// 温度补偿，返回 0.01℃ 单位（显示时 /100）
static int32_t bmpCompensateT(int32_t adcT) {
  int32_t v1 = ((((adcT >> 3) - ((int32_t)bmp_dig_T1 << 1))) * (int32_t)bmp_dig_T2) >> 11;
  int32_t v2 = (((((adcT >> 4) - (int32_t)bmp_dig_T1) *
                  ((adcT >> 4) - (int32_t)bmp_dig_T1)) >> 12) *
                (int32_t)bmp_dig_T3) >> 14;
  bmp_t_fine = v1 + v2;
  return (bmp_t_fine * 5 + 128) >> 8;
}

// 气压补偿，返回 Pa（int64 官方算法，结果 /256）
static uint32_t bmpCompensateP(int32_t adcP) {
  int64_t v1, v2, p;
  v1 = (int64_t)bmp_t_fine - 128000;
  v2 = v1 * v1 * (int64_t)bmp_dig_P6;
  v2 = v2 + ((v1 * (int64_t)bmp_dig_P5) << 17);
  v2 = v2 + (((int64_t)bmp_dig_P4) << 35);
  v1 = ((v1 * v1 * (int64_t)bmp_dig_P3) >> 8) + ((v1 * (int64_t)bmp_dig_P2) << 12);
  v1 = (((((int64_t)1) << 47) + v1) * (int64_t)bmp_dig_P1) >> 33;
  if (v1 == 0) return 0;
  p = 1048576 - adcP;
  p = (((p << 31) - v2) * 3125) / v1;
  v1 = (((int64_t)bmp_dig_P9) * (p >> 13) * (p >> 13)) >> 25;
  v2 = (((int64_t)bmp_dig_P8) * p) >> 19;
  p = ((p + v1 + v2) >> 8) + (((int64_t)bmp_dig_P7) << 4);
  return (uint32_t)(p / 256);
}

// 读温压；tempC 单位 ℃、pressPa 单位 Pa；成功返回 true
static bool bmpRead(float &tempC, float &pressPa) {
  uint8_t raw[6];
  Wire.beginTransmission(bmpAddr);
  Wire.write(0xF7);
  if (Wire.endTransmission() != 0) return false;
  Wire.requestFrom((int)bmpAddr, 6);
  if (Wire.available() != 6) return false;
  for (uint8_t i = 0; i < 6; i++) raw[i] = Wire.read();

  int32_t adcP = ((int32_t)raw[0] << 12) | ((int32_t)raw[1] << 4) | (raw[2] >> 4);
  int32_t adcT = ((int32_t)raw[3] << 12) | ((int32_t)raw[4] << 4) | (raw[5] >> 4);

  int32_t t10 = bmpCompensateT(adcT);
  uint32_t p  = bmpCompensateP(adcP);
  if (p == 0) return false;
  tempC  = t10 / 100.0f;   // 0.01℃ -> ℃（曾误除 10 显示 275℃）
  pressPa = (float)p;
  return true;
}

// 上电后 delay(500) 再取 10 次平均作气压基准（避免首次转换坏值，曾致高度虚偏 -2474m）
static void bmpSetPressRef() {
  delay(500);
  float t, p, psum = 0;
  int n = 0;
  for (int i = 0; i < 10; i++) {
    if (bmpRead(t, p)) { psum += p; n++; }
    delay(100);
  }
  if (n > 0) bmp_pressRef = psum / n;
}

/* ============================================================================
 * 姿态四元数 / 欧拉角（Mahony）
 * ========================================================================== */
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
static float eInt[3] = {0, 0, 0};
static float roll_rad, pitch_rad, yaw_rad;

/* 由重力方向给四元数赋初值（绕 Y 修正 roll、绕 X 修正 pitch） */
static void q_init_from_accel(float ax, float ay, float az) {
  (void)az;
  float pitch = asinf(constrain(ax, -1.0f, 1.0f));   // x 方向重力分量 -> pitch
  float roll  = asinf(constrain(-ay, -1.0f, 1.0f));  // y 方向重力分量 -> roll
  float cr = cosf(roll * 0.5f), sr = sinf(roll * 0.5f);
  float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
  q0 = cr*cp; q1 = sr*cp; q2 = cr*sp; q3 = -sr*sp;
  eInt[0] = eInt[1] = eInt[2] = 0.0f;
}

/* ============================================================================
 * 校准：静止陀螺零偏；加速度取平均作为初始重力方向
 * ========================================================================== */
static float gyr_bias[3] = {0, 0, 0};

static void imu_calibrate(uint16_t samples = 500) {
  float gs[3] = {0, 0, 0}, as[3] = {0, 0, 0};
  for (uint16_t i = 0; i < samples; i++) {
    icm_read_motion();
    gs[0] += gyr_dps[0]; gs[1] += gyr_dps[1]; gs[2] += gyr_dps[2];
    as[0] += acc_g[0];   as[1] += acc_g[1];   as[2] += acc_g[2];
    delayMicroseconds(1000);
  }
  for (int i = 0; i < 3; i++) {
    gyr_bias[i] = gs[i] / samples;
  }
  // 用平均重力方向初始化四元数（板子水平时 az≈1）
  float ax = as[0] / samples, ay = as[1] / samples, az = as[2] / samples;
  float n = sqrtf(ax*ax + ay*ay + az*az);
  if (n > 1e-6f) { ax /= n; ay /= n; az /= n; }
  q_init_from_accel(ax, ay, az);
}

/* ============================================================================
 * Mahony 6 轴姿态（四元数）
 * ========================================================================== */
/* ax,ay,az: 归一化加速度（机体系）；gx,gy,gz: 去零偏角速度(rad/s) */
static void mahony_update6(float ax, float ay, float az,
                           float gx, float gy, float gz, float dt) {
  // 1) 当前四元数估计的重力方向（机体系）
  float vx = 2.0f*(q1*q3 - q0*q2);
  float vy = 2.0f*(q0*q1 + q2*q3);
  float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

  // 2) 测量重力与估计重力的叉积 = 误差
  float ex = ay*vz - az*vy;
  float ey = az*vx - ax*vz;
  float ez = ax*vy - ay*vx;

  // 3) PI 补偿
  eInt[0] += ex * Ki_MAHONY * dt;
  eInt[1] += ey * Ki_MAHONY * dt;
  eInt[2] += ez * Ki_MAHONY * dt;
  gx += Kp_MAHONY*ex + eInt[0];
  gy += Kp_MAHONY*ey + eInt[1];
  gz += Kp_MAHONY*ez + eInt[2];

  // 4) 四元数微分（一阶积分）
  float dq0 = 0.5f*(-q1*gx - q2*gy - q3*gz);
  float dq1 = 0.5f*( q0*gx + q2*gz - q3*gy);
  float dq2 = 0.5f*( q0*gy - q1*gz + q3*gx);
  float dq3 = 0.5f*( q0*gz + q1*gy - q2*gx);
  q0 += dq0*dt; q1 += dq1*dt; q2 += dq2*dt; q3 += dq3*dt;

  // 5) 归一化
  float n = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  if (n > 1e-6f) { q0/=n; q1/=n; q2/=n; q3/=n; }
}

static void quat_to_euler() {
  roll_rad  = atan2f(2.0f*(q0*q1 + q2*q3), 1.0f - 2.0f*(q1*q1 + q2*q2));
  pitch_rad = asinf(constrain(2.0f*(q0*q2 - q3*q1), -1.0f, 1.0f));
  yaw_rad   = atan2f(2.0f*(q0*q3 + q1*q2), 1.0f - 2.0f*(q2*q2 + q3*q3));
}

/* ============================================================================
 * setup / loop
 * ========================================================================== */
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println("[M1] ESP32-S3 自研飞控 / ICM-20948 SPI");

  uint8_t who = icm_init();
  Serial.print("  WHO_AM_I = 0x");
  Serial.print(who, HEX);
  if (who == ICM_WHOAMI_EXPECT) {
    Serial.println("  (OK = EA)");
  } else {
    Serial.println("  异常! 应为 EA。检查: VCC/GND/SCK/MISO/MOSI/CS、模块是否切到 SPI");
    while (1) { delay(1000); }     // 自检失败，停机等待，避免带错继续
  }

  // ---- BMP280 气压计（I2C GPIO8/9；失败不阻塞 IMU，仅气压功能禁用）----
  Wire.begin(BMP_SDA, BMP_SCL);
  if (bmpInit()) {
    bmp_ok = true;
    bmpSetPressRef();
    Serial.printf("  BMP280: 在线 0x%02X | 基准气压 %.1f hPa\n",
                  bmpAddr, bmp_pressRef / 100.0f);
  } else {
    Serial.println("  BMP280: 未找到！检查 VCC=3V3 / SDA=8 / SCL=9 / CSB 接 3V3（不影响 IMU）");
  }

  Serial.println("[校准] 保持板子水平静止 2 秒...");
  delay(500);
  imu_calibrate();
  Serial.print("  陀螺零偏(dps): ");
  Serial.print(gyr_bias[0], 3); Serial.print(", ");
  Serial.print(gyr_bias[1], 3); Serial.print(", ");
  Serial.println(gyr_bias[2], 3);
  Serial.println("[运行] 输出 roll,pitch,yaw(度)，每秒 10 次");
}

void loop() {
  static uint32_t tLast = 0;
  uint32_t now = micros();
  float dt = (tLast == 0) ? (1.0f/LOOP_HZ) : (float)(now - tLast) / 1e6f;
  tLast = now;
  if (dt <= 0.0f || dt > 0.05f) dt = 1.0f/LOOP_HZ;   // 首帧/异常间隔保护

  icm_read_motion();

  // BMP280：每 500ms 读一次（打印节拍 1s，两读之间用最新值）
  static uint32_t tBmp = 0;
  if (bmp_ok && now - tBmp >= 500000UL) {
    tBmp = now;
    bmpRead(bmp_tempC, bmp_pressPa);
  }

  // 去零偏，转 rad/s
  float gx = (gyr_dps[0] - gyr_bias[0]) * DEG2RAD;
  float gy = (gyr_dps[1] - gyr_bias[1]) * DEG2RAD;
  float gz = (gyr_dps[2] - gyr_bias[2]) * DEG2RAD;

  // 归一化加速度
  float an = sqrtf(acc_g[0]*acc_g[0] + acc_g[1]*acc_g[1] + acc_g[2]*acc_g[2]);
  float ax = an > 1e-6f ? acc_g[0]/an : 0.0f;
  float ay = an > 1e-6f ? acc_g[1]/an : 0.0f;
  float az = an > 1e-6f ? acc_g[2]/an : 0.0f;

  mahony_update6(ax, ay, az, gx, gy, gz, dt);
  quat_to_euler();

  // 10Hz 打印，避免刷屏
  static uint32_t tPrint = 0;
  if (now - tPrint >= 100000UL) {
    tPrint = now;
    Serial.print(roll_rad * RAD2DEG, 2);  Serial.print(",");
    Serial.print(pitch_rad * RAD2DEG, 2); Serial.print(",");
    Serial.println(yaw_rad * RAD2DEG, 2);
    Serial.printf("  [RAW] acc %.3f %.3f %.3f g | gyr %.2f %.2f %.2f dps\n",
                  acc_g[0], acc_g[1], acc_g[2],
                  gyr_dps[0], gyr_dps[1], gyr_dps[2]);

    if (bmp_ok && bmp_pressPa > 0) {
      float alt = 44330.0f * (1.0f - powf(bmp_pressPa / bmp_pressRef, 1.0f / 5.255f));
      Serial.printf("  [BMP] %.1f C | %.1f hPa | 相对高度 %+.1f m\n",
                    bmp_tempC, bmp_pressPa / 100.0f, alt);
    } else {
      Serial.println("  [BMP] 无数据");
    }
  }

  // 严格 1kHz 节拍
  int32_t remain = (int32_t)LOOP_US - (int32_t)(micros() - now);
  if (remain > 0) delayMicroseconds(remain);
}

/* ============================================================================
 * 下一阶段 TODO（M1 第二步 -> M4）
 * ----------------------------------------------------------------------------
 * 1) 磁力计 AK09916（9 轴航向）：
 *    - ICM 内部经 AUX I2C 挂 AK09916(I2C 地址 0x0C)
 *    - bank0: 复位并使能 I2C master(USER_CTRL bit5)，配 I2C_MST_CTRL(0x24)
 *    - SLV0 写 CNTL2(0x31)=0x06 进连续 100Hz；SLV0 从 HXL(0x11) 读 8 字节
 *    - 数据在 EXT_SENS_DATA(0x3B 起)，灵敏度 0.15uT/LSB，小端
 *    - 9 轴 Mahony：把磁场做倾斜补偿后再加一路叉积误差（航向不漂移）
 *
 * 2) BMP280（I2C 0x76/0x77）：读温压算相对高度 —— 驱动已并入本文件（见上文），
 *    定高内环待 M5 再实现。
 *
 * 3) SBUS：UART 100000 8E2，反相信号需硬件反相器或特定反相 UART；
 *    解析 18 通道，取油门/横滚/俯仰/方向 + 开关（解锁/飞行模式）。
 *
 * 4) 电调：4 路 LEDC 50~500Hz（或 DShot），先拆桨做油门行程校准与转向核对，
 *    再上串级 PID（角速度内环 -> 角度外环），M4 装桨必须测试架系留。
 * ========================================================================== */
