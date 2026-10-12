/*
 * ============================================================================
 * ESP32-S3 自研四旋翼 —— M6 主飞控（自稳模式 v1）
 * ----------------------------------------------------------------------------
 * 整合：ICM-20948 姿态解算 + SBUS 遥控 + 串级 PID + X 型混控 + 解锁/失控保护
 *
 * ⚠️ 安全铁律（每次通电前必做）：
 *   1. 第一次烧录后，先拆桨！桌面空载测试，不要装桨！
 *   2. 先地面试转（系留/测试架），确认转向正确、PID 不抖，再装桨
 *   3. 接电池时拔掉 USB；USB 调试时不要接电池
 *   4. 失控保护：SBUS 丢信号 1 秒自动上锁停转
 *
 * 接线（全部已验证）：
 *   ICM-20948 SPI：SCK=12 MISO=13 MOSI=11 CS=10（3V3）
 *   BMP280 I2C  ：SDA=8  SCL=9
 *   SBUS 接收机  ：RX=18（软件反相，无需反相器）
 *   电机 PWM     ：M1=4  M2=5  M3=6  M4=7（50Hz，1000-2000µs）
 *   电调 BEC 红线全挑出！全机 5V = 好盈 UBEC
 *
 * 遥控器通道（FlySky 默认）：
 *   CH1 = 横滚（左右）
 *   CH2 = 俯仰（前后）
 *   CH3 = 油门（上下）
 *   CH4 = 偏航（左右转向）
 *
 * 解锁/上锁：
 *   油门最低 + 偏航右打 保持 2 秒 = 解锁（电机开始怠速）
 *   油门最低 + 偏航左打 保持 2 秒 = 上锁（电机停转）
 *
 * 调试：串口 115200，每秒打印一次姿态/遥控/油门状态
 * ============================================================================
 */

#include <SPI.h>
#include <math.h>
#include <Wire.h>

/* ============================================================================
 * 引脚定义
 * ========================================================================== */
#define ICM_SCK       12
#define ICM_MISO      13
#define ICM_MOSI      11
#define ICM_CS        10
#define ICM_SPI_HZ    8000000UL

#define BMP_SDA       8
#define BMP_SCL       9

#define SBUS_RX_PIN   18
#define SBUS_BAUD     100000
#define SBUS_FRAME_LEN 25

#define M1_PIN        4
#define M2_PIN        5
#define M3_PIN        6
#define M4_PIN        7

#define MIN_US        1000
#define MAX_US        2000
#define PWM_RES_BITS  14
#define PERIOD_US     20000

/* ============================================================================
 * ICM-20948 寄存器（复用 M1 已验证驱动）
 * ========================================================================== */
#define REG_WHO_AM_I          0x00
#define REG_USER_CTRL         0x03
#define REG_PWR_MGMT_1        0x06
#define REG_PWR_MGMT_2        0x07
#define REG_INT_PIN_CFG       0x0F
#define REG_ACCEL_XOUT_H      0x2D
#define REG_BANK_SEL          0x7F

#define B2_GYRO_SMPLRT_DIV    0x00
#define B2_GYRO_CONFIG_1      0x01
#define B2_GYRO_CONFIG_2      0x02
#define B2_ACCEL_SMPLRT_DIV_2 0x11
#define B2_ACCEL_CONFIG       0x14

#define GYRO_LSB_DPS          16.384f
#define ACC_LSB_G             4096.0f
#define ICM_WHOAMI_EXPECT     0xEA

/* ============================================================================
 * 控制环参数
 * ========================================================================== */
#define LOOP_HZ               1000
#define LOOP_US               (1000000UL / LOOP_HZ)

#define Kp_MAHONY             1.2f
#define Ki_MAHONY             0.05f
#define RAD2DEG               57.2957795f
#define DEG2RAD               0.01745329f

/* ---- PID 参数（先给保守默认值，飞起来再微调）---- */
// 角度环（外环）：输入角度误差(度)，输出目标角速度(度/秒)
#define ANGLE_P_ROLL          4.0f
#define ANGLE_P_PITCH         4.0f
#define ANGLE_P_YAW           2.0f

// 角速度环（内环）：输入角速度误差(度/秒)，输出电机混控量
#define RATE_P_ROLL           8.0f
#define RATE_I_ROLL           0.1f
#define RATE_D_ROLL           0.05f
#define RATE_P_PITCH          8.0f
#define RATE_I_PITCH          0.1f
#define RATE_D_PITCH          0.05f
#define RATE_P_YAW            6.0f
#define RATE_I_YAW            0.05f
#define RATE_D_YAW            0.0f

#define RATE_I_LIMIT          50.0f   // 积分限幅，防饱和

/* ---- 遥控通道映射 ---- */
#define CH_ROLL               0    // CH1
#define CH_PITCH              1    // CH2
#define CH_THROTTLE           2    // CH3
#define CH_YAW                3    // CH4

#define RC_MIN                1000
#define RC_MID                1500
#define RC_MAX                2000
#define RC_DEADBAND           20     // 中位死区(µs)，防漂移

/* ---- 油门映射 ---- */
#define THROTTLE_IDLE         1100   // 解锁后怠速油门
#define THROTTLE_MIN          1000
#define THROTTLE_MAX          1900   // 留 100µs 余量，防止顶满

/* ============================================================================
 * ICM SPI 底层
 * ========================================================================== */
SPIClass icmSPI(FSPI);
static inline void cs_low()  { digitalWrite(ICM_CS, LOW);  }
static inline void cs_high() { digitalWrite(ICM_CS, HIGH); }

static uint8_t icm_read_reg(uint8_t reg) {
  cs_low();
  icmSPI.transfer(reg | 0x80);
  uint8_t v = icmSPI.transfer(0x00);
  cs_high();
  return v;
}
static void icm_write_reg(uint8_t reg, uint8_t val) {
  cs_low();
  icmSPI.transfer(reg & 0x7F);
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
  icm_write_reg(REG_BANK_SEL, bank << 4);
}

static float acc_g[3], gyr_dps[3];
static void icm_read_motion() {
  uint8_t b[14];
  icm_read_burst(REG_ACCEL_XOUT_H, b, 14);
  int16_t ax = (int16_t)((b[0] << 8) | b[1]);
  int16_t ay = (int16_t)((b[2] << 8) | b[3]);
  int16_t az = (int16_t)((b[4] << 8) | b[5]);
  int16_t gx = (int16_t)((b[6] << 8) | b[7]);
  int16_t gy = (int16_t)((b[8] << 8) | b[9]);
  int16_t gz = (int16_t)((b[10] << 8) | b[11]);
  acc_g[0] = ax / ACC_LSB_G; acc_g[1] = ay / ACC_LSB_G; acc_g[2] = az / ACC_LSB_G;
  gyr_dps[0] = gx / GYRO_LSB_DPS; gyr_dps[1] = gy / GYRO_LSB_DPS; gyr_dps[2] = gz / GYRO_LSB_DPS;
}

static uint8_t icm_init() {
  pinMode(ICM_CS, OUTPUT);
  cs_high();
  icmSPI.begin(ICM_SCK, ICM_MISO, ICM_MOSI, ICM_CS);
  icmSPI.beginTransaction(SPISettings(ICM_SPI_HZ, MSBFIRST, SPI_MODE3));
  cs_high();
  delay(10);
  icm_bank(0);
  icm_write_reg(REG_PWR_MGMT_1, 0x80);
  delay(120);
  icm_write_reg(REG_PWR_MGMT_1, 0x01);
  delay(10);
  icm_write_reg(REG_PWR_MGMT_2, 0x00);
  icm_write_reg(REG_INT_PIN_CFG, 0x00);
  icm_bank(2);
  icm_write_reg(B2_GYRO_SMPLRT_DIV, 0x00);
  icm_write_reg(B2_GYRO_CONFIG_1, 0x1F);
  icm_write_reg(B2_GYRO_CONFIG_2, 0x00);
  icm_write_reg(B2_ACCEL_SMPLRT_DIV_2, 0x00);
  icm_write_reg(B2_ACCEL_CONFIG, 0x1D);
  icm_bank(0);
  delay(50);
  return icm_read_reg(REG_WHO_AM_I);
}

/* ============================================================================
 * 姿态解算（Mahony 6 轴）
 * ========================================================================== */
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
static float eInt[3] = {0, 0, 0};
static float roll_deg, pitch_deg, yaw_deg;
static float gyr_bias[3] = {0, 0, 0};

static void q_init_from_accel(float ax, float ay, float az) {
  (void)az;
  float pitch = asinf(constrain(ax, -1.0f, 1.0f));
  float roll  = asinf(constrain(-ay, -1.0f, 1.0f));
  float cr = cosf(roll * 0.5f), sr = sinf(roll * 0.5f);
  float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
  q0 = cr*cp; q1 = sr*cp; q2 = cr*sp; q3 = -sr*sp;
  eInt[0] = eInt[1] = eInt[2] = 0.0f;
}

static void imu_calibrate(uint16_t samples = 500) {
  float gs[3] = {0, 0, 0}, as[3] = {0, 0, 0};
  for (uint16_t i = 0; i < samples; i++) {
    icm_read_motion();
    gs[0] += gyr_dps[0]; gs[1] += gyr_dps[1]; gs[2] += gyr_dps[2];
    as[0] += acc_g[0];   as[1] += acc_g[1];   as[2] += acc_g[2];
    delayMicroseconds(1000);
  }
  for (int i = 0; i < 3; i++) gyr_bias[i] = gs[i] / samples;
  float ax = as[0] / samples, ay = as[1] / samples, az = as[2] / samples;
  float n = sqrtf(ax*ax + ay*ay + az*az);
  if (n > 1e-6f) { ax /= n; ay /= n; az /= n; }
  q_init_from_accel(ax, ay, az);
}

static void mahony_update6(float ax, float ay, float az, float gx, float gy, float gz, float dt) {
  float vx = 2.0f*(q1*q3 - q0*q2);
  float vy = 2.0f*(q0*q1 + q2*q3);
  float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
  float ex = ay*vz - az*vy;
  float ey = az*vx - ax*vz;
  float ez = ax*vy - ay*vx;
  eInt[0] += ex * Ki_MAHONY * dt;
  eInt[1] += ey * Ki_MAHONY * dt;
  eInt[2] += ez * Ki_MAHONY * dt;
  gx += Kp_MAHONY*ex + eInt[0];
  gy += Kp_MAHONY*ey + eInt[1];
  gz += Kp_MAHONY*ez + eInt[2];
  float dq0 = 0.5f*(-q1*gx - q2*gy - q3*gz);
  float dq1 = 0.5f*( q0*gx + q2*gz - q3*gy);
  float dq2 = 0.5f*( q0*gy - q1*gz + q3*gx);
  float dq3 = 0.5f*( q0*gz + q1*gy - q2*gx);
  q0 += dq0*dt; q1 += dq1*dt; q2 += dq2*dt; q3 += dq3*dt;
  float n = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  if (n > 1e-6f) { q0/=n; q1/=n; q2/=n; q3/=n; }
}

static void quat_to_euler_deg() {
  float roll_rad  = atan2f(2.0f*(q0*q1 + q2*q3), 1.0f - 2.0f*(q1*q1 + q2*q2));
  float pitch_rad = asinf(constrain(2.0f*(q0*q2 - q3*q1), -1.0f, 1.0f));
  float yaw_rad   = atan2f(2.0f*(q0*q3 + q1*q2), 1.0f - 2.0f*(q2*q2 + q3*q3));
  roll_deg  = roll_rad * RAD2DEG;
  pitch_deg = pitch_rad * RAD2DEG;
  yaw_deg   = yaw_rad * RAD2DEG;
}

/* ============================================================================
 * SBUS 遥控器（复用 M3 已验证）
 * ========================================================================== */
static uint16_t rc_ch[16] = {1500,1500,1000,1500,1500,1500};
static bool rc_failsafe = false;
static uint32_t rc_last_frame_ms = 0;

bool sbus_parse(const uint8_t *b) {
  if (b[0] != 0x0F) return false;
  rc_ch[0]  = ((b[1]       | b[2] << 8)                    & 0x07FF);
  rc_ch[1]  = ((b[2] >> 3  | b[3] << 5)                    & 0x07FF);
  rc_ch[2]  = ((b[3] >> 6  | b[4] << 2 | b[5] << 10)       & 0x07FF);
  rc_ch[3]  = ((b[5] >> 1  | b[6] << 7)                    & 0x07FF);
  rc_ch[4]  = ((b[6] >> 4  | b[7] << 4)                    & 0x07FF);
  rc_ch[5]  = ((b[7] >> 7  | b[8] << 1 | b[9] << 9)        & 0x07FF);
  rc_failsafe  = (b[23] & 0x08) != 0;
  return true;
}

/* ============================================================================
 * 电机 PWM 输出（复用 M4 已验证）
 * ========================================================================== */
static const uint8_t MOTOR_PINS[4] = {M1_PIN, M2_PIN, M3_PIN, M4_PIN};
static uint16_t motor_us[4] = {MIN_US, MIN_US, MIN_US, MIN_US};

void set_motor_us(uint8_t idx, uint16_t us) {
  if (us < MIN_US) us = MIN_US;
  if (us > MAX_US) us = MAX_US;
  motor_us[idx] = us;
  uint32_t duty = (uint32_t)us * (1UL << PWM_RES_BITS) / PERIOD_US;
  analogWrite(MOTOR_PINS[idx], duty);
}
void set_all_motors(uint16_t us) {
  for (int i = 0; i < 4; i++) set_motor_us(i, us);
}

/* ============================================================================
 * PID 控制器（串级：角度外环 -> 角速度内环）
 * ========================================================================== */
struct PID {
  float kp, ki, kd;
  float i_term, prev_err;
};

static void pid_init(PID *p, float kp, float ki, float kd) {
  p->kp = kp; p->ki = ki; p->kd = kd;
  p->i_term = 0; p->prev_err = 0;
}

static float pid_calc(PID *p, float err, float dt) {
  p->i_term += err * dt;
  if (p->i_term > RATE_I_LIMIT) p->i_term = RATE_I_LIMIT;
  if (p->i_term < -RATE_I_LIMIT) p->i_term = -RATE_I_LIMIT;
  float deriv = (err - p->prev_err) / dt;
  p->prev_err = err;
  return p->kp * err + p->ki * p->i_term + p->kd * deriv;
}

static PID pid_roll_rate, pid_pitch_rate, pid_yaw_rate;

/* ============================================================================
 * 飞控状态
 * ========================================================================== */
static bool armed = false;           // 是否解锁
static uint32_t arm_start_ms = 0;   // 解锁动作开始时间
static bool arm_switching = false;   // 正在执行解锁/上锁动作

/* ============================================================================
 * setup
 * ========================================================================== */
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println("========================================");
  Serial.println("  ESP32-S3 自研四旋翼 / M6 主飞控 v1");
  Serial.println("========================================");

  // ---- ICM 自检 ----
  uint8_t who = icm_init();
  Serial.printf("[ICM] WHO_AM_I = 0x%02X", who);
  if (who == ICM_WHOAMI_EXPECT) Serial.println("  OK");
  else { Serial.println("  异常！停机"); while(1) delay(1000); }

  // ---- 陀螺校准 ----
  Serial.println("[校准] 保持水平静止 2 秒...");
  delay(500);
  imu_calibrate();
  Serial.printf("  零偏: %.3f %.3f %.3f dps\n", gyr_bias[0], gyr_bias[1], gyr_bias[2]);

  // ---- 电机 PWM 初始化 ----
  for (int i = 0; i < 4; i++) {
    pinMode(MOTOR_PINS[i], OUTPUT);
    analogWriteResolution(MOTOR_PINS[i], PWM_RES_BITS);
    analogWriteFrequency(MOTOR_PINS[i], 50);
  }
  set_all_motors(MIN_US);
  Serial.println("[MOTOR] 全部油门 1000（停转）");

  // ---- SBUS 初始化 ----
  Serial2.begin(SBUS_BAUD, SERIAL_8E2, SBUS_RX_PIN, -1, true);
  Serial.println("[SBUS] GPIO18 软件反相");

  // ---- PID 初始化 ----
  pid_init(&pid_roll_rate,  RATE_P_ROLL,  RATE_I_ROLL,  RATE_D_ROLL);
  pid_init(&pid_pitch_rate, RATE_P_PITCH, RATE_I_PITCH, RATE_D_PITCH);
  pid_init(&pid_yaw_rate,   RATE_P_YAW,   RATE_I_YAW,   RATE_D_YAW);

  Serial.println();
  Serial.println("=== 就绪 ===");
  Serial.println("解锁：油门最低 + 偏航右打 2 秒");
  Serial.println("上锁：油门最低 + 偏航左打 2 秒");
  Serial.println("⚠ 首次测试务必拆桨！");
}

/* ============================================================================
 * loop（1kHz 控制环）
 * ========================================================================== */
void loop() {
  static uint32_t tLast = 0;
  uint32_t now = micros();
  float dt = (tLast == 0) ? (1.0f/LOOP_HZ) : (float)(now - tLast) / 1e6f;
  tLast = now;
  if (dt <= 0.0f || dt > 0.05f) dt = 1.0f/LOOP_HZ;

  // ---- 1. 读 IMU ----
  icm_read_motion();
  float gx = (gyr_dps[0] - gyr_bias[0]) * DEG2RAD;
  float gy = (gyr_dps[1] - gyr_bias[1]) * DEG2RAD;
  float gz = (gyr_dps[2] - gyr_bias[2]) * DEG2RAD;
  float an = sqrtf(acc_g[0]*acc_g[0] + acc_g[1]*acc_g[1] + acc_g[2]*acc_g[2]);
  float ax = an > 1e-6f ? acc_g[0]/an : 0.0f;
  float ay = an > 1e-6f ? acc_g[1]/an : 0.0f;
  float az = an > 1e-6f ? acc_g[2]/an : 0.0f;
  mahony_update6(ax, ay, az, gx, gy, gz, dt);
  quat_to_euler_deg();

  // ---- 2. 读 SBUS 遥控 ----
  static uint8_t sbus_buf[SBUS_FRAME_LEN];
  static uint8_t sbus_idx = 0;
  while (Serial2.available()) {
    uint8_t b = Serial2.read();
    if (sbus_idx == 0 && b != 0x0F) continue;
    sbus_buf[sbus_idx++] = b;
    if (sbus_idx >= SBUS_FRAME_LEN) {
      sbus_idx = 0;
      if (sbus_parse(sbus_buf)) rc_last_frame_ms = millis();
    }
  }

  // ---- 3. 失控保护：1 秒没收到帧 = 强制上锁 ----
  bool rc_lost = (millis() - rc_last_frame_ms > 1000) || rc_failsafe;
  if (rc_lost && armed) {
    armed = false;
    arm_switching = false;
    set_all_motors(MIN_US);
    Serial.println("[!] 失控！强制上锁");
  }

  // ---- 4. 解锁/上锁逻辑 ----
  float rc_thr = rc_ch[CH_THROTTLE];
  float rc_yaw = rc_ch[CH_YAW];
  bool thr_low = (rc_thr < 1050);

  if (thr_low && !rc_lost) {
    if (rc_yaw > 1800) {  // 偏航右打
      if (!arm_switching) { arm_switching = true; arm_start_ms = millis(); }
      if (!armed && millis() - arm_start_ms > 2000) {
        armed = true;
        arm_switching = false;
        Serial.println(">>> 已解锁，电机怠速 <<<");
      }
    } else if (rc_yaw < 1200) {  // 偏航左打
      if (!arm_switching) { arm_switching = true; arm_start_ms = millis(); }
      if (armed && millis() - arm_start_ms > 2000) {
        armed = false;
        arm_switching = false;
        set_all_motors(MIN_US);
        Serial.println(">>> 已上锁，电机停转 <<<");
      }
    } else {
      arm_switching = false;
    }
  } else {
    arm_switching = false;
  }

  // ---- 5. 遥控通道映射 + 死区 ----
  float rc_roll  = rc_ch[CH_ROLL];
  float rc_pitch = rc_ch[CH_PITCH];
  // 归一化到 -1 ~ +1
  float roll_cmd  = (fabs(rc_roll  - RC_MID)  < RC_DEADBAND) ? 0 : (rc_roll  - RC_MID)  / (RC_MAX - RC_MID);
  float pitch_cmd = (fabs(rc_pitch - RC_MID)  < RC_DEADBAND) ? 0 : (rc_pitch - RC_MID) / (RC_MAX - RC_MID);
  float yaw_cmd   = (fabs(rc_yaw   - RC_MID)  < RC_DEADBAND) ? 0 : (rc_yaw   - RC_MID)   / (RC_MAX - RC_MID);

  // 油门映射：1000~2000 -> 1000~1900
  float thr_out = map(rc_thr, RC_MIN, RC_MAX, THROTTLE_MIN, THROTTLE_MAX);

  // ---- 6. 串级 PID 计算 ----
  if (armed && !rc_lost) {
    // 外环：角度误差 -> 目标角速度
    float roll_target_rate  = roll_cmd  * 30.0f;   // 最大 30 度/秒
    float pitch_target_rate  = pitch_cmd * 30.0f;
    float yaw_target_rate    = yaw_cmd   * 100.0f;  // 偏航允许更快

    // 注意：自稳模式下，目标角度 = 摇杆量 * 最大倾斜角
    // 这里简化为：外环直接算角度误差
    float roll_angle_err  = roll_cmd  * 25.0f - roll_deg;   // 最大倾斜 25 度
    float pitch_angle_err = pitch_cmd * 25.0f - pitch_deg;
    // 偏航没有角度环（靠手动），直接用摇杆量当目标角速度

    // 内环：角速度 PID
    float roll_rate_err  = (roll_angle_err  * ANGLE_P_ROLL)  - gyr_dps[0];
    float pitch_rate_err = (pitch_angle_err * ANGLE_P_PITCH) - gyr_dps[1];
    float yaw_rate_err   = yaw_target_rate - gyr_dps[2];

    float roll_out  = pid_calc(&pid_roll_rate,  roll_rate_err,  dt);
    float pitch_out = pid_calc(&pid_pitch_rate, pitch_rate_err, dt);
    float yaw_out   = pid_calc(&pid_yaw_rate,   yaw_rate_err,   dt);

    // ---- 7. X 型混控 ----
    // M1=右前  M2=左前  M3=右后  M4=左后
    float m1 = thr_out + roll_out + pitch_out - yaw_out;
    float m2 = thr_out - roll_out + pitch_out + yaw_out;
    float m3 = thr_out + roll_out - pitch_out + yaw_out;
    float m4 = thr_out - roll_out - pitch_out - yaw_out;

    // 油门最低时给怠速
    if (thr_out < THROTTLE_IDLE) thr_out = THROTTLE_IDLE;
    m1 = thr_out + (m1 - thr_out) * 1.0f;  // PID 输出叠加在油门上
    m2 = thr_out + (m2 - thr_out) * 1.0f;
    m3 = thr_out + (m3 - thr_out) * 1.0f;
    m4 = thr_out + (m4 - thr_out) * 1.0f;

    set_motor_us(0, (uint16_t)m1);
    set_motor_us(1, (uint16_t)m2);
    set_motor_us(2, (uint16_t)m3);
    set_motor_us(3, (uint16_t)m4);
  } else {
    set_all_motors(MIN_US);
    pid_roll_rate.i_term = 0;
    pid_pitch_rate.i_term = 0;
    pid_yaw_rate.i_term = 0;
  }

  // ---- 8. 10Hz 状态打印 ----
  static uint32_t tPrint = 0;
  if (now - tPrint >= 100000UL) {
    tPrint = now;
    Serial.printf("%s | R:%6.1f P:%6.1f Y:%6.1f | RC: %4u %4u %4u %4u | M: %4u %4u %4u %4u\n",
                  armed ? "ARMED " : "DISARM",
                  roll_deg, pitch_deg, yaw_deg,
                  rc_ch[0], rc_ch[1], rc_ch[2], rc_ch[3],
                  motor_us[0], motor_us[1], motor_us[2], motor_us[3]);
  }

  // 严格 1kHz 节拍
  int32_t remain = (int32_t)LOOP_US - (int32_t)(micros() - now);
  if (remain > 0) delayMicroseconds(remain);
}

/* ============================================================================
 * 调试提示：
 * 1. 第一次烧录后，先不接电池，只 USB 供电，看串口状态正常
 * 2. 接电池前务必拆桨！
 * 3. 解锁后电机应微微怠速转（1100µs），推油门才加速
 * 4. 如果飞机往一边倒，调 PID 的 P 值或检查电机转向
 * 5. 电机转向：M1/M3 正桨，M2/M4 反桨（不对就换线序）
 * ========================================================================== */
