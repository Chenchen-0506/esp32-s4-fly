/*
 * ============================================================================
 * ESP32-S3 自研四旋翼 —— M6 主飞控（自稳模式 v1，模块化版）
 * ----------------------------------------------------------------------------
 * 整合：ICM-20948 姿态解算 + SBUS 遥控 + 串级 PID + X 型混控 + 解锁/失控保护
 *
 * 【功能模块化】本工程 src/ 目录按功能拆分，一个功能一个 .h/.cpp：
 *   src/ICM20948.h/.cpp   九轴 IMU 驱动 + Mahony 姿态 + 零偏校准
 *   src/SBUS.h/.cpp       SBUS 遥控解析（GPIO18 软件反相）
 *   src/PID.h/.cpp        串级 PID 控制器（含内环参数）
 *   src/Motors.h/.cpp     4 路电调 PWM 输出
 * 主文件只保留：引脚/常量的应用层定义、setup、1kHz 控制环、解锁/上锁
 * 状态机、X 型混控、状态打印。改功能只动对应模块，方便回档与开源。
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
 *   CH1 = 横滚（左右）  CH2 = 俯仰（前后）
 *   CH3 = 油门（上下）  CH4 = 偏航（左右转向）
 *
 * 解锁/上锁：
 *   油门最低 + 偏航右打 保持 2 秒 = 解锁（电机开始怠速）
 *   油门最低 + 偏航左打 保持 2 秒 = 上锁（电机停转）
 *
 * 调试：串口 115200，每秒打印一次姿态/遥控/油门状态
 * ============================================================================
 */

#include <Arduino.h>
#include <math.h>

#include "src/ICM20948.h"   // IMU 姿态（提供 acc_g/gyr_dps/roll/pitch/yaw、icm_init、imu_calibrate、mahony_update6、quat_to_euler_deg）
#include "src/SBUS.h"       // 遥控（提供 rc_ch[]、sbus_begin、sbus_poll、sbus_get_lost）
#include "src/PID.h"        // PID（提供 PID 结构体、pid_init、pid_calc、内环参数宏）
#include "src/Motors.h"     // 电机（提供 motor_us[]、motors_begin、set_motor_us、set_all_motors）

/* ============================================================================
 * 控制环常量
 * ========================================================================== */
#define LOOP_HZ               1000
#define LOOP_US               (1000000UL / LOOP_HZ)

/* ---- 角度外环参数（串级 PID 外环，内环参数在 PID.h）---- */
#define ANGLE_P_ROLL          4.0f
#define ANGLE_P_PITCH         4.0f
#define ANGLE_P_YAW           2.0f

/* ---- 遥控通道映射（FlySky：CH1~CH4）---- */
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
 * 飞控状态
 * ========================================================================== */
static bool armed = false;           // 是否解锁
static uint32_t arm_start_ms = 0;   // 解锁动作开始时间
static bool arm_switching = false;   // 正在执行解锁/上锁动作

static PID pid_roll_rate, pid_pitch_rate, pid_yaw_rate;   // 内环实例（外环用比例增益直接计算）

/* ============================================================================
 * setup
 * ========================================================================== */
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println("========================================");
  Serial.println("  ESP32-S3 自研四旋翼 / M6 主飞控 v1 (modular)");
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
  motors_begin();
  Serial.println("[MOTOR] 全部油门 1000（停转）");

  // ---- SBUS 初始化 ----
  sbus_begin();
  Serial.println("[SBUS] GPIO18 软件反相");

  // ---- PID 初始化（内环）----
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

  // ---- 1. 读 IMU + 姿态解算 ----
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
  sbus_poll();

  // ---- 3. 失控保护：1 秒没收到帧 = 强制上锁 ----
  bool rc_lost = sbus_get_lost();
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
    // 自稳模式：目标角度 = 摇杆量 * 最大倾斜角，角度误差乘外环增益得目标角速度
    float roll_angle_err  = roll_cmd  * 25.0f - roll_deg;   // 最大倾斜 25 度
    float pitch_angle_err = pitch_cmd * 25.0f - pitch_deg;
    // 偏航没有角度环（靠手动），直接用摇杆量当目标角速度
    float yaw_target_rate = yaw_cmd * 100.0f;

    // 内环：角速度 PID（误差 = 目标角速度 - 实测角速度）
    float roll_rate_err  = (roll_angle_err  * ANGLE_P_ROLL)  - gyr_dps[0];
    float pitch_rate_err = (pitch_angle_err * ANGLE_P_PITCH) - gyr_dps[1];
    float yaw_rate_err   = yaw_target_rate - gyr_dps[2];

    float roll_out  = pid_calc(&pid_roll_rate,  roll_rate_err,  dt);
    float pitch_out = pid_calc(&pid_pitch_rate, pitch_rate_err, dt);
    float yaw_out   = pid_calc(&pid_yaw_rate,   yaw_rate_err,   dt);

    // ---- 7. X 型混控 ----
    // M1=右前  M2=左前  M3=右后  M4=左后
    if (thr_out < THROTTLE_IDLE) thr_out = THROTTLE_IDLE;   // 油门最低时给怠速

    float m1 = thr_out + roll_out + pitch_out - yaw_out;
    float m2 = thr_out - roll_out + pitch_out + yaw_out;
    float m3 = thr_out + roll_out - pitch_out + yaw_out;
    float m4 = thr_out - roll_out - pitch_out - yaw_out;

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
