/*
 * ============================================================================
 * ESP32-S3 自研四旋翼 —— M7 主飞控（自稳 + 返航，模块化版）
 * ----------------------------------------------------------------------------
 * 整合：ICM-20948 姿态解算 + SBUS 遥控 + 串级 PID + X 型混控 + CH5 解锁
 *       + CH6 手动返航 + 遥控丢失自动返航（GPS + 罗盘 + 气压定高）
 *
 * 【功能模块化】本工程 src/ 目录按功能拆分，一个功能一个 .h/.cpp：
 *   src/ICM20948.h/.cpp   九轴 IMU 驱动 + Mahony 姿态 + 零偏校准
 *   src/BMP280.h/.cpp     气压计驱动 + 相对高度
 *   src/GPS.h/.cpp        BN-880 NMEA 解析（定位/卫星/经纬度）
 *   src/MAG.h/.cpp        BN-880 内置罗盘（航向读取）
 *   src/RTL.h/.cpp        返航状态机（手动/失控自动返航）
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
 *   4. 失控保护：丢信号 1 秒自动返航（有 GPS 定位）；无定位则缓降上锁
 *   5. 返航依赖 GPS 定位 + 罗盘校准！未校准罗盘请勿使用返航（见 MAG.h）
 *
 * 接线（全部已验证）：
 *   ICM-20948 SPI：SCK=12 MISO=13 MOSI=11 CS=10（3V3）
 *   BMP280 I2C  ：SDA=8  SCL=9
 *   BN-880 GPS  ：RX=16  TX=17（罗盘共用 I2C：SDA=8 SCL=9）
 *   SBUS 接收机  ：RX=18（软件反相，无需反相器）
 *   电机 PWM     ：M1=4  M2=5  M3=6  M4=7（50Hz，1000-2000µs）
 *   电调 BEC 红线全挑出！全机 5V = 好盈 UBEC
 *
 * 遥控器通道（FlySky 默认）：
 *   CH1 = 横滚（左右）  CH2 = 俯仰（前后）
 *   CH3 = 油门（上下）  CH4 = 偏航（左右转向）
 *   CH5 = 解锁/上锁开关  CH6 = 返航开关
 *
 * 解锁/上锁：
 *   油门最低 + CH5 拨高 保持 2 秒 = 解锁
 *   CH5 拨低 = 上锁
 *
 * home 记录：
 *   解锁起飞后，GPS 定位连续 1 分钟自动记录当前位置为 home（供返航）
 *
 * 返航（120m 巡航高度）：
 *   CH6 拨高 或 遥控丢失 = 自动返航（爬升到 120m -> 飞回 home -> 降落 -> 上锁）
 *   无 GPS 定位时失控 = 直接缓降上锁（安全兜底）
 *
 * 调试：串口 115200，每秒打印一次姿态/遥控/GPS/返航状态
 * ============================================================================
 */

#include <Arduino.h>
#include <math.h>

#include "src/ICM20948.h"   // IMU 姿态（提供 acc_g/gyr_dps/roll/pitch/yaw、icm_init、imu_calibrate、mahony_update6、quat_to_euler_deg）
#include "src/SBUS.h"       // 遥控（提供 rc_ch[]、sbus_begin、sbus_poll、sbus_get_lost）
#include "src/PID.h"        // PID（提供 PID 结构体、pid_init、pid_calc、内环参数宏）
#include "src/Motors.h"     // 电机（提供 motor_us[]、motors_begin、set_motor_us、set_all_motors）
#include "src/BMP280.h"     // 气压计（提供 bmp_begin、bmp_read、bmp_set_ground、bmp_rel_alt_m）
#include "src/GPS.h"        // GPS（提供 gps_begin、gps_poll、gps_lat/gps_lon/gps_fix）
#include "src/MAG.h"        // 罗盘（提供 mag_begin、mag_read）
#include "src/RTL.h"        // 返航（提供 rtl_begin、rtl_set_home、rtl_update、RTL_Output）
#include "src/ALTHOLD.h"   // 定高（CH7 进入：高度外环->垂速内环->油门）
#include "src/POSHOLD.h"   // 悬停（CH7 进入：位置误差->倾角 + 航向锁定）

/* ============================================================================
 * 控制环常量
 * ========================================================================== */
#define LOOP_HZ               1000
#define LOOP_US               (1000000UL / LOOP_HZ)

/* ---- 角度外环参数（串级 PID 外环，内环参数在 PID.h）---- */
#define ANGLE_P_ROLL          4.0f
#define ANGLE_P_PITCH         4.0f
#define ANGLE_P_YAW           2.0f

/* ---- 遥控通道映射（FlySky：CH1~CH6）---- */
#define CH_ROLL               0    // CH1
#define CH_PITCH              1    // CH2
#define CH_THROTTLE           2    // CH3
#define CH_YAW                3    // CH4
#define CH_ARM                4    // CH5 解锁/上锁开关（高=解锁，低=上锁）
#define CH_RTL                5    // CH6 返航开关（高=返航）
#define CH_POS                6    // CH7 定高/悬停开关（高=进入）

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

// ---- BMP280 状态 ----
static bool bmp_ok = false;             // 气压计是否在线
static float bmp_tempC = 0, bmp_pressPa = 0, bmp_relAlt = 0;

// ---- GPS / 罗盘 / 返航状态（M7）----
static bool gps_ok = false;             // GPS 串口是否在线（能收到语句）
static bool mag_ok = false;             // 罗盘是否在线
static float mag_heading = 0.0f;        // 罗盘航向（度 0~360）
static bool  rtl_active = false;        // 是否处于返航模式
static bool  rtl_src_failsafe = false;  // 返航触发源：true=失控自动（不可中途退出），false=CH6 手动
static RTL_Output rtl_out;              // 返航输出指令

// ---- home 记录：起飞后 GPS 定位连续 1 分钟才记录（用户要求）----
#define HOME_LOCK_DELAY_MS   60000UL   // 定位连续保持时长
static bool      gps_locked = false;   // 是否已进入定位计时
static uint32_t  gps_lock_ms = 0;      // 定位连续保持计时起点

// ---- 罗盘自动校准模式（未解锁：油门最低 + CH6 拨高 3 秒进入）----
static bool     mag_cal_mode = false;  // 校准采样进行中
static bool     mag_cal_holding = false; // CH6 保持计时中
static uint32_t mag_cal_hold_ms = 0;   // 保持计时起点

/* ============================================================================
 * setup
 * ========================================================================== */
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println("========================================");
  Serial.println("  ESP32-S3 自研四旋翼 / M7 主飞控 (modular)");
  Serial.println("========================================");

  // ---- I2C 总线统一初始化（BMP280 + 罗盘共用 SDA=8 SCL=9）----
  Wire.begin(8, 9);

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

  // ---- BMP280 气压计 ----
  bmp_ok = bmp_begin();
  if (bmp_ok) {
    bmp_read(bmp_tempC, bmp_pressPa);
    bmp_set_ground(bmp_pressPa);   // 记录起飞参考气压
    Serial.printf("[BMP] 在线 | %.1f C | %.0f Pa | 参考高度已记录\n",
                  bmp_tempC, bmp_pressPa);
  } else {
    Serial.println("[BMP] 未找到！检查 SDA=8 SCL=9 / VCC=3V3 / CSB 接 3V3");
  }

  // ---- GPS 初始化（M7）----
  gps_begin();
  gps_ok = true;   // 波特率扫描完成后串口已配置，能否定位看后续 gps_fix
  Serial.println("[GPS] 已初始化（室外等待定位，室内无信号正常）");

  // ---- 罗盘初始化（M7）----
  mag_ok = mag_begin();
  if (mag_ok) {
    mag_load_cal();   // 从 NVS 加载上次校准值
    Serial.printf("[MAG] 罗盘在线，校准偏移 X=%.1f Y=%.1f（未校则为 0）\n",
                  mag_x_off, mag_y_off);
  } else {
    Serial.println("[MAG] 罗盘未找到！返航功能不可用");
  }

  // ---- 返航模块 ----
  rtl_begin();
  Serial.println("[RTL] 返航模块就绪（CH6 或失控自动返航）");

  // ---- 定高/悬停模块 ----
  althold_begin();
  althold_load_hover();   // 从 NVS 加载悬停油门基准（自动学习结果）
  poshold_begin();
  Serial.printf("[ALT] 定高/悬停就绪，悬停油门基准 %.0fµs（CH7 高进入；悬停中自动学习）\n",
                althold_hover_thr());

  // ---- PID 初始化（内环）----
  pid_init(&pid_roll_rate,  RATE_P_ROLL,  RATE_I_ROLL,  RATE_D_ROLL);
  pid_init(&pid_pitch_rate, RATE_P_PITCH, RATE_I_PITCH, RATE_D_PITCH);
  pid_init(&pid_yaw_rate,   RATE_P_YAW,   RATE_I_YAW,   RATE_D_YAW);

  Serial.println();
  Serial.println("=== 就绪 ===");
  Serial.println("解锁：油门最低 + CH5 拨高 2 秒");
  Serial.println("上锁：CH5 拨低");
  Serial.println("返航：CH6 拨高（120m 巡航；需 home + 罗盘校准）");
  Serial.println("home：起飞后 GPS 定位连续 1 分钟自动记录");
  Serial.println("失控：自动返航（无定位则缓降上锁）");
  if (mag_ok) {
    Serial.println("罗盘校准：未解锁 + 油门最低 + CH6 拨高 3 秒（旋转整机 30 秒自动完成）");
  }
  Serial.println("定高+悬停：解锁后 CH7 拨高（悬停稳定时自动学习悬停油门）；CH7 拨低退出");
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

  // ---- 2.5 读 BMP280（100Hz 节拍，I2C 约 2ms，不影响 1kHz 环）----
  if (bmp_ok) {
    static uint32_t tBmp = 0;
    if (now - tBmp >= 10000UL) {           // 每 10ms 一次
      tBmp = now;
      bmp_read(bmp_tempC, bmp_pressPa);
      if (bmp_pressPa > 0) {
        bmp_relAlt = bmp_rel_alt_m(bmp_pressPa);
      }
    }
  }

  // ---- 2.6 GPS + 罗盘采样（10Hz 节拍）----
  static uint32_t tNav = 0;
  if (now - tNav >= 100000UL) {
    tNav = now;
    gps_poll();                            // GPS 串口收包 + GGA 解析
    if (gps_fix) gps_ok = true;
    if (mag_ok) mag_read(mag_heading);
  }

  // ---- 3. 失控保护 + 返航触发（CH6 手动 / 遥控丢失自动）----
  bool rc_lost = sbus_get_lost();
  bool ch6_rtl = (rc_ch[CH_RTL] > 1500);   // CH6 拨高 = 手动返航

  if (rc_lost) {
    // —— 遥控丢失：自动返航（有定位）或缓降上锁（无定位）——
    if (armed && !rtl_active) {
      if (gps_fix && rtl_home_valid()) {
        rtl_reset();               // 确保从 IDLE/CLIMB 重新开始
        rtl_active = true;
        rtl_src_failsafe = true;   // 失控触发，不可中途退出
        rtl_out = rtl_update(true, true, gps_lat, gps_lon, mag_heading, bmp_relAlt);
        Serial.println("[!] 遥控丢失 → 自动返航");
      } else {
        // 无定位无法返航：安全兜底直接上锁
        armed = false;
        arm_switching = false;
        rtl_active = false;
        rtl_reset();
        set_all_motors(MIN_US);
        Serial.println("[!] 遥控丢失且无 GPS 定位 → 强制上锁（安全策略）");
      }
    }
  } else if (armed && ch6_rtl) {
    // —— CH6 手动返航 ——
    if (!rtl_active) {
      if (gps_fix && rtl_home_valid()) {
        rtl_reset();               // 确保从 IDLE/CLIMB 重新开始
        rtl_active = true;
        rtl_src_failsafe = false;  // 手动触发
        rtl_out = rtl_update(true, false, gps_lat, gps_lon, mag_heading, bmp_relAlt);
        Serial.println("[!] CH6 拨高 → 手动返航");
      } else {
        Serial.println("[!] CH6 返航被拒绝：无 GPS 定位/home（继续手动控制）");
      }
    }
  } else {
    // —— 遥控正常且 CH6 松开：仅手动返航可退出；降落阶段禁止退出（避免悬空）——
    if (rtl_active && !rtl_src_failsafe &&
        rtl_out.mode != RTL_DESCEND && rtl_out.mode != RTL_LANDED) {
      rtl_active = false;
      rtl_reset();
      Serial.println("[RTL] 手动返航已退出（CH6 拨低）");
    }
  }

  // ---- 3.5 罗盘自动校准模式（未解锁：油门最低 + CH6 拨高 3 秒）----
  bool cal_hold = !armed && mag_ok &&
                  (rc_ch[CH_THROTTLE] < 1050) && (rc_ch[CH_RTL] > 1500);
  if (cal_hold) {
    if (!mag_cal_holding) { mag_cal_holding = true; mag_cal_hold_ms = millis(); }
    if (!mag_cal_mode && millis() - mag_cal_hold_ms > 3000UL) {
      mag_cal_mode = true;
      mag_cal_start();
      Serial.println(">>> 进入罗盘校准：水平端平机身，慢速旋转整机 2~3 圈 + 8 字 <<<");
      Serial.println(">>> 30 秒自动完成，或拨低 CH6 提前结束 <<<");
    }
  } else {
    if (mag_cal_mode) {            // CH6 拨低 = 提前结束并用现有极值保存
      mag_cal_finish();
      mag_cal_mode = false;
      Serial.printf(">>> 校准已保存：X_off=%.1f Y_off=%.1f <<<\n", mag_x_off, mag_y_off);
    }
    mag_cal_holding = false;
  }
  if (mag_cal_mode) {
    int r = mag_cal_step();        // 采样推进 + 30 秒自动完成
    if (r == 1) {
      mag_cal_mode = false;
      mag_cal_holding = false;
      Serial.printf(">>> 校准完成：X_off=%.1f Y_off=%.1f <<<\n", mag_x_off, mag_y_off);
      Serial.println(">>> 验证：机头朝真北 HDG≈0/360，朝东≈90（±3° 内合格）<<<");
    }
  }

  // ---- 4. 解锁/上锁逻辑（M7：CH5 开关控制）----
  //   解锁：油门最低 + CH5 拨高 保持 2 秒；上锁：CH5 拨低
  float rc_thr = rc_ch[CH_THROTTLE];
  bool thr_low = (rc_thr < 1050);
  bool ch5_arm = (rc_ch[CH_ARM] > 1500);   // CH5 高 = 请求解锁

  if (thr_low && !rc_lost && !mag_cal_mode && !mag_cal_holding) {
    if (ch5_arm) {
      if (!arm_switching) { arm_switching = true; arm_start_ms = millis(); }
      if (!armed && millis() - arm_start_ms > 2000) {
        armed = true;
        arm_switching = false;
        Serial.println(">>> 已解锁 <<<");
        if (!gps_fix) {
          Serial.println(">>> 等待 GPS 定位...（定位连续 1 分钟后自动记录 home）<<<");
        }
      }
    } else {
      if (armed) {
        armed = false;
        arm_switching = false;
        rtl_active = false;          // 清空返航状态：防残留导致再解锁后续飞返航
        rtl_reset();
        set_all_motors(MIN_US);
        Serial.println(">>> CH5 拨低，已上锁，电机停转 <<<");
      }
      arm_switching = false;
    }
  } else {
    arm_switching = false;
  }

  // ---- 4.5 home 记录：起飞后 GPS 定位连续 1 分钟 ----
  if (armed) {
    if (gps_fix) {
      if (!gps_locked) { gps_locked = true; gps_lock_ms = millis(); }
      if (!rtl_home_valid() && millis() - gps_lock_ms >= HOME_LOCK_DELAY_MS) {
        rtl_set_home(gps_lat, gps_lon);
        Serial.printf(">>> home 已记录: %.6f, %.6f <<<\n", gps_lat, gps_lon);
      }
    } else {
      gps_locked = false;   // 定位丢失，重新计时
    }
  } else {
    gps_locked = false;
  }

  // ---- 4.6 返航状态机推进（rtl_active 且已解锁时每环更新）----
  if (rtl_active && armed) {
    rtl_out = rtl_update(armed, rc_lost, gps_lat, gps_lon, mag_heading, bmp_relAlt);
  }

  // ---- 5. 遥控通道映射 + 死区 ----
  float rc_roll  = rc_ch[CH_ROLL];
  float rc_pitch = rc_ch[CH_PITCH];
  float rc_yaw   = rc_ch[CH_YAW];
  // 归一化到 -1 ~ +1
  float roll_cmd  = (fabs(rc_roll  - RC_MID)  < RC_DEADBAND) ? 0 : (rc_roll  - RC_MID)  / (RC_MAX - RC_MID);
  float pitch_cmd = (fabs(rc_pitch - RC_MID)  < RC_DEADBAND) ? 0 : (rc_pitch - RC_MID) / (RC_MAX - RC_MID);
  float yaw_cmd   = (fabs(rc_yaw   - RC_MID)  < RC_DEADBAND) ? 0 : (rc_yaw   - RC_MID)   / (RC_MAX - RC_MID);

  // 油门映射：1000~2000 -> 1000~1900
  float thr_out = map(rc_thr, RC_MIN, RC_MAX, THROTTLE_MIN, THROTTLE_MAX);

  // ---- 5.5 返航模式：用 RTL 输出指令覆盖遥控 ----
  if (rtl_active && armed) {
    // 在返航中：油门由 RTL 状态机控制，姿态角由 RTL 计算
    thr_out = rtl_out.throttle_us;
  }

  // ---- 5.6 定高/悬停模式（CH7 高进入；返航/上锁/失控优先退出）----
  float ph_pitch_cmd = 0, ph_roll_cmd = 0, ph_yaw_cmd = 0;
  bool ch7_pos = (rc_ch[CH_POS] > 1500);
  if (armed && !rtl_active && !rc_lost) {
    if (ch7_pos) {
      if (!althold_active()) althold_start(bmp_relAlt, (uint16_t)thr_out);
      if (gps_fix && !poshold_active()) poshold_start(gps_lat, gps_lon, mag_heading);
      if (!gps_fix && poshold_active()) poshold_stop();
      thr_out = althold_update(bmp_relAlt, rc_thr);
      poshold_update(gps_lat, gps_lon, mag_heading,
                     rc_pitch, rc_roll, dt,
                     ph_pitch_cmd, ph_roll_cmd, ph_yaw_cmd);
      if (!poshold_active()) { ph_pitch_cmd = 0; ph_roll_cmd = 0; ph_yaw_cmd = 0; }
    } else {
      if (althold_active()) althold_stop();
      if (poshold_active()) poshold_stop();
    }
  } else {
    if (althold_active()) althold_stop();
    if (poshold_active()) poshold_stop();
  }

  // ---- 6. 串级 PID 计算 ----
  // 返航激活时（含失控触发）也必须执行控制环，用 RTL 指令输出
  if (armed && (rtl_active || !rc_lost)) {
    float roll_angle_err, pitch_angle_err, yaw_target_rate;

    if (rtl_active) {
      // 返航：目标是 RTL 给出的姿态角/偏航速率
      roll_angle_err   = rtl_out.target_roll   - roll_deg;
      pitch_angle_err  = rtl_out.target_pitch  - pitch_deg;
      yaw_target_rate  = rtl_out.target_yaw_rate;
    } else if (poshold_active()) {
      // 悬停：目标倾角/偏航率由 POSHOLD 给出（位置误差->倾角，航向锁定）
      roll_angle_err   = ph_roll_cmd  - roll_deg;
      pitch_angle_err  = ph_pitch_cmd - pitch_deg;
      yaw_target_rate  = ph_yaw_cmd;
    } else {
      // 自稳模式：目标角度 = 摇杆量 * 最大倾斜角
      roll_angle_err   = roll_cmd  * 25.0f - roll_deg;   // 最大倾斜 25 度
      pitch_angle_err  = pitch_cmd * 25.0f - pitch_deg;
      yaw_target_rate  = yaw_cmd * 100.0f;              // 偏航手动
    }

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

    // 返航完成后自动上锁
    if (rtl_active && rtl_out.mode == RTL_LANDED) {
      armed = false;
      rtl_active = false;
      rtl_reset();
      set_all_motors(MIN_US);
      Serial.println(">>> 返航完成，已上锁 <<<");
    }
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
    const char *rtl_name = mag_cal_mode ? "MAGCAL" : (rtl_active ? "RTL " : "MANU");
    if (mag_cal_mode) {
      rtl_name = "MAGCAL";
    } else if (rtl_active) {
      switch (rtl_out.mode) {
        case RTL_CLIMB:   rtl_name = "RTL-CLIMB"; break;
        case RTL_FLY:     rtl_name = "RTL-FLY  "; break;
        case RTL_DESCEND: rtl_name = "RTL-DESC "; break;
        case RTL_LANDED:  rtl_name = "RTL-LAND "; break;
        default: break;
      }
    } else if (poshold_active()) {
      rtl_name = "POSHOLD";
    }
    Serial.printf("%s %s | R:%6.1f P:%6.1f Y:%6.1f | ALT:%+6.1fm | RC: %4u %4u %4u %4u %4u %4u %4u | GPS:%c%u %6.4f,%6.4f | HOME:%c | HDG:%5.1f | M: %4u %4u %4u %4u\n",
                  armed ? "ARMED " : "DISARM",
                  rtl_name,
                  roll_deg, pitch_deg, yaw_deg,
                  bmp_ok ? bmp_relAlt : 0.0f,
                  rc_ch[0], rc_ch[1], rc_ch[2], rc_ch[3], rc_ch[4], rc_ch[5], rc_ch[6],
                  gps_fix ? 'F' : 'N', gps_sats, gps_lat, gps_lon,
                  rtl_home_valid() ? 'Y' : 'N',
                  mag_ok ? mag_heading : -1.0f,
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
