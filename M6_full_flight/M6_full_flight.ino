/*
 * ============================================================================
 * ESP32-S3 自研四旋翼 —— M7 主飞控（自稳 + 返航，模块化版）
 * ----------------------------------------------------------------------------
 * 整合：ICM-20948 姿态解算 + iBus(FlySky) 遥控 + 串级 PID + X 型混控 + CH5 解锁
 *       + CH6 手动返航 + 遥控丢失自动返航（GPS + 罗盘 + 气压定高）
 *
 * 【功能模块化】本工程 src/ 目录按功能拆分，一个功能一个 .h/.cpp：
 *   src/ICM20948.h/.cpp   九轴 IMU 驱动 + Mahony 姿态 + 零偏校准
 *   src/BMP280.h/.cpp     气压计驱动 + 相对高度
 *   src/GPS.h/.cpp        BN-880 NMEA 解析（定位/卫星/经纬度）
 *   src/MAG.h/.cpp        BN-880 内置罗盘（航向读取）
 *   src/RTL.h/.cpp        返航状态机（手动/失控自动返航）
 *   src/SBUS.h/.cpp       iBus 遥控解析（GPIO44 19200 8N1，FS-iA6B；UART0，与 GPS 的 UART2 分离）
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
 *   iBus 接收机  ：RX=44（UART0，19200 8N1，FS-iA6B，无需反相；⚠ GPS 已占用 UART2(16/17)）
 *   电机 PWM     ：M1=4  M2=5  M3=6  M4=7（50Hz，1000-2000µs）
 *   数传 LINK   ：TX=14 RX=15（UART1，57600 8N1，模块可更换）
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
#include "src/WAYPOINT.h"   // 航点飞行（WiFi 网页下发目标坐标）
#include "src/PID.h"        // PID（提供 PID 结构体、pid_init、pid_calc、内环参数宏）
#include "src/Motors.h"     // 电机（提供 motor_us[]、motors_begin、set_motor_us、set_all_motors）
#include "src/BMP280.h"     // 气压计（提供 bmp_begin、bmp_read、bmp_set_ground、bmp_rel_alt_m）
#include "src/GPS.h"        // GPS（提供 gps_begin、gps_poll、gps_lat/gps_lon/gps_fix）
#include "src/MAG.h"        // 罗盘（提供 mag_begin、mag_read）
#include "src/RTL.h"        // 返航（提供 rtl_begin、rtl_set_home、rtl_update、RTL_Output）
#include "src/ALTHOLD.h"   // 定高（CH7 进入：高度外环->垂速内环->油门）
#include "src/POSHOLD.h"   // 悬停（CH7 进入：位置误差->倾角 + 航向锁定）
#include "src/VERTEST.h"   // 垂直融合（加速度+气压互补：融合高度/垂速）
#include "src/LOGGER.h"    // 飞行黑匣子（RAM 环形 + Flash 落盘，串口 D 导出）
#include "src/TELEMETRY.h" // WiFi 实时看板（手机连热点浏览器查看/下载）
#include "src/LINK.h"     // 数传链路（UART1：TX=14 RX=15，57600）
#include "src/PARAM.h"    // 参数在线调整（PARAM:SET/GET/RESET，NVS 持久化）

/* ============================================================================
 * 控制环常量
 * ========================================================================== */
#define LOOP_HZ               1000
#define LOOP_US               (1000000UL / LOOP_HZ)

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
bool wp_pending = false;             // 航点请求（WiFi 网页置位，主程序校验后执行）

// ---- 无 GPS 失控缓降（安全策略：怠速慢降，触地/超时自动上锁）----
#define GLIDE_THROTTLE_US   1150     // 缓降油门（低于悬停基准，缓慢下降）
#define GLIDE_LAND_ALT_M    0.6f     // 视为触地的高度阈值（相对起飞点）
#define GLIDE_TIMEOUT_MS    15000UL  // 缓降最长 15s，超时强制上锁（兜底）
static bool     glide_landing = false; // 失控缓降进行中
static uint32_t glide_start_ms = 0;
static WP_Output wp_out;             // 航点输出指令
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

  // ---- 遥控（iBus）初始化 ----
  sbus_begin();
  Serial.println("[RC] iBus(FS-iA6B) GPIO44(UART0) 19200 8N1");

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

  // ---- 垂直融合初始化（M10：BMP 已记录地面参考气压，rel_alt 初始=0）----
  vertest_begin();
  vertest_reset(0.0f);
  Serial.println("[VERT] 垂直融合就绪（加速度+气压互补，垂速噪声降 5~10 倍）");

  // ---- 定高/悬停模块 ----
  althold_begin();
  althold_load_hover();   // 从 NVS 加载悬停油门基准（自动学习结果）
  poshold_begin();
  Serial.printf("[ALT] 定高/悬停就绪，悬停油门基准 %.0fµs（CH7 高进入；悬停中自动学习）\n",
                althold_hover_thr());

  // ---- 参数加载（NVS，可在线调整）----
  param_begin();

  // ---- PID 初始化（内环，参数来自 PARAM）----
  pid_init(&pid_roll_rate,  rate_p_roll,  rate_i_roll,  rate_d_roll);
  pid_init(&pid_pitch_rate, rate_p_pitch, rate_i_pitch, rate_d_pitch);
  pid_init(&pid_yaw_rate,   rate_p_yaw,   rate_i_yaw,   rate_d_yaw);

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
  // ---- 黑匣子 + WiFi 看板初始化 ----
  logger_begin();
  telemetry_begin();
  link_begin();
  Serial.println("[LINK] 数传就绪：UART1 GPIO14(TX)/15(RX) 57600（10Hz 遥测上行，命令下行）");

  Serial.println("⚠ 首次测试务必拆桨！");
}

/* ============================================================================
 * loop（1kHz 控制环）
 * ========================================================================== */
void loop() {
  // ---- 串口命令：D = 导出黑匣子日志 ----
  if (Serial.available()) {
    char c = (char)Serial.read();
    if (c == 'D' || c == 'd') logger_dump_all();
  }

  // ---- 数传命令下行：航点/日志请求（地面站 → 433MHz → UART1）----
  {
    String lnk;
    while (link_read_line(lnk)) {
      if (lnk == "D" || lnk == "d") {
        char lb[LOG_LINE_LEN];
        uint32_t seqx = logger_seq();
        for (uint32_t ss = (seqx > 10) ? seqx - 9 : 1; ss <= seqx; ss++) {
          if (logger_line_at(ss, lb)) link_send(String(lb));
        }
      } else {
        tel_wp_cmd(lnk);
      }
    }
  }

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

  // ---- 1.2 垂直融合预测（1kHz，加速度积分更新高度/垂速）----
  vertest_predict(acc_g[0], acc_g[1], acc_g[2], imu_q, dt);   // 姿态补偿：机体3轴加速度→地理系向上分量

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
        vertest_correct(bmp_relAlt);   // 垂直融合：气压观测修正（10Hz）
      }
    }
  }

  // ---- 2.6 GPS + 罗盘采样（10Hz 节拍）----
  static uint32_t tNav = 0;
  if (now - tNav >= 100000UL) {
    tNav = now;
    gps_poll();                            // GPS 串口收包 + GGA 解析
    if (gps_fix) {
      gps_ok = true;
      mag_auto_update(gps_lat, gps_lon);   // 磁偏角自动识别（WMM 查表）
    }
    if (mag_ok) mag_read(mag_heading);
  }

  // ---- 3. 失控保护 + 返航触发（CH6 手动 / 遥控丢失自动）----
  bool rc_lost = sbus_get_lost();
  bool ch6_rtl = (rc_ch[CH_RTL] > 1500);   // CH6 拨高 = 手动返航

  if (rc_lost) {
    // —— 遥控丢失：自动返航（有定位）或缓降上锁（无定位）——
    if (armed && rtl_active &&
        (rtl_out.mode == RTL_DESCEND || rtl_out.mode == RTL_LANDED)) {
      // 返航已进入降落段：失控不打断，继续降落（禁止重新爬升！）
      rtl_src_failsafe = true;   // 标记为失控源，防止手动退出
    } else if (armed && !rtl_active) {
      if (gps_fix && rtl_home_valid()) {
        rtl_reset();               // 确保从 IDLE/CLIMB 重新开始
        rtl_active = true;
        rtl_src_failsafe = true;   // 失控触发，不可中途退出
        wp_pending = false;
        wp_stop();
        rtl_out = rtl_update(true, true, gps_lat, gps_lon, mag_heading, vertest_alt());
        Serial.println("[!] 遥控丢失 → 自动返航");
      } else {
        // 无定位无法返航：怠速缓降（不直接停桨，避免自由落体）
        if (!glide_landing) {
          glide_landing = true;
          glide_start_ms = millis();
          wp_pending = false;
          wp_stop();
          Serial.println("[!] 遥控丢失且无 GPS 定位 → 怠速缓降（触地自动上锁）");
        }
      }
    }
  } else if (armed && ch6_rtl) {
    // —— CH6 手动返航 ——
    if (!rtl_active) {
      if (gps_fix && rtl_home_valid()) {
        rtl_reset();               // 确保从 IDLE/CLIMB 重新开始
        rtl_active = true;
        rtl_src_failsafe = false;  // 手动触发
        wp_pending = false;
        wp_stop();
        rtl_out = rtl_update(true, false, gps_lat, gps_lon, mag_heading, vertest_alt());
        Serial.println("[!] CH6 拨高 → 手动返航");
      } else {
        Serial.println("[!] CH6 返航被拒绝：无 GPS 定位/home（继续手动控制）");
      }
    }
  } else {
    // —— 遥控恢复：退出失控缓降，恢复正常手动控制 ——
    if (glide_landing && !rtl_active) {
      glide_landing = false;
      Serial.println("[!] 遥控恢复，退出缓降（继续手动控制）");
    }
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
    static uint32_t tMagCal = 0;
    if (now - tMagCal < 10000UL) { } else {   // 10ms 节流，避免拖慢 1kHz 环
    tMagCal = now;
    int r = mag_cal_step();        // 采样推进 + 30 秒自动完成
    if (r == 1) {
      mag_cal_mode = false;
      mag_cal_holding = false;
      Serial.printf(">>> 校准完成：X_off=%.1f Y_off=%.1f <<<\n", mag_x_off, mag_y_off);
      Serial.println(">>> 验证：机头朝真北 HDG≈0/360，朝东≈90（±3° 内合格）<<<");
    }
    }   // 磁校准节流结束
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
        // 解锁瞬间重新记录地面气压 + 垂直融合归零（防气压漂移导致高度基准偏）
        if (bmp_ok) {
          bmp_read(bmp_tempC, bmp_pressPa);
          if (bmp_pressPa > 0) {
            bmp_set_ground(bmp_pressPa);
            bmp_relAlt = 0.0f;
            vertest_reset(0.0f);
          }
        }
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
        wp_pending = false;
        wp_stop();
        set_all_motors(MIN_US);
        Serial.println(">>> CH5 拨低，已上锁，电机停转 <<<");
        logger_flush();   // 黑匣子落盘
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
    rtl_out = rtl_update(armed, rc_lost, gps_lat, gps_lon, mag_heading, vertest_alt());
  }

  // ---- 4.7 航点飞行推进（网页请求 + 已解锁 + 定位 + 无返航/无失控）----
  if (wp_pending) {
    if (!armed) { wp_pending = false; wp_stop(); }            // 上锁即取消
    else if (rc_lost || rtl_active) { wp_pending = false; wp_stop(); }  // 返航/失控优先
    else if (!gps_fix) {
      // GPS 定位丢失：航线不可持续，立即取消回手动（防止油门/姿态指令悬空）
      wp_pending = false; wp_stop();
      Serial.println("[WP] GPS 定位丢失，航线已取消（回手动）");
    }
    else {
      // 摇杆偏离中位 → 立即退出航点回手动（用户接管优先，与 README 一致）
      if (fabsf(rc_ch[CH_ROLL]  - RC_MID) > RC_DEADBAND + 10 ||
          fabsf(rc_ch[CH_PITCH] - RC_MID) > RC_DEADBAND + 10 ||
          fabsf(rc_ch[CH_YAW]   - RC_MID) > RC_DEADBAND + 10) {
        wp_pending = false; wp_stop();
        Serial.println("[WP] 摇杆接管，航线已退出（回手动）");
      } else {
        wp_out = wp_update(armed, gps_lat, gps_lon, mag_heading, vertest_alt());
      }
    }
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

  // ---- 5.55 航点模式：用 WP 输出指令覆盖遥控 ----
  if (wp_pending && armed && gps_fix && !rtl_active && !rc_lost) {
    thr_out = wp_out.throttle_us;
  }

  // ---- 5.555 失控缓降：油门固定怠速，自然下滑（姿态已归水平）----
  if (glide_landing && armed) {
    thr_out = GLIDE_THROTTLE_US;
  }

  // ---- 5.6 定高/悬停模式（CH7 高进入；返航/上锁/失控优先退出）----
  float ph_pitch_cmd = 0, ph_roll_cmd = 0, ph_yaw_cmd = 0;
  bool ch7_pos = !wp_pending && (rc_ch[CH_POS] > 1500);   // 航点模式下 CH7 定高不接管（防止油门被二次覆盖）
  if (armed && !rtl_active && !rc_lost) {
    if (ch7_pos) {
      if (!althold_active()) althold_start(vertest_alt(), (uint16_t)thr_out);
      if (gps_fix && !poshold_active()) poshold_start(gps_lat, gps_lon, mag_heading);
      if (!gps_fix && poshold_active()) poshold_stop();
      thr_out = althold_update(vertest_alt(), vertest_vel(), rc_thr);
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
  if (armed && (rtl_active || !rc_lost || glide_landing)) {
    float roll_angle_err, pitch_angle_err, yaw_target_rate;

    if (glide_landing) {
      // 失控缓降：姿态目标水平（保持机体水平缓慢下降，不跟冻结摇杆）
      roll_angle_err   = -roll_deg;
      pitch_angle_err  = -pitch_deg;
      yaw_target_rate  = 0;
    } else if (rtl_active) {
      // 返航：目标是 RTL 给出的姿态角/偏航速率
      roll_angle_err   = rtl_out.target_roll   - roll_deg;
      pitch_angle_err  = rtl_out.target_pitch  - pitch_deg;
      yaw_target_rate  = rtl_out.target_yaw_rate;
    } else if (wp_pending && armed && gps_fix && !rc_lost) {
      // 航点：目标是 WP 给出的姿态角/偏航速率
      roll_angle_err   = wp_out.target_roll   - roll_deg;
      pitch_angle_err  = wp_out.target_pitch  - pitch_deg;
      yaw_target_rate  = wp_out.target_yaw_rate;
    } else if (poshold_active()) {
      // 悬停：目标倾角/偏航率由 POSHOLD 给出（位置误差->倾角，航向锁定）
      roll_angle_err   = ph_roll_cmd  - roll_deg;
      pitch_angle_err  = ph_pitch_cmd - pitch_deg;
      yaw_target_rate  = ph_yaw_cmd;
    } else {
      // 自稳模式：目标角度 = 摇杆量 * 最大倾斜角
      roll_angle_err   = roll_cmd  * max_tilt_manu - roll_deg;
      pitch_angle_err  = pitch_cmd * max_tilt_manu - pitch_deg;
      yaw_target_rate  = yaw_cmd * yaw_rate_manu;
    }

    // 内环：角速度 PID（误差 = 目标角速度 - 实测角速度）
    float roll_rate_err  = (roll_angle_err  * angle_p_roll)  - gyr_dps[0];
    float pitch_rate_err = (pitch_angle_err * angle_p_pitch) - gyr_dps[1];
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
      logger_flush();   // 黑匣子落盘
    }
  } else {
    set_all_motors(MIN_US);
    pid_roll_rate.i_term = 0;
    pid_pitch_rate.i_term = 0;
    pid_yaw_rate.i_term = 0;
  }

  // ---- 7.5 缓降落地检测（100Hz 节拍；气压可用时按高度判触地，否则等超时兜底）----
  if (glide_landing) {
    static uint32_t tGlide = 0;
    if (now - tGlide >= 10000UL) {
      tGlide = now;
      bool landed = (bmp_ok && vertest_alt() <= GLIDE_LAND_ALT_M) ||
                    (millis() - glide_start_ms > GLIDE_TIMEOUT_MS);
      if (landed) {
        armed = false;
        glide_landing = false;
        rtl_active = false;
        rtl_reset();
        wp_pending = false;
        wp_stop();
        set_all_motors(MIN_US);
        Serial.println(">>> 缓降完成，已上锁 <<<");
        logger_flush();   // 黑匣子落盘
      }
    }
  }

  // ---- 8. 10Hz 状态打印 ----
  static uint32_t tPrint = 0;
  if (now - tPrint >= 100000UL) {
    tPrint = now;
    const char *rtl_name = mag_cal_mode ? "MAGCAL" : (glide_landing ? "GLIDE " : (rtl_active ? "RTL " : "MANU"));
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
    } else if (wp_pending && gps_fix && wp_active()) {
      switch (wp_out.mode) {
        case WP_CLIMB:   rtl_name = "WP-CLIMB "; break;
        case WP_FLY:     rtl_name = "WP-FLY   "; break;
        case WP_ARRIVED: rtl_name = "WP-ARRIV "; break;
        default: break;
      }
    } else if (poshold_active()) {
      rtl_name = "POSHOLD";
    }
    char log_line[LOG_LINE_LEN];
    snprintf(log_line, sizeof(log_line),
             "%s %s | R:%6.1f P:%6.1f Y:%6.1f | ALT:%+6.1fm | RC: %4u %4u %4u %4u %4u %4u %4u | GPS:%c%u %6.4f,%6.4f | HOME:%c | HDG:%5.1f | M: %4u %4u %4u %4u\n",
             armed ? "ARMED " : "DISARM",
             rtl_name,
             roll_deg, pitch_deg, yaw_deg,
             bmp_ok ? vertest_alt() : 0.0f,
             rc_ch[0], rc_ch[1], rc_ch[2], rc_ch[3], rc_ch[4], rc_ch[5], rc_ch[6],
             gps_fix ? 'F' : 'N', gps_sats, gps_lat, gps_lon,
             rtl_home_valid() ? 'Y' : 'N',
             mag_ok ? mag_heading : -1.0f,
             motor_us[0], motor_us[1], motor_us[2], motor_us[3]);
    Serial.print(log_line);
    logger_log(log_line);

    // ---- 遥测快照（WiFi 仪表盘 /api，10Hz）----
    tel_status.armed = armed;
    strncpy(tel_status.mode, rtl_name, sizeof(tel_status.mode) - 1);
    tel_status.mode[sizeof(tel_status.mode) - 1] = 0;
    tel_status.roll  = roll_deg;
    tel_status.pitch = pitch_deg;
    tel_status.yaw   = yaw_deg;
    tel_status.alt   = bmp_ok ? vertest_alt() : 0.0f;
    tel_status.vspd  = bmp_ok ? vertest_vel() : 0.0f;

    // 水平地速：GPS 位置差分（10Hz，无定位时为 0）
    {
      static float spd_prev_lat = 0, spd_prev_lon = 0;
      if (gps_fix && spd_prev_lat != 0) {
        float dLat = (gps_lat - spd_prev_lat) * 111320.0f;
        float dLon = (gps_lon - spd_prev_lon) * 111320.0f * cosf(gps_lat * DEG2RAD);
        tel_status.spd = sqrtf(dLat * dLat + dLon * dLon) / 0.1f;
      } else {
        tel_status.spd = 0;
      }
      spd_prev_lat = gps_lat;
      spd_prev_lon = gps_lon;
    }

    for (int i = 0; i < 7; i++) tel_status.rc[i] = rc_ch[i];
    tel_status.gps_fix    = gps_fix;
    tel_status.gps_sats   = gps_sats;
    tel_status.gps_lat    = gps_lat;
    tel_status.gps_lon    = gps_lon;
    tel_status.home_valid = rtl_home_valid();
    tel_status.home_lat   = rtl_home_valid() ? rtl_home_lat() : 0.0f;
    tel_status.home_lon   = rtl_home_valid() ? rtl_home_lon() : 0.0f;
    tel_status.hdg        = mag_ok ? mag_heading : -1.0f;
    tel_status.mag_decl   = mag_decl;
    tel_status.rssi_pct   = sbus_signal_pct();
    tel_status.mag_cal    = mag_cal_done() ? 1 : 0;
    tel_status.wp_dist    = wp_pending ? wp_out.dist : -1.0f;
    for (int i = 0; i < 4; i++) tel_status.mot[i] = motor_us[i];

    // ---- 遥测传输：USB(Type-C,115200) + 数传(UART1,57600)，10Hz ----
    String js = tel_build_json();
    Serial.println(js);
    link_send(js);
  }

  // ---- 9. 黑匣子节拍 + WiFi 看板服务 ----
  logger_tick(armed);   // 飞行中只存 RAM 不落盘，防 Flash 写阻塞 1kHz 环
  telemetry_poll();

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
