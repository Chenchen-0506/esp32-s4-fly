/*
 * ============================================================================
 * ICM20948 功能模块 —— 九轴 IMU 驱动 + Mahony 姿态解算（SPI）
 * ----------------------------------------------------------------------------
 * 来源：M1_attitude_sensor / M6_full_flight 已验证代码，原样提取
 * 引脚：SCK=12  MISO=13  MOSI=11  CS=10（3V3）
 * 接口：
 *   uint8_t icm_init();                返回 WHO_AM_I（期望 0xEA）
 *   void icm_read_motion();            读一次 accel+gyro 到 acc_g[]/gyr_dps[]
 *   void imu_calibrate(samples=500);   静止零偏校准 + 四元数初值
 *   void mahony_update6(ax,ay,az,gx,gy,gz,dt);  1kHz 姿态更新
 *   void quat_to_euler_deg();          四元数 -> roll/pitch/yaw(度)
 * 全局输出：acc_g[3], gyr_dps[3], roll_deg, pitch_deg, yaw_deg, gyr_bias[3]
 * ============================================================================
 */
#ifndef ICM20948_H
#define ICM20948_H

#include <Arduino.h>
#include <SPI.h>

/* ---- 引脚 ---- */
#define ICM_SCK       12
#define ICM_MISO      13
#define ICM_MOSI      11
#define ICM_CS        10
#define ICM_SPI_HZ    8000000UL

/* ---- 寄存器（Bank0）---- */
#define REG_WHO_AM_I          0x00
#define REG_USER_CTRL         0x03
#define REG_PWR_MGMT_1        0x06
#define REG_PWR_MGMT_2        0x07
#define REG_INT_PIN_CFG       0x0F
#define REG_ACCEL_XOUT_H      0x2D
#define REG_BANK_SEL          0x7F

/* ---- 寄存器（Bank2）---- */
#define B2_GYRO_SMPLRT_DIV    0x00
#define B2_GYRO_CONFIG_1      0x01
#define B2_GYRO_CONFIG_2      0x02
#define B2_ACCEL_SMPLRT_DIV_2 0x11
#define B2_ACCEL_CONFIG       0x14

/* ---- 量程灵敏度 ----
 * ACCEL_CONFIG(0x14)=0x1D → ACCEL_FS_SEL=11 → ±16g → 2048 LSB/g
 * GYRO_CONFIG_1(0x01)=0x1F → GYRO_FS_SEL=11 → ±2000dps → 16.4 LSB/dps
 * （原 ACC_LSB_G=4096 是 ±8g 的，与实际 ±16g 配置不匹配，
 *   会让 VERTEST 垂直加速度缩水一半，静止 acc_z≈0.5g，定高垂速错乱） */
#define GYRO_LSB_DPS          16.384f
#define ACC_LSB_G             2048.0f
#define ICM_WHOAMI_EXPECT     0xEA

/* ---- Mahony 参数 ---- */
#define Kp_MAHONY             1.2f
#define Ki_MAHONY             0.05f
#define RAD2DEG               57.2957795f
#define DEG2RAD               0.01745329f

/* ---- 全局输出（主程序可读）---- */
extern float acc_g[3];     // 加速度 g
extern float gyr_dps[3];   // 陀螺 度/秒
extern float roll_deg, pitch_deg, yaw_deg;
extern float gyr_bias[3];  // 零偏
extern float imu_q[4];      // 姿态四元数 (w,x,y,z)，供 VERTEST 姿态补偿
extern float imu_q[4];      // 姿态四元数 (w,x,y,z)，供 VERTEST 姿态补偿

/* ---- 接口 ---- */
uint8_t icm_init();
void icm_read_motion();
void imu_calibrate(uint16_t samples = 500);
void mahony_update6(float ax, float ay, float az,
                    float gx, float gy, float gz, float dt);
void quat_to_euler_deg();

#endif
