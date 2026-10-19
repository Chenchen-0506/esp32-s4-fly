/*
 * ============================================================================
 * 罗盘功能模块 —— BN-880 内置磁力计（QMC5883L / HMC5883 自动识别）
 * ----------------------------------------------------------------------------
 * 引脚：I2C（SDA=8 SCL=9，与 BMP280 共用总线）
 *
 * 【自动磁校准】（无需手记数字/改代码）：
 *   未解锁 + 油门最低 + CH6 拨高 3 秒 → 进入校准模式
 *   水平端平机身，慢速旋转整机 2~3 圈 + 8 字，30 秒自动完成
 *   （或拨低 CH6 提前结束）
 *   校准值自动存入 ESP32 NVS（掉电不丢），下次开机自动加载。
 *
 * 【磁偏角（自动识别，无需改代码）】：
 *   机载内置 NOAA WMM 全球磁偏角查表（MAG_DECL_TABLE.h，10° 网格）。
 *   GPS 定位后自动按经纬度计算并更新 mag_decl（内部节流 10s，
 *   变化 >0.2° 才写 NVS 持久化），去哪飞都不用改。
 *   GPS 未定位时用 NVS 上次保存值；完全没有时用默认 MAG_DECL。
 * ============================================================================
 */
#ifndef MAG_H
#define MAG_H

#include <Arduino.h>

// 磁偏角默认值（仅当从未有过 GPS 定位且 NVS 无值时使用；桂林约 -2.5°）
#define MAG_DECL   -2.5f

// 运行时硬磁偏移（自动校准后由 NVS 加载，mag_read 使用）
extern float mag_x_off;
extern float mag_y_off;

// 运行时磁偏角（真北修正，度）：GPS 定位后自动更新，mag_read 使用
extern float mag_decl;
extern bool  mag_calibrated;   // 已做过磁校准（NVS 持久化）

void mag_load_cal();                  // 开机：从 NVS 加载校准值 + 磁偏角
void mag_cal_start();                 // 开始校准采样（清空极值）
int  mag_cal_step();                  // 采样推进：0=进行中 1=完成（已保存）
void mag_cal_finish();                // 立即用当前极值计算并保存（提前结束用）
void mag_read(float &headingDeg);     // 读取航向（应用偏移 + 磁偏角）
float mag_auto_decl(float lat, float lon);   // 查表计算磁偏角（度，-90~90）
void mag_auto_update(float lat, float lon);  // GPS fix 后周期调用：自动更新并持久化

bool mag_begin();
bool mag_cal_done(void);   // 返回是否已校准

#endif
