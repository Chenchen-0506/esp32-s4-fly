/*
 * ============================================================================
 * BMP280 气压计模块（I2C，寄存器级驱动，零第三方库）
 * ----------------------------------------------------------------------------
 * 引脚：SDA=8  SCL=9（与主文件接线注释一致）
 * 功能：初始化自检（自动找 0x76/0x77）、读温度/气压、气压高度换算、
 *       相对起飞点高度（起飞时记录参考气压）。
 *
 * 对外接口：
 *   bmp_begin()                初始化，成功返回 true
 *   bmp_read(tempC, pressPa)   读一次温度(℃)/气压(Pa)
 *   bmp_alt_m(pressPa)         由气压算海拔（m，标准大气模型）
 *   bmp_set_ground(pressPa)    记录起飞参考气压（建议在解锁前调用）
 *   bmp_rel_alt_m(pressPa)     相对起飞点高度（m，正=高于起飞点）
 *
 * 高度公式：alt = 44330 * (1 - (P/101325)^(1/5.255))
 * ============================================================================
 */
#ifndef BMP280_H
#define BMP280_H

#include <Arduino.h>
#include <Wire.h>

// I2C 引脚（ESP32-S3：SDA=8 SCL=9）
#define BMP_I2C_SDA  8
#define BMP_I2C_SCL  9

// 海平面标准气压（Pa），用于绝对海拔换算
#define BMP_SEA_LEVEL_PA  101325.0f

// 供主程序使用的全局参考气压（Pa），由 bmp_set_ground 写入
extern float bmp_ground_pa;

bool  bmp_begin();                        // 初始化，成功返回 true
void  bmp_read(float &tempC, float &pressPa); // 读温度(℃)/气压(Pa)，失败时 pressPa=0
float bmp_alt_m(float pressPa);           // 气压 -> 海拔（m）
void  bmp_set_ground(float pressPa);      // 记录起飞参考气压
float bmp_rel_alt_m(float pressPa);       // 相对起飞点高度（m）

#endif
