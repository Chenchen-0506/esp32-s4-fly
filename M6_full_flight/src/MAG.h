/*
 * ============================================================================
 * 罗盘功能模块 —— BN-880 内置磁力计（QMC5883L / HMC5883 自动识别）
 * ----------------------------------------------------------------------------
 * 来源：M5_gps_test 已验证代码，抽成模块。
 * 引脚：I2C（SDA=8 SCL=9，与 BMP280 共用总线）
 * 接口：
 *   bool  mag_begin();         初始化，成功返回 true
 *   void  mag_read(float &hdg);  读取航向（度 0~360，未校准）
 * 说明：航向为未校准值。正式返航前必须做磁校准：先测出 X/Y 偏移量，
 *       把校准后的 heading 偏移写入本文件 MAG_DECL/MAG_X_OFF/MAG_Y_OFF。
 * ============================================================================
 */
#ifndef MAG_H
#define MAG_H

#include <Arduino.h>

// 校准参数：水平旋转一周记录 X/Y 的最大最小值，填入后启用
#define MAG_X_OFF  0.0f    // (Xmax+Xmin)/2 取负
#define MAG_Y_OFF  0.0f    // (Ymax+Ymin)/2 取负
#define MAG_DECL   0.0f    // 磁偏角（桂林约 -2°~-3°，可查当地值）

bool mag_begin();
void mag_read(float &headingDeg);

#endif
