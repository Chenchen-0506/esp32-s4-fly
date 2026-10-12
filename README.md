# ESP32-S3 自研四旋翼无人机（固件仓库）

从零自研四旋翼无人机固件。ESP32-S3 主控，Arduino IDE 开发。

## 硬件（450mm X 型大机）

- 主控：ESP32-S3 DevKitC N16R8（SPI：ICM-20948 / I2C：BMP280 / UART：BN-880 GPS / SBUS：GPIO18）
- 动力：2212 KV920 ×4 + 30A BLHeli 电调 ×4 + 1045 桨 + 4S 3000mAh
- 传感器：ICM-20948 九轴（SPI）+ BMP280 气压计（I2C）+ BN-880 GPS/罗盘（UART）

## 目录结构与功能模块约定

每个里程碑（M1~M7）是一个**独立 Arduino 工程文件夹**，可直接用 Arduino IDE 打开烧录，互不依赖，方便按阶段回档。

```
esp32 S3/
├── esp32_board_check/        # 板子自检（N16R8 真伪 + 心跳）
├── esp32_sensor_check/       # 传感器自检（BMP280 + GPS NMEA）
├── M1_attitude_sensor/       # M1：ICM-20948 姿态解算（Mahony 6 轴）
├── M2_simulation/            # M2：刚体动力学 + 串级 PID 仿真
├── M3_remote_pwm/            # M3 备选：遥控 PWM 解析
├── M3_remote_sbus/           # M3：遥控 SBUS 解析
├── M4_esc_test/              # M4：电调校准 + 单电机测试
├── M5_gps_test/              # M5：GPS 定位测试
└── M6_full_flight/           # M6：完整飞控（主程序 + src/ 功能模块）
```

**功能模块规范（M6 起）**：每个 sketch 文件夹下的 `src/` 子目录按功能拆分，一个功能一个 `.h/.cpp` 对，由主 `.ino` 引用。Arduino IDE 会自动编译 `src/` 下的代码，无需额外配置。

```
M6_full_flight/
├── M6_full_flight.ino        # 主程序：setup/loop、状态机、解锁、混控
└── src/
    ├── ICM20948.h/.cpp       # 九轴 IMU 驱动 + Mahony 姿态 + 零偏校准
    ├── BMP280.h/.cpp         # 气压计驱动 + 相对高度
    ├── SBUS.h/.cpp           # SBUS 接收解析 + 通道
    ├── PID.h/.cpp            # 串级 PID 控制器
    └── Motors.h/.cpp         # 4 路电机 PWM 输出
```

## 阶段回档（Git 规范）

- 每个里程碑完成并验证后，提交一次并打 `vX.Y-Mn` 标签，例如：
  - `v0.0-baseline`：M1~M6 初始代码快照
  - `v0.1-M6-modular`：M6 模块化重构完成
- 回档命令：`git checkout v0.1-M6-modular`
- 提交信息规范：`vX.Y <阶段>：<做了什么>`，中文描述

## 安全铁律

- 任何电调/电机通电前**必须拆桨**
- 测试顺序：拆桨桌面 → 测试架 → 系留 → 自由首飞
- 接电池时拔 USB，USB 调试时不接电池
