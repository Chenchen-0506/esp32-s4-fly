# ESP32-S3 自研四旋翼无人机（固件仓库）

从零自研四旋翼无人机固件。ESP32-S3 主控，Arduino IDE 开发。

## 硬件（450mm X 型大机）

- 主控：ESP32-S3 DevKitC N16R8（SPI：ICM-20948 / I2C：BMP280 + 罗盘 / UART：BN-880 GPS / SBUS：GPIO18）
- 动力：2212 KV920 ×4 + 30A BLHeli 电调 ×4 + 1045 桨 + 4S 3000mAh
- 传感器：ICM-20948 九轴（SPI）+ BMP280 气压计（I2C）+ BN-880 GPS/罗盘（UART+I2C）
- 遥控：FlySky 6 通道（SBUS）：CH5 解锁开关、CH6 返航开关

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
└── M6_full_flight/           # M6/M7：完整飞控（主程序 + src/ 功能模块）
```

**功能模块规范（M6 起）**：每个 sketch 文件夹下的 `src/` 子目录按功能拆分，一个功能一个 `.h/.cpp` 对，由主 `.ino` 引用。Arduino IDE 会自动编译 `src/` 下的代码，无需额外配置。

```
M6_full_flight/
├── M6_full_flight.ino        # 主程序：setup/loop、状态机、解锁、混控、返航
└── src/
    ├── ICM20948.h/.cpp       # 九轴 IMU 驱动（SPI）+ Mahony 姿态 + 零偏校准
    ├── BMP280.h/.cpp         # 气压计驱动（I2C）+ 相对高度
    ├── GPS.h/.cpp            # BN-880 NMEA 解析（定位/卫星/经纬度）
    ├── MAG.h/.cpp            # BN-880 内置罗盘（航向读取）
    ├── RTL.h/.cpp            # 返航状态机（手动/失控自动返航）
    ├── SBUS.h/.cpp           # SBUS 接收解析 + 通道
    ├── PID.h/.cpp            # 串级 PID 控制器
    └── Motors.h/.cpp         # 4 路电机 PWM 输出
```

## 功能说明（M7）

### 解锁 / 上锁（CH5 开关）

- **解锁**：油门最低（<1050）+ CH5 拨高保持 2 秒；解锁瞬间记录 GPS 起飞点（home）
- **上锁**：CH5 拨低，电机立即停转

### 返航（CH6 手动 / 失控自动）

- **CH6 拨高**：进入手动返航（需 GPS 定位 + home 已记录，否则拒绝并提示）
- **遥控丢失**（SBUS >1 秒无帧）：自动返航；**无 GPS 定位时直接强制上锁**（安全兜底）
- 返航流程：爬升到 15m（RTL_ALT）→ 机头对准 home 飞行（罗盘航向 + 距离 P 控制）→ 距 home <12m（RTL_NEAR）缓降 → 触地自动上锁
- 手动返航在爬升/飞行阶段可拨低 CH6 退出；降落阶段禁止退出（防悬空）
- 失控触发的返航不可中途退出，只能完成降落或 CH5 上锁

### 参数调节

返航参数集中在 `src/RTL.h`：巡航高度、缓降油门、最大倾斜角、P 增益等，首次试飞建议保守值。

### 磁校准（返航前必须）

罗盘未校准时返航航向会有偏差。校准方法见 `src/MAG.h` 注释：水平旋转一周，记录 X/Y 最大值最小值，填入 `MAG_X_OFF/MAG_Y_OFF`，并查本地磁偏角填入 `MAG_DECL`。

## 软件依赖

### 开发环境

| 软件 | 版本 | 用途 |
| --- | --- | --- |
| Arduino IDE | 2.x（实测 2.3.x） | 主开发环境，打开各工程文件夹直接编译烧录 |
| ESP32 核心（esp32 by Espressif） | 3.3.x（实测 3.3.11） | Arduino IDE 库管理器安装，提供 `esp32:esp32:esp32s3` 板型 |
| arduino-cli | 1.5.x | 可选，命令行编译/验证用（Arduino IDE 内置） |

### 板型选择

- **开发板**：`ESP32S3 Dev Module`（FQBN：`esp32:esp32:esp32s3`）
- **Flash Size**：16MB（N16）
- **PSRAM**：OPI PSRAM（R8）

### 第三方库

**零第三方依赖**。全部代码仅使用 ESP32 核心自带的库：

- `<Arduino.h>`、`<SPI.h>`、`<Wire.h>`、`<math.h>`

IMU（ICM-20948）、气压计（BMP280）、GPS（NMEA）、罗盘（QMC5883L/HMC5883）、返航（RTL）、SBUS、PID、电机驱动全部为自研代码，位于 `M6_full_flight/src/`，clone 仓库后无需安装任何额外库即可编译。

### 命令行编译（可选）

```bash
# 以 M6 为例（arduino-cli 需已配置 esp32 核心）
arduino-cli compile --fqbn esp32:esp32:esp32s3 "M6_full_flight"
```

## 阶段回档（Git 规范）

- 每个里程碑完成并验证后，提交一次并打 `vX.Y-Mn` 标签，例如：
  - `v0.0-baseline`：M1~M6 初始代码快照
  - `v0.1-M6-modular`：M6 模块化重构完成
  - `v0.2-readme`：README 软件依赖章节
  - `v0.3-bmp280`：BMP280 模块接入
  - `v0.4-M7`：M7 返航功能（GPS/罗盘/RTL + CH5/CH6）
- 回档命令：`git checkout v0.1-M6-modular`
- 提交信息规范：`vX.Y <阶段>：<做了什么>`，中文描述

## 安全铁律

- 任何电调/电机通电前**必须拆桨**
- 测试顺序：拆桨桌面 → 测试架 → 系留 → 自由首飞
- 接电池时拔 USB，USB 调试时不接电池
- 返航/失控保护属于高风险功能：首次试飞请在开阔场地、无 GPS 拒止环境进行，且必须先完成罗盘磁校准
