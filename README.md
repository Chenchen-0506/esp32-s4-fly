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
    ├── MAG.h/.cpp            # BN-880 内置罗盘（航向读取 + 自动磁校准 NVS）
    ├── RTL.h/.cpp            # 返航状态机（手动/失控自动返航）
    ├── SBUS.h/.cpp           # SBUS 接收解析 + 通道
    ├── PID.h/.cpp            # 串级 PID 控制器
    ├── Motors.h/.cpp         # 4 路电机 PWM 输出
    ├── ALTHOLD.h/.cpp        # 定高 + 悬停（CH7）
    ├── POSHOLD.h/.cpp        # 位置保持（GPS 悬停，参考 ArduPilot Loiter）
    ├── VERTEST.h/.cpp        # 垂直融合（1D 互补：加速度预测 1kHz + 气压修正 10Hz）
    ├── LOGGER.h/.cpp         # 黑匣子（RAM 环形 512 行 + LittleFS 落盘，串口 D 导出）
    └── TELEMETRY.h/.cpp      # WiFi 看板（softAP 热点，手机浏览器实时查看/下载）
```

## 功能说明（M7 起，v0.12 最新）

### 解锁 / 上锁（CH5 开关）

- **解锁**：油门最低（<1050）+ CH5 拨高保持 2 秒；解锁瞬间重新记录地面气压并归零垂直融合（防气压漂移导致高度基准偏）
- **home 记录**：起飞后 GPS 定位**连续 1 分钟**自动记录当前位置为 home（解锁瞬间不记录）
- **上锁**：CH5 拨低，电机立即停转

### 返航（CH6 手动 / 失控自动，120m 巡航）

- **CH6 拨高**：进入手动返航（需 GPS 定位 + home 已记录，否则拒绝并提示）
- **遥控丢失**（SBUS >1 秒无帧）：自动返航；**无 GPS 定位时直接强制上锁**（安全兜底）
- 返航流程：爬升到 **120m**（RTL_ALT）→ 机头对准 home 飞行（罗盘航向 + 距离 P 控制）→ 距 home <5m（RTL_NEAR）缓降 → 触地自动上锁
- 手动返航在爬升/飞行阶段可拨低 CH6 退出；降落阶段禁止退出（防悬空）
- 失控触发的返航不可中途退出；**已进入降落段时失控不打断，继续降落（禁止重爬 120m）**
- 返航高度/垂速由**垂直融合**（VERTEST）提供：加速度 1kHz 预测 + 气压 10Hz 修正

### 定高 / 悬停（CH7 开关）

- **CH7 拨高**：进入定高 + 悬停模式（需 GPS 定位）
  - 高度保持：定高 PID（悬停油门自动学习，10 秒持久化到 NVS，下次开机自动加载）
  - 位置保持：位置误差→倾角（死区 1.5m 压制 GPS 噪声，限幅 15°），摇杆=移动悬停目标点
- **CH7 拨低**：退出，恢复纯手动

### 磁校准（返航前必须，全自动）

- 未解锁 + 油门最低 + **CH6 拨高 3 秒** → 进入自动校准模式
- 水平端平机身，慢速旋转整机 2~3 圈 + 8 字，**30 秒自动完成**（或拨低 CH6 提前结束）
- 校准值自动存入 ESP32 NVS（掉电不丢），下次开机自动加载
- 磁偏角查表：默认 `MAG_DECL=-2.5°`（桂林），其他地区修改 `src/MAG.h`

### 黑匣子（LOGGER，v0.11）

- 飞行数据实时写入 RAM 环形缓冲（512 行），每 60 秒落盘 Flash（LittleFS `/log.txt`），**上锁/返航完成立即落盘**（断电最多丢 1 分钟）
- 串口监视器输入字母 **`D`** → 导出全部日志
- 首次使用 LittleFS 自动格式化分区（正常现象）

### WiFi 实时看板（TELEMETRY，v0.11）

- 开机自动开热点 **`S4Fly-Telemetry`**（密码 `12345678`）
- 手机连热点 → 浏览器打开 **`http://192.168.4.1`**：实时滚动姿态/高度/GPS/返航状态，右上角可下载完整记录
- 完全无需 USB，适合外场悬停/试飞观测

### 参数调节

- 返航参数集中在 `src/RTL.h`；定高/悬停参数在 `src/ALTHOLD.h` / `src/POSHOLD.h`；垂直融合在 `src/VERTEST.h`；内环 PID 在 `src/PID.h`，外环角度增益在主程序开头 `ANGLE_P_*`
- 首次试飞建议保守值，按串口/黑匣子数据逐步微调

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

- 每个里程碑完成并验证后，提交一次并打 `vX.Y-<名字>` 标签：
  - `v0.0-baseline`：M1~M6 初始代码快照
  - `v0.1-M6-modular`：M6 模块化重构完成
  - `v0.2-readme`：README 软件依赖章节
  - `v0.3-bmp280`：BMP280 模块接入
  - `v0.4-M7`：M7 返航功能（GPS/罗盘/RTL + CH5/CH6）
  - `v0.5-home-120m`：home 定位改为起飞后 GPS 连续 1 分钟记录；返航高度 120m、距 home 5m 降落
  - `v0.6-safety-fix`：失控无 GPS 强制上锁等安全修复
  - `v0.7-mag-autocal`：罗盘自动磁校准 + NVS 持久化
  - `v0.8-althold-poshold`：CH7 定高 + 悬停（悬停油门学习）
  - `v0.9-hover-learn`：悬停油门学习优化
  - `v0.10-vertfusion-descend`：垂直融合（VERTEST）+ 返航下降段水平位置修正
  - `v0.11-blackbox-wifi`：黑匣子（LOGGER）+ WiFi 看板（TELEMETRY）
  - `v0.12-selfcheck-fix`：代码自检修复（加速度量程 bug / 失控降落保护 / 解锁重定高 / 磁校准节流）
- 回档命令：`git checkout v0.1-M6-modular`
- 提交信息规范：`vX.Y <阶段>：<做了什么>`，中文描述

## 安全铁律

- 任何电调/电机通电前**必须拆桨**
- 测试顺序：拆桨桌面 → 测试架 → 系留 → 自由首飞
- 接电池时拔 USB，USB 调试时不接电池
- 返航/失控保护属于高风险功能：首次试飞请在开阔场地、无 GPS 拒止环境进行，且必须先完成罗盘磁校准
