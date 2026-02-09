# OpenEarable 2 Firmware — Project Structure & Architecture

## 项目结构概览

- 根目录：
  - `CMakeLists.txt`, `Kconfig`, `west.yml`, `README.md`, `LICENSE` 等工程配置文件
  - `prj.conf`, `prj_release.conf`, `prj_fota.conf`：不同构建配置（调试/发布/FOTA）
  - `pm_static_fota.yml`、`sysbuild_fota.conf`：FOTA/分区相关配置
  - `cdc_acm_uart0_console.overlay`：USB CDC ACM串口重定向overlay
- `boards/`：板级配置、DTS、overlay文件
- `include/`：核心头文件（`openearable_common.h`, `zbus_common.h`）
- `src/`：主要功能模块
  - `audio/`、`bluetooth/`、`buttons/`、`Battery/`、`SensorManager/`、`SD_Card/`、`drivers/`、`modules/`、`utils/`、`Wire/`
- `unicast_server/`：示例应用（main.cpp、CMakeLists.txt、overlay）
- `doc/`：文档（架构、构建、FOTA、用户接口等）
- `tools/`：脚本（烧录、构建、串口工具等）

## 架构说明

- 平台：nRF5340，基于 Zephyr RTOS，NCS v3.0.1
- 双核分工：CPUNET 运行 SoftDevice Controller，CPUAPP 运行 Bluetooth Host + 应用
- 音频角色：Gateway/client 与 Headset/server，代码通过 Kconfig/overlay 配置切换
- 消息总线：模块间通过 Zephyr `zbus` 传递消息（见 `include/zbus_common.h`）
- 主要模块：
  - 音频数据路径（LC3 编码/解码、同步补偿）
  - 传感器管理（IMU、燃料计、充电管理、LED 控制等）
  - BLE GATT 服务（如 meow_ctrl_service，支持串口/蓝牙控制 IMU 和电池查询）
  - SD 卡日志、按钮、状态指示、驱动等

## 编译与调试经验

- 推荐使用 VS Code + nRF Connect 扩展，Toolchain/SDK 固定为 3.0.1
- 构建命令示例：
  - 调试：`west build -b openearable_v2/nrf5340/cpuapp --pristine -- -DBOARD_ROOT=E:\New_FirmIMU`
  - 发布：`west build -b openearable_v2/nrf5340/cpuapp --pristine -- -DFILE_SUFFIX=release`
  - FOTA：`west build -b openearable_v2/nrf5340/cpuapp --pristine -- -DFILE_SUFFIX=fota`
- 烧录脚本：
  - 非FOTA：`tools/flash/flash.sh`
  - FOTA：`tools/flash/flash_fota.sh`
  - UICR 恢复：`tools/flash/recover.sh --snr <SNR>`
- USB CDC ACM 串口：
  - 串口输出已重定向到 USB CDC ACM（虚拟 COM），波特率 115200
  - 插入 USB 后，打开串口即可看到调试信息和控制入口
- BLE 调试：
  - GATT 服务支持 IMU 数据和电池查询，需订阅 CCC 后才能收到 Notify
  - MTU/包长需注意（IMU batch size 需适配 BLE MTU）
- I2C 设备扫描/初始化：
  - BMX160 初始化需延时，1.8V 电源需先开启
  - I2C 地址、引脚需与硬件实际一致（见 netlist/DTS）
- DTS/overlay 注意：
  - UART0 TX/RX 未连接，所有串口需用 USB CDC ACM
  - 确认 overlay 文件已应用（如 cdc_acm_uart0_console.overlay）
- FOTA/分区：
  - FOTA 构建需检查分区配置（pm_static_fota.yml）
  - MCUboot/DFU 需按文档操作，避免 UICR 损坏

## 常见注意事项

- DTS 引脚配置需与硬件实际一致，避免 I2C/UART 冲突
- BMX160、BQ27220、BQ25120a 等 I2C 设备需正确初始化
- BLE Notify 需订阅 CCC，电池特性需支持 Read
- 串口调试建议用 USB CDC ACM，硬件 UART 不可用
- FOTA/DFU 操作需严格按脚本和文档流程，避免设备不可恢复

---

如需详细架构、构建、FOTA、脚本说明，请参考 `doc/firmware_architecture.rst`、`doc/building.rst`、`doc/fota.rst` 及 `tools/flash/README.md`。

---

> 拼写建议：正确为 `architecture.md`（不是 archtecture.md）。