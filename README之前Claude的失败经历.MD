# Meow Sense Tag 固件

基于 nRF5340 (MDBT53-V-DB-40 模块) 的 BLE 感应标签固件，硬件参考 [OpenEarable v2](https://github.com/OpenEarable/open-earable-2) 设计。

> **⚠️ 警告：不正确的烧录操作会导致 nRF5340 永久变砖（APPROTECT 锁死 SWD）。请务必通读本文档，
> 特别是 [致命教训](#致命教训血泪总结) 和 [烧录](#烧录极其重要) 章节。**

## 目录

- [致命教训（血泪总结）](#致命教训血泪总结)
- [与 OpenEarable 的关键差异](#与-openearable-的关键差异)
- [硬件概览](#硬件概览)
- [开发环境](#开发环境)
- [项目结构](#项目结构)
- [构建](#构建)
- [新模块出厂固件备份（首次烧录前必做）](#新模块出厂固件备份首次烧录前必做)
- [烧录（极其重要）](#烧录极其重要)
- [救砖](#救砖)
- [APPROTECT 深度解析](#approtect-深度解析)
- [固件初始化流程](#固件初始化流程)
- [BQ25120A PMIC 驱动注意事项](#bq25120a-pmic-驱动注意事项)
- [Device Tree Overlay 配置](#device-tree-overlay-配置)
- [Kconfig 配置说明](#kconfig-配置说明)
- [BLE 功能](#ble-功能)
- [完整踩坑记录（12+ 个 Bug）](#完整踩坑记录12-个-bug)
- [参考资料](#参考资料)

---

## 致命教训（血泪总结）

> **以下内容总结了真实开发中导致多块硬件板子永久变砖的经历。**
> **每一条都是用"砖"换来的。**

### 🚨 第一条铁律：绝对不要从固件写 UICR

```
❌ 绝对不要在固件中写 UICR（NRF_UICR_S->xxx = ...）
```

**原因：**
- UICR 是 Flash 背写的，写入需要 NVMC 操作（先擦后写）
- 如果写入过程中发生复位（BQ25120A /RESET 抖动、电源不稳），UICR 会损坏
- 损坏的 UICR 可能把 APPROTECT 设为非法值 → SWD 永久锁死
- **OpenEarable 在固件中绝不碰 UICR**，它们仅通过 `nrfjprog --memwr` 在烧录时写入

**正确做法：** UICR 写入全部交给烧录脚本（`tools/flash.ps1`），固件代码只读不写。

### 🚨 第二条铁律：UICR 备份可能是 "毒药"

```
❌ 不要盲目恢复 UICR 备份到 chip erase 后的芯片
```

**原因（这是导致所有板子变砖的根本原因）：**
1. 全新芯片的 UICR.APPROTECT = `0xFFFFFFFF`（**锁定！** nRF5340 默认值）
2. `nrfjprog --readuicr` 备份了这个 "锁定" 状态的 UICR
3. `--chiperase` 擦除整个 Flash + UICR
4. 写入新固件
5. `nrfjprog --program uicr_backup.hex` 恢复 UICR → **APPROTECT 被重新锁定！**
6. `nrfjprog --reset` → 芯片复位 → APPROTECT 生效 → SWD 死锁 → 变砖

**时序窗口极小：** 恢复 UICR 锁定值后，必须在复位前写入解锁值。如果这一步失败（SWD 不稳定、BQ25120A /RESET 干扰），就永久变砖。

**正确做法：** 恢复 UICR 后，**无条件** 覆写 APPROTECT 和 SECUREAPPROTECT 为 `0x50FA50FA`（当前 `flash.ps1` 已修复）。

> **⚠️ 重要补充（来自出厂固件分析）：**
> MDBT53-1M 出厂预装了 Raytac BLE 测试固件（手机可搜到 "MDBT53-1M"，连接需输入 PIN）。
> 但 nRF Connect Programmer 的 "Read" 功能**只读取 Flash，不读取 UICR**！
> 实测备份文件 `Original_nRF_Connect_Programmer_1770540961762.hex` 中**不包含 UICR 区域**
> （0x00FF8000），因此无法从该备份确认出厂 APPROTECT 值。
> **必须使用 `nrfjprog --readuicr` 单独备份 UICR！** 详见 [新模块出厂固件备份](#新模块出厂固件备份首次烧录前必做)。

### 🚨 第三条铁律：固件里的 CD 引脚不能搞错

```
❌ CD = P1.05 是错误的！（P1.05 是按钮/ON+BTN 信号）
✅ CD = P0.17 才是 BQ25120A 的真正 CD 引脚
```

**原因：**
- MDBT53 引脚 26 (CC_#CD) = **P0.17** → BQ25120A CD 引脚 (E2)
- MDBT53 引脚 22 (ON+BTN) = **P1.05** → 按钮信号（通过 Q2 MOSFET）
- 如果固件拉高 P1.05 而不是 P0.17，BQ25120A 在电池模式下不会被唤醒
- I2C 不通 → LDO 3.3V 不输出 → KTD2026 无电 → LED 不亮

### 🚨 第四条铁律：Network Core 必须启用

```
❌ 不设置 CONFIG_SOC_NRF53_CPUNET_ENABLE=y → bt_enable() 永久阻塞
```

**原因：**
- nRF5340 的网络核心（CPUNET）默认处于 FORCEOFF 状态
- 没有 `CONFIG_SOC_NRF53_CPUNET_ENABLE=y`，Zephyr 不会释放 FORCEOFF
- `bt_enable()` 等待网络核心的 IPC 响应，但网络核心没在运行 → 永久卡死
- **被卡死意味着固件不会输出心跳日志、LED 不闪烁、看起来像变砖了（实际只是阻塞）**

### 🚨 第五条铁律：两个核心都必须烧录

```
❌ 只烧 merged.hex（APP）而不烧 merged_CPUNET.hex（NET）→ BLE 完全不工作
```

**原因：**
- `merged.hex` = Application Core 固件
- `merged_CPUNET.hex` = Network Core 固件（hci_ipc BLE 控制器）
- 没有网络核心固件，蓝牙协议栈没有底层传输 → `bt_enable()` 永久阻塞

### 🚨 第六条铁律：BQ25120A 的 /RESET 直连 nRF5340 RESET

BQ25120A 的 /RESET 引脚（D3）直接连到 nRF5340 的 RESETN 引脚（MDBT53 引脚 6）。
这意味着 **PMIC 状态直接控制 MCU 的复位**：

| BQ25120A 状态 | /RESET 输出 | nRF5340 |
|------|------|------|
| Active (CD=HIGH 或 USB 接入) | HIGH | 正常运行 |
| Ship Mode / Hi-Z (CD=LOW, 无 USB) | LOW | **持续复位！** |
| 故障 (TS 故障、UVLO 等) | LOW | **持续复位！** |

**风险：** 如果 BQ25120A 处于 Hi-Z 模式且没有 USB 供电，nRF5340 根本无法启动。即使 J-Link 供电了 MCU，BQ25120A 仍然会通过 /RESET 拉低 RESETN，导致 SWD 不稳定。

---

## 与 OpenEarable 的关键差异

通过深入研究 [OpenEarable/open-earable-2](https://github.com/OpenEarable/open-earable-2) 代码，以下是关键区别和教训：

### OpenEarable 的做法（安全、经过验证）

| 方面 | OpenEarable 做法 | 我们早期的错误做法 |
|------|------|------|
| **APPROTECT** | **固件中绝不碰 UICR/CTRLAP** | 在 SYS_INIT 中写 UICR（危险！） |
| **CONFIG_NRF_APPROTECT** | 不设 USER_HANDLING | 设了 USER_HANDLING |
| **CD 引脚** | P0.17（从 DTS：`cd-gpios`） | P1.05（搞混了 ON+BTN） |
| **烧录时钟** | 8000 kHz | 1000 kHz（太慢！） |
| **烧录顺序** | NET 先, APP 后 | 一致 |
| **UICR 恢复** | 备份→烧录→恢复→**写自定义 UICR** | 备份→烧录→恢复→**没写 APPROTECT 解锁！** |
| **Recovery** | NET 先, APP 后, clockspeed 8000 | 顺序不固定, clockspeed 1000 |
| **电源管理** | board_init.c + mcuboot_hook.c 双重保障 | 仅 SYS_INIT |
| **3.3V 负载开关** | 有 P0.14 控制 | 早期遗漏了 P0.14 |
| **寄存器 0x06** | 不存在（正确跳过） | 写入了 0x06（导致 PMIC 异常） |

### OpenEarable 烧录流程详解

```bash
# OpenEarable tools/flash/flash.sh 完整流程：
CLOCKSPEED=8000

# 1. 备份 UICR（保留序列号、通道、硬件版本等自定义数据）
nrfjprog --readuicr ./tools/flash/uicr_backup.hex -f NRF53 --snr $SNR --clockspeed $CLOCKSPEED

# 2. 先烧 NET core（必须先烧！APP core 依赖 NET core 的 BLE 控制器）
nrfjprog --program ./build/merged_CPUNET.hex --chiperase --verify -f NRF53 \
    --coprocessor CP_NETWORK --snr $SNR --clockspeed $CLOCKSPEED

# 3. 再烧 APP core
nrfjprog --program ./build/merged.hex --chiperase --verify -f NRF53 \
    --coprocessor CP_APPLICATION --snr $SNR --clockspeed $CLOCKSPEED

# 4. 恢复 UICR（含序列号等自定义数据）
nrfjprog --program ./tools/flash/uicr_backup.hex -f NRF53 --snr $SNR --clockspeed $CLOCKSPEED --verify

# 5. 写自定义 UICR 数据（通道、独立模式、硬件版本）
nrfjprog --memwr 0x00FF80F4 --val 0          # 通道
nrfjprog --memwr 0x00FF80FC --val 0          # 独立模式
nrfjprog --memwr 0x00FF8100 --val $HW_VALUE  # 硬件版本

# 6. 复位
nrfjprog --reset -f NRF53 --snr $SNR --clockspeed $CLOCKSPEED
```

注意：OpenEarable **不需要** 写 APPROTECT 解锁，因为它们的芯片 UICR 中本来就没锁。但如果使用全新芯片或被 chip erase 过的芯片，UICR.APPROTECT 会是 `0xFFFFFFFF`（锁定值），恢复这样的备份就会变砖。

### OpenEarable 恢复流程

```bash
# OpenEarable tools/flash/recover.sh：
# 始终 NET 先, APP 后, clockspeed 8000
nrfjprog --recover -f NRF53 --coprocessor CP_NETWORK --snr $SNR --clockspeed 8000
nrfjprog --recover -f NRF53 --coprocessor CP_APPLICATION --snr $SNR --clockspeed 8000
```

### OpenEarable 的 board_init.c（电源管理参考）

```c
// MCUboot 阶段（PRE_KERNEL_2, priority 80）— 硬编码引脚：
gpio_pin_configure(gpio1, 11, GPIO_OUTPUT_ACTIVE);  // P1.11 = 1.8V 负载开关
gpio_pin_configure(gpio0, 14, GPIO_OUTPUT_ACTIVE);  // P0.14 = 3.3V 负载开关

// 正常启动阶段（POST_KERNEL, priority 80）— 通过 DT PM_DEVICE 管理：
// load_switch = P1.11 (1.8V), 延迟 250µs
// load_switch_3_3 = P0.14 (3.3V LDO gate), 延迟 500µs
// load_switch_sd = SD 卡供电
```

### OpenEarable 的网络核心释放

```c
// boards/teco/openearable_v2/openearable_v2_nrf5340_cpunet_reset.c
static int remoteproc_mgr_boot(const struct device *dev) {
    // SPU 配置：允许 NET core 访问 Secure 域
    NRF_SPU->EXTDOMAIN[0].PERM = 1 << 4;
    // 释放网络核心！
    NRF_RESET->NETWORK.FORCEOFF = RESET_NETWORK_FORCEOFF_FORCEOFF_Release;
    return 0;
}
SYS_INIT(remoteproc_mgr_boot, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);
```

---

## 硬件概览

| 器件 | 型号 | I2C 地址 | 说明 |
|------|------|----------|------|
| MCU | nRF5340 (MDBT53-V-DB-40) | — | 双核：Application (128MHz) + Network (64MHz) |
| PMIC | BQ25120A | 0x6A | 充电管理 + LDO 3.3V 输出 + /RESET 控制 |
| 电量计 | BQ27220 | 0x55 | 电池电压 / 电流 / SOC |
| RGB LED | KTD2026 | 0x30 | 3 通道 LED 驱动 |

### 关键引脚（已验证，与原理图一致）

| 引脚 | 功能 | 说明 |
|------|------|------|
| **P0.17** | **BQ25120A CD (真正的 CD)** | **MDBT53 pin 26 (CC_#CD) → BQ25120A E2 引脚。SYS_INIT 必须拉高** |
| P0.14 | 3.3V 负载开关 (LSCTRL) | 使能 BQ25120A LDO 到 3.3V 外设的供电通路 |
| P0.21 | I2C1 SDA | 连接 KTD2026 / BQ25120A / BQ27220 |
| P0.24 | I2C1 SCL | 同上 |
| P1.05 | ON+BTN（按钮信号） | **MDBT53 pin 22，通过 Q2 MOSFET → 不是 CD 引脚！** |
| P1.11 | 1.8V 负载开关使能 | 拉高使能 1.8V 电源轨 |
| BQ25120A /RESET (D3) | nRF5340 RESETN (MDBT53 pin 6) | **PMIC 复位输出直连 MCU 复位！CD=LOW → /RESET=LOW → MCU 持续复位** |

> **⚠️ P0.17 vs P1.05 是导致大量问题的根源。原理图上 MDBT53 pin 26 标注为 CC_#CD，
> 对应 P0.17；pin 22 标注为 ON+BTN，对应 P1.05。两者功能完全不同。**

### 电源架构

```
电池 ──→ BQ25120A ──→ SYS (nRF5340 VDDH/VDD)
              │
              ├──→ LDO 输出 3.3V (寄存器 0x07 使能) ──→ [P0.14 负载开关] ──→ KTD2026, BQ27220, 传感器
              │
              └──→ /RESET (D3) ──→ nRF5340 RESETN (MDBT53 pin 6)
                    │
                    ├─ CD=HIGH 或 USB PG=active → /RESET=HIGH → MCU 运行
                    └─ CD=LOW 且 无 USB            → /RESET=LOW  → MCU 持续复位!
```

**核心依赖链：**
```
P0.17 (CD) = HIGH
  → BQ25120A 退出 Hi-Z/Ship 模式
    → /RESET 释放为 HIGH
      → nRF5340 可以正常启动
        → I2C 可用
          → 写寄存器 0x07 使能 LDO 3.3V
            → P0.14 (LSCTRL) = HIGH 使能负载开关
              → 3.3V 送达 KTD2026/BQ27220
                → LED 亮起, 传感器可用
```

**如果链条中任何一环断裂，后续全部不工作。**

---

## 开发环境

| 工具 | 版本 | 说明 |
|------|------|------|
| nRF Connect SDK (NCS) | v3.0.1 | 含 Zephyr 4.0.99 |
| 工具链路径 | `C:\ncs\toolchains\0b393f9e1b\` | Windows |
| CMake | 3.21.0 | NCS 自带 |
| nrfjprog | 必须安装 | 烧录脚本依赖（`nrfjprog --version` 验证） |
| J-Link | V8.94+ | 外部 J-Link EDU Mini 或同等 |
| JLink.exe | `C:\Program Files\SEGGER\JLink_V894\JLink.exe` | recover_and_flash.ps1 fallback 需要 |

### 环境配置

```powershell
# 确保 NCS 环境已激活
# 方式1: 在 nRF Connect for Desktop → Toolchain Manager 打开终端
# 方式2: 手动设置
$env:ZEPHYR_BASE = "C:\ncs\v3.0.1\zephyr"
```

---

## 项目结构

```
peripheral_nus/
├── src/
│   └── main.c                          # 主固件（所有驱动和 BLE 逻辑）
├── boards/
│   ├── nrf5340dk_nrf5340_cpuapp.conf   # Board Kconfig（CPUNET_ENABLE、RTT 等）
│   └── nrf5340dk_nrf5340_cpuapp.overlay # DTS Overlay（I2C、USB CDC）
├── tools/
│   ├── flash.ps1                        # ★ 双核安全烧录脚本（UICR 备份/恢复 + APPROTECT 解锁）
│   ├── recover.ps1                      # 救砖脚本（nrfjprog --recover + 重试）
│   ├── recover_and_flash.ps1            # 强力恢复+烧录（5次重试 + J-Link Commander fallback）
│   ├── do_all.ps1                       # 一键恢复+烧录（输出到日志文件）
│   ├── build.ps1                        # 构建脚本
│   └── jlink_cmds.txt                   # J-Link Commander 命令参考
├── prj.conf                             # 项目 Kconfig
├── sysbuild.conf                        # Sysbuild 配置（网络核心 hci_ipc）
└── CMakeLists.txt                       # 构建配置
```

---

## 构建

```powershell
cd e:\peripheral_nus

# 全量构建（首次或修改 Kconfig / DTS 后）
west build -b nrf5340dk/nrf5340/cpuapp --sysbuild -p

# 增量构建
west build
```

构建产物：

| 文件 | 路径 | 说明 |
|------|------|------|
| `merged.hex` | `build/merged.hex` | Application Core 固件 |
| `merged_CPUNET.hex` | `build/merged_CPUNET.hex` | Network Core 固件（hci_ipc BLE 控制器） |

> **⚠️ 两个 hex 文件都必须烧录！只烧 `merged.hex` 会导致 `bt_enable()` 永久阻塞。**

### 构建前检查清单

- [ ] `sysbuild.conf` 包含 `SB_CONFIG_NETCORE_HCI_IPC=y`（否则不生成 `merged_CPUNET.hex`）
- [ ] `boards/nrf5340dk_nrf5340_cpuapp.conf` 包含 `CONFIG_SOC_NRF53_CPUNET_ENABLE=y`（否则 bt_enable 卡死）
- [ ] 使用 `--sysbuild` 参数（否则不构建网络核心）
- [ ] 构建后确认 `build/merged_CPUNET.hex` 存在（~422KB）

---

## 新模块出厂固件备份（首次烧录前必做）

> **🚨 每一块新的 MDBT53 模块，在第一次烧录自定义固件之前，必须完成以下备份操作！**
> **一旦执行 `--chiperase`，出厂固件将永久丢失，无法恢复。**

### MDBT53-1M 出厂状态

Raytac MDBT53-1M 模块出厂时**预装了 BLE 测试固件**：
- 手机蓝牙可以搜索到名为 **"MDBT53-1M"** 的设备
- 连接时需要输入 PIN 码
- 固件包含 APP 核心 (~118.7 KB) 和 NET 核心 (~152.3 KB) 双核程序
- 字符串中可以发现 `"Raytac"`, `"MDBT53"`, `"nRF5340"` 等标识

### 出厂固件备份分析

通过 nRF Connect Programmer 读取的出厂备份（`Original_nRF_Connect_Programmer_*.hex`）包含：

| 区域 | 地址范围 | 大小 | 说明 |
|------|----------|------|------|
| APP 核心固件 | 0x00000000 - 0x0001DAC7 | 118.7 KB | Raytac BLE 测试固件（广播 "MDBT53-1M"） |
| Settings 存储 | 0x000FE000 - 0x000FE019 | 26 B | MCUboot 镜像 hash (`bt/hash`) |
| 分区表 | 0x000FEFE0 - 0x000FEFF7 | 24 B | Flash 分区配置信息 |
| NET 核心固件 | 0x01000000 - 0x01026157 | 152.3 KB | 网络核心 BLE Controller |

**⚠️ 关键发现：nRF Connect Programmer 只读取了 Flash，没有读取 UICR (0x00FF8000)！**
因此从 Programmer 备份中**无法确认出厂 APPROTECT 值**。

### 必须执行的完整备份命令

```powershell
$SNR = "802002436"   # 替换为你的 J-Link 序列号
$CLOCKSPEED = 8000
$TIMESTAMP = Get-Date -Format "yyyyMMdd_HHmmss"

# ====== 1. 备份 Flash (全部代码区) ======
# APP 核心 Flash
nrfjprog --readcode "factory_app_${TIMESTAMP}.hex" --family NRF53 --coprocessor CP_APPLICATION --snr $SNR --clockspeed $CLOCKSPEED
# NET 核心 Flash
nrfjprog --readcode "factory_net_${TIMESTAMP}.hex" --family NRF53 --coprocessor CP_NETWORK --snr $SNR --clockspeed $CLOCKSPEED

# ====== 2. 备份 UICR (极其重要！Programmer 不会读这个！) ======
# APP 核心 UICR（包含 APPROTECT/SECUREAPPROTECT）
nrfjprog --readuicr "factory_uicr_app_${TIMESTAMP}.hex" --family NRF53 --coprocessor CP_APPLICATION --snr $SNR --clockspeed $CLOCKSPEED
# NET 核心 UICR
nrfjprog --readuicr "factory_uicr_net_${TIMESTAMP}.hex" --family NRF53 --coprocessor CP_NETWORK --snr $SNR --clockspeed $CLOCKSPEED

# ====== 3. 直接读取 APPROTECT 值（立即确认是否解锁） ======
Write-Host "APP UICR.APPROTECT:"
nrfjprog --memrd 0x00FF8000 --n 4 --family NRF53 --snr $SNR --clockspeed $CLOCKSPEED
Write-Host "APP UICR.SECUREAPPROTECT:"
nrfjprog --memrd 0x00FF800C --n 4 --family NRF53 --snr $SNR --clockspeed $CLOCKSPEED

# 预期值:
#   0x50FA50FA = 解锁（安全，Raytac 出厂大概率是这个值）
#   0xFFFFFFFF = 锁定（危险！恢复这个备份会导致变砖）
```

### 备份文件建议存放

```
peripheral_nus/
├── factory_backup/                    # Git 忽略，单独保存
│   ├── factory_app_20260209_120000.hex
│   ├── factory_net_20260209_120000.hex
│   ├── factory_uicr_app_20260209_120000.hex  # ★ 最关键的文件
│   └── factory_uicr_net_20260209_120000.hex
```

> **建议把 `factory_backup/` 添加到 `.gitignore`，但务必在其他地方也保留一份副本。**

### 为什么 nRF Connect Programmer 的备份不够？

| 项目 | nRF Connect Programmer "Read" | nrfjprog --readcode + --readuicr |
|------|------|------|
| APP Flash | ✅ 读取 | ✅ 读取 |
| NET Flash | ✅ 读取 | ✅ 读取 |
| **UICR** | **❌ 不读取** | **✅ 读取** |
| APPROTECT 值 | **❌ 无法确认** | **✅ 可以确认** |
| 恢复出厂完整性 | ❌ 不完整 | ✅ 完整 |

---

## 烧录（极其重要）

### ❌ 绝对不要这样做

```powershell
# 危险！--erase 会清除 UICR → APPROTECT 锁定 → 变砖
nrfutil device program --firmware merged.hex --erase --jlink

# 也不要在 VS Code nRF Connect 插件里直接 "Flash All"（它可能带 --erase）

# 不要只烧一个核心
nrfjprog --program merged.hex --chiperase -f NRF53

# 不要不备份 UICR 就 chiperase
nrfjprog --program merged.hex --chiperase --coprocessor CP_APPLICATION -f NRF53

# 不要从固件写 UICR（即使是写 APPROTECT 解锁值，也有变砖风险）
NRF_UICR_S->APPROTECT = 0x50FA50FA;  // ❌ 不要这样做！
```

### ✅ 正确的烧录方式

使用项目自带的安全烧录脚本：

```powershell
.\tools\flash.ps1 -Snr <你的JLink序列号>

# 例如：
.\tools\flash.ps1 -Snr 802002436
```

脚本流程（与 OpenEarable `flash.sh` 保持一致）：

1. **备份 UICR**（`nrfjprog --readuicr`）— 失败则自动 recover
2. **烧录 Network Core**（`merged_CPUNET.hex` + `--chiperase --coprocessor CP_NETWORK`）
3. **烧录 Application Core**（`merged.hex` + `--chiperase --coprocessor CP_APPLICATION`）
4. **恢复 UICR**（从备份 hex 恢复）
5. **⭐ 无条件写 APPROTECT 解锁**（`0x50FA50FA` 到 `0x00FF8000` 和 `0x00FF800C`）
6. **复位**

> **关键：第 5 步是防变砖核心。** 即使 UICR 备份中 APPROTECT 是锁定值（`0xFFFFFFFF`），
> 第 5 步也会覆写为解锁值。这比 OpenEarable 的 `flash.sh` 多了一层保护。

### UICR 地址参考

```
nRF5340 UICR Application Core 布局：
  基地址:          0x00FF8000
  APPROTECT:       0x00FF8000  (offset 0x000) — 关键！
  RESERVED:        0x00FF8004  (offset 0x004) — 不要写！
  RESERVED:        0x00FF8008  (offset 0x008) — 不要写！
  SECUREAPPROTECT: 0x00FF800C  (offset 0x00C) — 关键！

  解锁值: 0x50FA50FA
  锁定值: 0xFFFFFFFF (chip erase 后的默认值)
  
  ⚠️ 注意：0x004 和 0x008 是 RESERVED！
  ⚠️ SECUREAPPROTECT 在 0x00C 不在 0x004！
```

### 为什么 --chiperase 会变砖？(根本原因链)

```
变砖因果链（真实发生过的场景）：

1. 全新芯片 → UICR.APPROTECT = 0xFFFFFFFF（出厂默认 = 锁定）
2. flash.ps1 第1步 → --readuicr → 备份了这个 0xFFFFFFFF
3. flash.ps1 第2-3步 → --chiperase + --program → UICR 被擦除
4. flash.ps1 第4步 → 恢复 UICR 备份 → APPROTECT 被写回 0xFFFFFFFF = 锁定!
5. ❌❌❌ 如果第5步（写解锁值）失败或被跳过 → 变砖

导致第5步失败的可能原因：
  - SWD 连接不稳定（J-Link 走线过长或接触不良）
  - BQ25120A /RESET 抖动（电池电压低、Hi-Z 模式切换）
  - nrfjprog 超时
  - 脚本在第4步后被中断

更致命的场景（旧版脚本没有第5步）：
  - 旧版 flash.ps1 恢复 UICR 后直接 reset → APPROTECT 生效 → SWD 死锁
  - 这就是导致所有板子变砖的直接原因
```

### 烧录前安全检查

```powershell
# 1. 确认 J-Link 能连上
nrfjprog --ids

# 2. 确认 SWD 通信正常
nrfjprog --memrd 0x00FF8000 --n 16 --family NRF53 --snr <SNR> --clockspeed 8000

# 3. 确认两个 hex 文件都存在
Test-Path build\merged.hex
Test-Path build\merged_CPUNET.hex

# 4. 确认 APPROTECT 当前状态
# 0x50FA50FA = 解锁（安全）
# 0xFFFFFFFF = 锁定（危险！）
nrfjprog --memrd 0x00FF8000 --family NRF53 --snr <SNR>
nrfjprog --memrd 0x00FF800C --family NRF53 --snr <SNR>
```

---

## 救砖

### 方案 1：标准恢复

```powershell
.\tools\recover.ps1 -Snr <JLink序列号>
```

脚本流程：
1. Recover NET core（clockspeed 8000，失败则降至 1000 重试）
2. Recover APP core（同上）
3. 验证 SWD 连接
4. 写 APPROTECT + SECUREAPPROTECT 解锁
5. 复位

### 方案 2：强力恢复（标准恢复失败时使用）

```powershell
# 在新的 PowerShell 窗口中运行（不要在 VS Code 终端）
cd E:\peripheral_nus
powershell -ExecutionPolicy Bypass -File .\tools\recover_and_flash.ps1
```

这个脚本：
1. nrfjprog --recover 重试 5 次，每次间隔 3 秒
2. 如果失败 → 使用 J-Link Commander 通过 CTRL-AP 执行 mass erase
3. 恢复成功后直接烧录 + 写 APPROTECT 解锁

### 方案 3：硬件干预（所有软件方案都失败时）

如果 SWD 完全无法连接，原因通常是 BQ25120A 通过 /RESET 将 nRF5340 持续拉低。

**诊断方法：**
1. 用万用表测量 nRF5340 RESETN 引脚电压
   - 如果 = 0V → BQ25120A /RESET 拉低 → 需要硬件干预
   - 如果 = VDD → 问题在 APPROTECT → 继续尝试软件方案

**硬件干预步骤：**

1. **同时接电池 + USB**（BQ25120A 检测到 PG 信号后可能释放 /RESET）
2. **接 USB 后立刻运行 recover**（时间窗口很短）：
   ```powershell
   # 接 USB → 立刻执行：
   nrfjprog --recover -f NRF53 --coprocessor CP_NETWORK --snr <SNR> --clockspeed 8000
   nrfjprog --recover -f NRF53 --coprocessor CP_APPLICATION --snr <SNR> --clockspeed 8000
   ```
3. 如果还不行 → **飞线 P0.17 到 VDD**（强制 CD=HIGH → BQ25120A 退出 Hi-Z → /RESET 释放）
   - 找到 P0.17 焊盘或 MDBT53 pin 26
   - 用镊子或飞线短接到 VDD (1.8V 或 3.3V 均可)
   - 保持短接的同时运行 recover 命令
4. 恢复后立即烧录修复固件：
   ```powershell
   .\tools\flash.ps1 -Snr <SNR>
   ```

> **⚠️ 注意：是 P0.17（不是 P1.05！）** 旧文档和旧代码中 P1.05 是按钮信号，不是 CD 引脚。

### Recovery 顺序注意事项

与 OpenEarable 一致，**始终先 recover NET core，再 recover APP core**：

```powershell
# 正确顺序（OpenEarable recover.sh）
nrfjprog --recover -f NRF53 --coprocessor CP_NETWORK --snr <SNR> --clockspeed 8000      # ← 先 NET
nrfjprog --recover -f NRF53 --coprocessor CP_APPLICATION --snr <SNR> --clockspeed 8000   # ← 后 APP
```

原因：APP core 的 SPU 可能会阻止对 NET core 的访问。先 recover NET core 可以绕过 SPU 限制。

---

## APPROTECT 深度解析

### 什么是 APPROTECT？

nRF5340 有两个硬件保护机制：
- **APPROTECT** — 保护 Application Core 的 SWD 调试端口
- **SECUREAPPROTECT** — 保护 Secure 域的调试端口

这两个机制由 UICR 中的值和 CTRLAP 寄存器共同控制。

### APPROTECT 生效流程

```
芯片复位
  │
  ├─ 硬件读取 UICR.APPROTECT (0x00FF8000)
  │   如果 = 0xFFFFFFFF → APPROTECT 启用（SWD 锁定）
  │   如果 = 0x50FA50FA → APPROTECT 禁用（SWD 开放）
  │
  ├─ 如果 APPROTECT 启用：
  │   固件可以通过 CTRLAP.APPROTECT.DISABLE = 0x50FA50FA
  │   在本次上电期间临时解锁 SWD
  │   但复位后又会锁定
  │
  └─ 如果 APPROTECT 禁用（UICR = 0x50FA50FA）：
      SWD 始终开放，无需固件干预
```

### 当前策略（与 OpenEarable 一致）

| 层级 | 手段 | 说明 |
|------|------|------|
| **烧录脚本** | `flash.ps1` 无条件写 UICR 解锁 | 唯一的 UICR 写入入口 |
| **Kconfig** | 不设 `USER_HANDLING` | 让 Zephyr 默认 SOC init 根据 UICR 值处理 CTRLAP |
| **固件** | 不碰 UICR，不碰 CTRLAP | 零风险 |

**OpenEarable 也是这个策略：固件中完全不涉及 APPROTECT，仅通过 nrfjprog 外部工具管理。**

### 为什么不在固件里写 UICR？

| 风险 | 说明 |
|------|------|
| NVMC 操作失败 | UICR 是 Flash 背写的，需要 NVMC enable → erase page → write。期间断电 = 损坏 |
| /RESET 干扰 | BQ25120A 可能在写 UICR 期间拉低 /RESET → NVMC 操作中断 → UICR 损坏 |
| 竞态条件 | SYS_INIT 太早，GPIO 驱动未就绪，I2C 未就绪，无法确认 BQ25120A 状态 |
| 变砖无法恢复 | 一旦 UICR 被写为非法值，`--recover` 是唯一救命手段 |

### 为什么不设 CONFIG_NRF_APPROTECT_USER_HANDLING？

设了 `USER_HANDLING=y` 意味着 Zephyr SOC init 不会自动处理 CTRLAP.APPROTECT.DISABLE。
如果你的固件自己也不写 CTRLAP（我们现在就是这样），那么：
- UICR = 0x50FA50FA → OK，APPROTECT 禁用
- UICR = 0xFFFFFFFF → APPROTECT 启用 → SWD 锁死

不设 `USER_HANDLING` 时，Zephyr SOC init 会自动检测 UICR 值并设置 CTRLAP。
这是更安全的默认行为。OpenEarable 也不设 `USER_HANDLING`。

---

## 固件初始化流程

固件启动顺序严格按照以下步骤执行，**顺序不可更改**：

### Phase 1: SYS_INIT (PRE_KERNEL_1, priority 0)

在内核启动之前执行。使用 HAL 寄存器直接操作，因为此时 GPIO 驱动尚未初始化。

```c
SYS_INIT(early_hw_init, PRE_KERNEL_1, 0);
```

执行内容：
1. **P0.17 = HIGH**（通过 `NRF_P0_S->PIN_CNF[17]` + `NRF_P0_S->OUTSET`）— 唤醒 BQ25120A
2. **无 APPROTECT 操作** — 遵循 OpenEarable 策略，UICR/CTRLAP 全部由烧录脚本处理

> **为什么在 SYS_INIT 里用原始寄存器？**
> PRE_KERNEL_1 priority 0 时 GPIO 驱动尚未初始化。必须用 `NRF_P0_S` 直接操作。
> OpenEarable 的 `mcuboot_hook.c` 也在 PRE_KERNEL_2 用 GPIO 驱动硬编码引脚。

### Phase 2: main() 电源轨初始化

```
main() 开始
  │
  ├─ usb_enable()                  // USB CDC 串口（非阻塞）
  │
  ├─ P1.11 = HIGH                  // 使能 1.8V 负载开关（OE: load_switch, default-on, delay=300µs）
  ├─ P0.14 = HIGH                  // 使能 3.3V 负载开关（OE: bq25120a/load-switch, delay=600µs）
  ├─ P0.17 = HIGH (GPIO 驱动加固)  // CD pin 双保险（SYS_INIT 已设）
  ├─ k_msleep(5)                   // 等电源轨稳定
  │
  ├─ I2C1 init                     // P0.21/P0.24, 400kHz（与 OE 完全一致）
  │
  ├─ bq25120_setup()               // ★ BQ25120A 完整配置（寄存器值与 OE 完全一致）
  │   ├─ CD = HIGH + 2ms 等待      //   唤醒 BQ25120A
  │   ├─ 读 Status (0x00)          //   验证 I2C
  │   ├─ 读 Fault (0x01)           //   记录故障
  │   ├─ TS 禁用 (0x02=0x00)       //   OE: setup_ts_control() → 写 0x00
  │   ├─ 电池电压 (0x05=0x8C)      //   OE: write_battery_voltage_control(4.3V)
  │   ├─ 充电电流 (0x03=0x9C)      //   OE: write_charging_control(110mA)
  │   ├─ 终止电流 (0x04=0x92)      //   OE: write_termination_control(10mA, enable)
  │   ├─ ★ LDO 3.3V (0x07=0xE4)   //   OE: write_LDO_voltage_control(3.3) + 显式使能
  │   └─ ILIM/UVLO (0x09=0x1A)     //   OE: write_uvlo_ilim(200mA, 3.0V)
  │
  ├─ k_msleep(5)                   // 等 3.3V LDO 稳定
  │
  ├─ ktd2026_reset()               // KTD2026 复位（0x00=0x07, 与 OE 一致）
  ├─ ktd2026_pulse(blue)           // 蓝色呼吸灯 = 广播模式
  │
  ├─ BQ27220 读电池电压             // 可选，失败不影响运行
  │
  ├─ bt_enable()                   // ★ 启用 BLE（阻塞等待网络核心 hci_ipc 响应）
  ├─ bt_nus_cb_register()          // 注册 NUS 回调
  ├─ bt_le_adv_start()             // 开始 BLE 广播 "Meow Sense Tag"
  │
  └─ while(1) k_sleep(10s)         // 心跳循环
```

> **关键：`bt_enable()` 是阻塞调用。** 如没烧 `merged_CPUNET.hex` 或没设 `CONFIG_SOC_NRF53_CPUNET_ENABLE=y`，固件永远卡在这里。

### 与 OpenEarable 初始化对比

| 阶段 | OpenEarable | Meow Sense Tag |
|------|------|------|
| Boot 引脚 | board_init.c POST_KERNEL: PM 设备管理 3 个 load switch | SYS_INIT PRE_KERNEL_1: P0.17 原始寄存器; main(): GPIO 驱动 P1.11/P0.14 |
| BQ25120A 寄存器 | setup(): reg 0x02/0x05/0x03/0x04/0x07/0x09 全写 | bq25120_setup(): **完全相同的寄存器和值** |
| KTD2026 操作 | reset(0x07), setColor(val-1), pulse() | **完全相同** |
| RT 核心释放 | board 级别 cpunet 配置 | CONFIG_SOC_NRF53_CPUNET_ENABLE=y |
| APPROTECT | 不在固件中操作 | **同样不操作**（flash.ps1 管理） |
| 语言 | C++ (C++17) | C |

---

## BQ25120A PMIC 驱动注意事项

### 寄存器映射（已通过 OpenEarable 源码和 TI 数据手册双重验证）

| 地址 | 名称 | 说明 |
|------|------|------|
| 0x00 | STATUS | 状态 & Ship Mode Reset |
| 0x01 | FAULTS | 故障 & 故障掩码 |
| 0x02 | TS_CTRL | 温度传感器控制 & 故障 |
| 0x03 | FAST_CHG | 快充电流控制 |
| 0x04 | TERM_CTRL | 终止 / 预充电控制 |
| 0x05 | BAT_V_CTRL | 电池电压控制 |
| **0x06** | **不存在！** | **⚠️ 写入会导致未定义行为 → PMIC 异常 → /RESET 拉低 → MCU 复位** |
| 0x07 | LS_LDO_CTRL | **负载开关 / LDO 控制（关键：使能 3.3V 输出）** |
| 0x08 | BTN_CTRL | 按键控制（wake_1, wake_2 状态） |
| 0x09 | ILIM_UVLO | 输入限流 & UVLO 控制（写 0x80 可 reset PMIC） |

### CD 引脚行为（P0.17，不是 P1.05！）

| 供电状态 | CD = HIGH | CD = LOW |
|----------|-----------|----------|
| **纯电池** | 退出 Hi-Z/Ship 模式，/RESET=HIGH，I2C 可用 | 进入 Hi-Z/Ship 模式，/RESET=LOW，I2C 不可用 |
| **USB 供电** | 禁止充电 | 允许充电 |

OpenEarable 对 CD 的操作模式：
```cpp
// 需要 I2C 通信时：
exit_high_impedance() → gpio_pin_set_dt(&cd_pin, 1);  // CD = HIGH
k_usleep(1000);  // 等 1ms 唤醒

// I2C 通信完成后（省电）：
enter_high_impedance() → gpio_pin_set_dt(&cd_pin, 0);  // CD = LOW

// USB 接入时的充电控制：
enable_charge()  → gpio_pin_set_dt(&cd_pin, 0);  // CD = LOW = 充电
disable_charge() → gpio_pin_set_dt(&cd_pin, 1);  // CD = HIGH = 禁充
```

**OpenEarable 的 I2C 时序要求：**
- I2C 操作间隔最小 66µs
- 退出 Hi-Z 后到 I2C 可用最小 1000µs（1ms）

### LDO 3.3V 使能

所有 BQ25120A 寄存器值均来自 OpenEarable 的 `BQ25120a::setup()` 函数：

```c
// bq25120_setup() 中的写入顺序与 OE setup() 完全一致：
i2c_reg_write(BQ25120A_ADDR, 0x02, 0x00);  // TS 禁用（OE: setup_ts_control()）
i2c_reg_write(BQ25120A_ADDR, 0x05, 0x8C);  // 电池电压 4.3V（OE: write_battery_voltage_control）
i2c_reg_write(BQ25120A_ADDR, 0x03, val);   // 充电 110mA（OE: write_charging_control, val≈0x9C）
i2c_reg_write(BQ25120A_ADDR, 0x04, 0x92);  // 终止 10mA（OE: write_termination_control）
i2c_reg_write(BQ25120A_ADDR, 0x07, 0xE4);  // LDO 3.3V 使能（OE: write_LDO_voltage_control +显式EN）
i2c_reg_write(BQ25120A_ADDR, 0x09, 0x1A);  // ILIM 200mA, UVLO 3.0V（OE: write_uvlo_ilim）
```

### PMIC 复位

```c
// 通过 ILIM_UVLO 寄存器 (0x09) 的 bit 7 软复位：
uint8_t val = 0x80;
i2c_write_reg(BQ25120A_ADDR, 0x09, val);
k_usleep(1000);  // 等复位完成
```

---

## Device Tree Overlay 配置

文件：`boards/nrf5340dk_nrf5340_cpuapp.overlay`

### I2C 总线

```dts
&pinctrl {
    i2c1_default: i2c1_default {
        group1 {
            psels = <NRF_PSEL(TWIM_SDA, 0, 21)>,   // P0.21 = SDA
                    <NRF_PSEL(TWIM_SCL, 0, 24)>;    // P0.24 = SCL
        };
    };
    i2c1_sleep: i2c1_sleep {
        group1 {
            psels = <NRF_PSEL(TWIM_SDA, 0, 21)>,
                    <NRF_PSEL(TWIM_SCL, 0, 24)>;
            low-power-enable;
        };
    };
};

&i2c1 {
    status = "okay";
    pinctrl-0 = <&i2c1_default>;
    pinctrl-1 = <&i2c1_sleep>;
    pinctrl-names = "default", "sleep";

    ktd2026: ktd2026@30 {
        compatible = "i2c-device";
        reg = <0x30>;
        label = "KTD2026";
    };
};
```

> BQ25120A (0x6A) 和 BQ27220 (0x55) 也在同一 I2C 总线上，通过直接地址访问，无需 DTS 声明。

### USB CDC ACM

```dts
&zephyr_udc0 {
    cdc_acm_uart0: cdc_acm_uart0 {
        compatible = "zephyr,cdc-acm-uart";
    };
};
```

### CD 引脚不使用 gpio-hog

**重要：** CD 引脚（P0.17）由 C 代码 (SYS_INIT + main) 控制，不使用 DTS gpio-hog。
原因：gpio-hog 在 GPIO 驱动初始化时才生效（POST_KERNEL），但我们需要在 PRE_KERNEL_1
就拉高 P0.17 以唤醒 BQ25120A。

P1.05 是 ON+BTN 信号（按钮），不是 CD 引脚，不需要 gpio-hog。

---

## Kconfig 配置说明

### prj.conf（项目级）

| 配置项 | 值 | 说明 |
|--------|----|------|
| `CONFIG_BT` | y | 启用蓝牙协议栈 |
| `CONFIG_BT_PERIPHERAL` | y | 外围设备角色 |
| `CONFIG_BT_DEVICE_NAME` | "Meow Sense Tag" | BLE 广播名称 |
| `CONFIG_BT_NUS` | y | Nordic UART Service |
| `CONFIG_BT_ZEPHYR_NUS` | y | Zephyr 原生 NUS 实现 |
| `CONFIG_I2C` | y | I2C 驱动 |
| `CONFIG_GPIO` | y | GPIO 驱动 |
| `CONFIG_USB_DEVICE_STACK` | y | USB 设备栈 |
| `CONFIG_USB_CDC_ACM` | y | USB 虚拟串口 |
| `CONFIG_NEWLIB_LIBC` | y | C 库（sscanf/snprintf） |
| `CONFIG_HEAP_MEM_POOL_SIZE` | 2048 | 堆内存 |

### boards/nrf5340dk_nrf5340_cpuapp.conf（板级）

| 配置项 | 值 | 说明 |
|--------|----|------|
| `CONFIG_UART_CONSOLE` | y | UART 控制台 |
| `CONFIG_USE_SEGGER_RTT` | y | J-Link RTT 日志（调试时不依赖 USB） |
| `CONFIG_LOG_BACKEND_RTT` | y | 日志通过 RTT 输出 |
| `CONFIG_SEGGER_RTT_BUFFER_SIZE_UP` | 2048 | RTT 缓冲区大小 |
| **`CONFIG_SOC_NRF53_CPUNET_ENABLE`** | **y** | **★ 关键！释放网络核心，否则 bt_enable() 卡死** |

**注意：不设 CONFIG_NRF_APPROTECT_USER_HANDLING。** 让 Zephyr 默认处理（与 OpenEarable 一致）。

### sysbuild.conf

```
SB_CONFIG_NETCORE_HCI_IPC=y
```

**必须有此配置。** 没有它，`merged_CPUNET.hex` 不会被生成。

---

## BLE 功能

### 设备信息

- 广播名称：`Meow Sense Tag`
- 服务：Nordic UART Service (NUS)
- UUID: `6E400001-B5A3-F393-E0A9-E50E24DCCA9E`
- 最大连接数：1

### NUS 命令

| 命令 | 示例 | 说明 |
|------|------|------|
| `LED:R,G,B` | `LED:255,0,0` | 设置 RGB 颜色 (0-255) |
| `MODE:OFF` | `MODE:OFF` | 关闭 LED |
| `MODE:SOLID` | `MODE:SOLID` | 常亮模式 |
| `MODE:BREATHE` | `MODE:BREATHE` | 呼吸灯效果 |
| `STATUS` | `STATUS` | 查询当前 LED 状态 |
| `BAT` | `BAT` | 读取电池电压/电流/SOC |

### LED 状态指示

| 状态 | LED 行为 |
|------|----------|
| 启动完成 | 蓝色常亮 |
| BLE 广播中 | 蓝色呼吸 |
| BLE 已连接 | 绿色常亮 |
| BLE 断开 | 回到蓝色呼吸 |

---

## 完整踩坑记录（12+ 个 Bug）

以下按发现顺序记录所有遇到的问题。每一个都是真实遇到的。

### Bug #1: BQ25120A 寄存器 0x06 不存在

**现象：** PMIC 异常，/RESET 被拉低，MCU 持续复位。

**原因：** 代码定义了 `BQ25120_REG_SYS_V_CTRL = 0x06` 并写入 `0x90`。BQ25120A 的寄存器从 0x05 直接跳到 0x07，0x06 不存在。写入未定义寄存器导致 I2C NACK 或 PMIC 异常。

**修复：** 删除 0x06，改用 0x07 (`LS_LDO_CTRL`)。

### Bug #2: 3.3V LDO 从未使能

**现象：** KTD2026 无响应，LED 不亮。

**原因：** BQ25120A LDO 默认关闭，需写 0x07=0xE4 使能。旧代码从未写此寄存器。

**修复：** `bq25120_init()` 中添加 `i2c_write_reg(BQ25120A_ADDR, 0x07, 0xE4)`。

### Bug #3: 网络核心未烧录

**现象：** `bt_enable()` 永远不返回。

**原因：** VS Code nRF Connect "Flash All" 有时只烧 APP core。必须用 `flash.ps1` 确保双核烧录。

**修复：** 使用 `flash.ps1`，始终先烧 NET (`--coprocessor CP_NETWORK`) 再烧 APP。

### Bug #4: --chiperase 摧毁 UICR → APPROTECT 变砖

**现象：** 烧录后 J-Link 再也连不上。

**原因：** `--chiperase` 清除 UICR，APPROTECT 恢复默认锁定 (`0xFFFFFFFF`)。

**修复：** 烧录脚本备份/恢复 UICR + 无条件写 APPROTECT 解锁。

### Bug #5: 电源时序不当

**现象：** I2C 设备偶尔无响应。

**原因：** 在 BQ25120A LDO 使能之前访问 KTD2026 → 设备无供电，I2C NACK。

**修复：** 严格按 P1.11(1.8V) → BQ25120A init(3.3V LDO) → k_msleep(5) → KTD2026 顺序。

### Bug #6: SECUREAPPROTECT 地址写错

**现象：** 写了 APPROTECT 但 SECUREAPPROTECT 仍然锁定 → Secure 域 SWD 死锁。

**原因：** 误以为 SECUREAPPROTECT 在 0x00FF8004（offset 0x004），实际在 **0x00FF800C**（offset 0x00C）。0x004 和 0x008 是 RESERVED。

**修复：** 使用正确地址 0x00FF800C。

### Bug #7: BLE 扫描响应缺少 NUS UUID

**现象：** 手机 nRF Connect 能看到设备但按 NUS 过滤搜不到。

**原因：** 扫描响应数据 (`sd[]`) 中没有包含 NUS UUID。

**修复：** 在 `sd[]` 中添加 `BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_SRV_VAL)`。

### Bug #8: 固件写 UICR.APPROTECT（最危险的 Bug）

**现象：** 板子在烧录后随机变砖。

**原因：** SYS_INIT 中执行 `NRF_UICR_S->APPROTECT = 0x50FA50FA`。UICR 是 Flash 背写的，需要 NVMC 擦写操作。如果 BQ25120A /RESET 在写入过程中抖动 → NVMC 中断 → UICR 损坏 → 可能变为锁定值。OpenEarable **从不在固件中碰 UICR**。

**修复：** 删除所有固件中的 UICR 写入，完全交给烧录脚本。

### Bug #9: CD 引脚搞混 P1.05 vs P0.17

**现象：** BQ25120A 不响应 I2C，LDO 不输出。

**原因：** 把 P1.05（ON+BTN，按钮信号通过 Q2 MOSFET）当成了 CD 引脚。实际的 BQ25120A CD 引脚连到 P0.17（MDBT53 pin 26 = CC_#CD）。

**修复：** `early_hw_init()` 改为操作 `NRF_P0_S` pin 17，`bq25120_cd_high()` 改为操作 gpio0 pin 17。

### Bug #10: CONFIG_NRF_APPROTECT_USER_HANDLING 引发风险

**现象：** 设了 USER_HANDLING 但固件不写 CTRLAP → chip erase 后 APPROTECT 没有被固件解锁。

**原因：** USER_HANDLING 意味着 Zephyr SOC init 不处理 CTRLAP，交给固件自行处理。但我们（已修正为）不在固件里写 CTRLAP/UICR → 导致矛盾。

**修复：** 删除 USER_HANDLING，让 Zephyr 默认处理（根据 UICR 值自动设置 CTRLAP）。

### Bug #11: UICR 备份恢复了 "锁定" 状态（★ 直接变砖原因）

**现象：** 烧录后板子立刻变砖，recovery 也失败。

**原因：** 全新芯片 UICR.APPROTECT = `0xFFFFFFFF`（默认锁定）。烧录脚本备份了这个值，chip erase 后恢复了它，APPROTECT 重新锁定。旧版脚本恢复后没有覆写解锁值就直接 reset → SWD 死锁。

**修复：** 恢复 UICR 后，**无条件** 写 APPROTECT=`0x50FA50FA` 和 SECUREAPPROTECT=`0x50FA50FA`，然后才 reset。

### Bug #12: CONFIG_SOC_NRF53_CPUNET_ENABLE 缺失

**现象：** LED 不亮（可能），BLE 不广播，固件看似卡死。

**原因：** 没有 `CONFIG_SOC_NRF53_CPUNET_ENABLE=y`，网络核心停留在 FORCEOFF。`bt_enable()` 永久阻塞等待 IPC。因为阻塞发生在 LED 初始化之后，LED 可能已经亮了，但没有 BLE。

**修复：** 在 `boards/nrf5340dk_nrf5340_cpuapp.conf` 添加 `CONFIG_SOC_NRF53_CPUNET_ENABLE=y`。

### Bug #13: P0.14 (LSCTRL 3.3V 负载开关) 未使能

**现象：** 即使 BQ25120A LDO 已设 3.3V，KTD2026 仍然不工作。

**原因：** BQ25120A LDO 输出通过 P0.14 的负载开关才到达 3.3V 外设。不拉高 P0.14，3.3V 送不到 KTD2026/BQ27220。OpenEarable 的 `board_init.c` 显式管理这个开关。

**修复：** `main()` 中添加 `gpio_pin_configure(gpio0_dev, 14, GPIO_OUTPUT_HIGH)`。

---

## 参考资料

### OpenEarable 关键文件

- [OpenEarable/open-earable-2](https://github.com/OpenEarable/open-earable-2) — 硬件参考设计的原始固件
  - `src/Battery/BQ25120a.cpp` — BQ25120A 驱动（CD pin 管理、I2C 时序、Hi-Z 模式）
  - `src/Battery/BQ25120a.h` — 正确的寄存器映射（0x00-0x05, **跳过 0x06**, 0x07-0x09）
  - `boards/teco/openearable_v2/board_init.c` — 电源管理 / 负载开关（P1.11、P0.14）
  - `boards/teco/openearable_v2/mcuboot_hook.c` — MCUboot 阶段硬编码 P1.11+P0.14
  - `boards/teco/openearable_v2/board.cmake` — J-Link 配置（`--speed=4000`）
  - `boards/teco/openearable_v2/openearable_v2_nrf5340_cpunet_reset.c` — 网络核心释放
  - `tools/flash/flash.sh` — **安全烧录脚本**（UICR 备份/恢复, NET→APP 顺序, clockspeed 8000）
  - `tools/flash/recover.sh` — 恢复脚本（NET 先, APP 后）
  - `tools/buildprog/program.py` — Python 烧录工具（NET=sectorerase, APP=chiperase, `--recover_on_fail`）

### 数据手册

- [BQ25120A Datasheet (TI)](https://www.ti.com/product/BQ25120A) — 寄存器映射在 Section 9.6
- [BQ27220 Datasheet (TI)](https://www.ti.com/product/BQ27220) — 标准命令在 Section 2.2
- [KTD2026 Datasheet](https://www.kinet-ic.com/ktd2026/) — LED 驱动寄存器
- [MDBT53-V-DB-40 Datasheet (Raytac)](https://www.raytac.com/product/ins.php?index_id=143) — 引脚映射
- [nRF5340 Product Specification (Nordic)](https://infocenter.nordicsemi.com/topic/ps_nrf5340/) — UICR/APPROTECT 在 Chapter 5

### 工具

- [Zephyr RTOS Documentation](https://docs.zephyrproject.org/)
- [nRF Connect SDK Documentation](https://docs.nordicsemi.com/bundle/ncs-latest/page/nrf/index.html)
- [nrfjprog User Guide](https://docs.nordicsemi.com/bundle/nrfjprog/page/UG/nrfjprog/nrfjprog_intro.html) — 命令行参考
