# XinyiIMU — nRF5340 Multi-Sensor IMU Firmware

XinyiIMU is a dual-core nRF5340-based firmware for a compact, multi-sensor wearable IMU platform. It provides 9-axis inertial sensing (BMI160/BMX160), environmental sensing, optical heart-rate (PPG), temperature monitoring, LE Audio, BLE data streaming, and SD card logging — all running on Zephyr RTOS (nRF Connect SDK v3.0.1).

## Features

- **Dual-core nRF5340**: Application core (Cortex-M33 @ 128 MHz) + Network core (SoftDevice Controller / LE Audio)
- **9-axis IMU**: BMI160/BMX160 accelerometer + gyroscope + magnetometer via I2C, configurable ODR (25–800 Hz)
- **Environmental sensors**: BMP388 barometer, MLX90632 IR thermometer
- **Optical PPG**: MAXM86161 pulse oximetry / heart-rate sensor
- **Bone-conduction mic**: BMA580 high-frequency accelerometer for bone-conduction audio
- **Power management**: BQ25120A PMIC with load-switch sequencing, BQ27220 fuel gauge
- **Audio codec**: ADAU1860 + LC3 encode/decode for LE Audio unicast
- **BLE streaming**: Real-time sensor data streaming over GATT notifications
- **SD card logging**: FAT/exFAT microSD recording in binary `.oe` format
- **FOTA support**: MCUBoot + mcumgr SMP for over-the-air firmware updates
- **LED status**: KTD2026 RGB LED for battery/connection/recording states

## Hardware

| Bus  | Pins (SCL / SDA)   | Sensors                                                         |
|------|--------------------|-----------------------------------------------------------------|
| I2C1 | P0.24 / P0.21     | BQ25120A (PMIC), BQ27220 (Fuel Gauge), ADAU1860, KTD2026 (LED) |
| I2C2 | P1.02 / P1.03    | **BMI160 (IMU)**, MAXM86161 (PPG), MLX90632 (Temp)             |
| I2C3 | P1.00 / P1.15      | BMP388 (Barometer), BMA580 (Bone Conduction)                   |

## Table of Contents

1. [Setup](#setup)
2. [Build & Flash](#build--flash)
3. [Battery States](#battery-states)
4. [Connection States](#connection-states)
5. [SD Card](#sd-card)
6. [Changelog](#changelog)
7. [Acknowledgments](#acknowledgments)

---

## Setup

### Prerequisites

1. **Visual Studio Code** — [https://code.visualstudio.com](https://code.visualstudio.com)
2. **J-Link Software** — [https://www.segger.com/downloads/jlink/](https://www.segger.com/downloads/jlink/)
3. **nRF-Util** — [Nordic nRF Util](https://www.nordicsemi.com/Products/Development-tools/nRF-Util) (add to `PATH`)
4. **nRF Connect for VS Code** extension — install from the VS Code Extensions tab, including all prompted dependencies

### SDK & Toolchain

1. Open the **nRF Connect** tab in VS Code.
2. Install **Toolchain v3.0.1** and **SDK v3.0.1**.

### Open & Configure Project

1. `File > Open Folder` → select the `XinyiIMU` firmware directory.
2. In the **APPLICATIONS** section of the nRF Connect tab, click **"+ Add build configuration"**.
3. Set:
   - **Board target**: `openearable_v2/nrf5340/cpuapp`
   - **SDK / Toolchain**: 3.0.1
4. For **FOTA** builds: add `-DFILE_SUFFIX="fota"` as Extra CMake argument, build directory `build_fota`.
5. For **standard** builds: select `prj.conf` as the Kconfig fragment.

---

## Build & Flash

### VS Code (GUI)

Click **Generate and Build** in the nRF Connect extension panel.

### CLI

```bash
# Pristine build (standard)
west build -p always -b openearable_v2/nrf5340/cpuapp -d build -- -DBOARD_ROOT=.

# FOTA build
west build -p always -b openearable_v2/nrf5340/cpuapp -d build_fota -- -DBOARD_ROOT=. -DFILE_SUFFIX="fota"
```

### Flash

```bash
# Standard flash (may need --recover if readback protection is enabled)
west flash -d build

# With recovery (erases full flash + clears access port protection)
west flash -d build --recover
```

### Flash Scripts (with J-Link serial number)

```bash
# Linux / macOS
./tools/flash/flash_fota.sh --snr <JLINK_SERIAL> --left --hw 2.0.1

# Windows (PowerShell, run as Administrator)
.\tools\flash\flash_fota.ps1 -snr <JLINK_SERIAL> -left -hw 2.0.1
```

### Recovery

If the application or network core becomes unresponsive:

```bash
./tools/flash/recover.sh --snr <JLINK_SERIAL>
```

### Debug Output

1. Enable **Virtual COM-Port** in J-Link Configuration program.
2. Install the [Serial Monitor](https://marketplace.visualstudio.com/items?itemName=ms-vscode.vscode-serial-monitor) VS Code extension.
3. Open Serial Monitor → select J-Link COM port → baud rate **115200** → Start Monitoring.

---

## Battery States

Battery LED states override connection LED states. All LED states can be manually overwritten via BLE service.

### Charging

| LED State              | Description                                         |
|------------------------|-----------------------------------------------------|
| 🟥 Red – Solid         | Battery fault or deep discharge, charging current = 0 |
| 🔴 Red – Pulsing       | Pre-charge phase                                    |
| 🟧 Orange – Solid      | Power connected, charging current not at target     |
| 🟠 Orange – Pulsing    | ≥ 80% of target charging current reached            |
| 🟢 Green – Pulsing     | Trickle/CV phase, final voltage reached             |
| 🟩 Green – Solid       | Fully charged                                       |

> If the device enters deep discharge (solid red), unplug and replug USB to recover.

### Discharging

| LED State              | Description                          |
|------------------------|--------------------------------------|
| 🟠 Orange – Blinking   | Battery low (≈ 7%)                   |
| 🔴 Red – Blinking      | Battery critical (≈ 3%)              |

---

## Connection States

| LED State                       | Description                              |
|---------------------------------|------------------------------------------|
| 🔵 Blue – Blinking Very Fast    | Searching for paired device              |
| 🔵 Blue – Blinking Fast         | Ready for bonding                        |
| 🔵 Blue – Blinking Slow         | Bonded, waiting for connection           |
| 🟢 Green – Blinking Slow        | Connected                                |
| 🟣 Purple – Blinking Slow       | SD card recording active                 |

---

## SD Card

ZephyrOS does not support SD card hot-swap — **power off the device before inserting or removing the microSD card**.

Recorded binary `*.oe` files can be parsed with <a href="https://colab.research.google.com/drive/1qwdvjAM5Y5pLbNW5t3r9f0ITpAuxBKeq" target="_blank">this Python notebook</a>.

---

## Changelog

### v2.2.2 — 2026-03-01
- **Fix: I2C init priority race** — `power_sequence.c` SYS_INIT moved from priority 50 → 51, ensuring I2C drivers initialize before PMIC load-switch programming
- **Fix: Zephyr BMI160 driver conflict** — Disabled in-tree `CONFIG_BMI160` to prevent double-init contention with custom DFRobot_BMI160 driver
- **Fix: DTS compatible change** — BMI160 node changed from `"bosch,bmi160"` to `"i2c-device"` to prevent auto-selection of Zephyr driver
- **Improved: IMU power-up margin** — Extended BMI160 power-up delay from 50 ms to 100 ms for reliable load-switch rail stabilization
- **Improved: BMI160 scan diagnostics** — Added I2C bus readiness check and per-attempt debug logging in `scan()`

### v2.2.1 — 2025-12-01
- Major firmware refactoring & build system fix
- Auto-discovery of `BOARD_ROOT` via `sysbuild/CMakeLists.txt` and `zephyr/module.yml`
- New `power_sequence.c` — early-boot BQ25120A PMIC load-switch initialization
- Battery management: BQ25120A driver, BQ27220 fuel gauge integration, PowerManager
- LED control via KTD2026 driver with battery/connection state visualization
- BLE sensor streaming and SD card logging modules
- Zephyr zbus message architecture for cross-module communication

### v1.0.0 — Initial
- Initial commit — base firmware ported to custom nRF5340 board

---

## Acknowledgments

This project is built upon the [OpenEarable](https://open-earable.teco.edu/) open-source platform developed by TECO (Karlsruhe Institute of Technology). The hardware design, Zephyr board definitions, audio pipeline, and BLE architecture originate from the OpenEarable 2 project. We gratefully acknowledge their pioneering work in open-source ear-based sensing.
