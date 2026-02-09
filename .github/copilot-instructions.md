# AI Coding Agent Guide — OpenEarable 2 Firmware

This repository is a Zephyr/NCS-based dual-core nRF5340 firmware for OpenEarable 2 with LE Audio, SD logging, and optional FOTA. Use these guidelines to be productive quickly.

## Big Picture
- Platform: nRF Connect SDK v3.0.1 (pinned in west manifest) with Zephyr RTOS.
- Dual-core split: CPUNET runs SoftDevice Controller; CPUAPP runs Bluetooth Host + application.
- Audio roles: Gateway/client vs Headset/server; same codebase configured via Kconfig fragments and overlays.
- Message bus: Modules communicate via Zephyr `zbus` with typed messages (see `include/zbus_common.h`).

## Code Organization
- Core headers: `include/openearable_common.h` (device/shared types), `include/zbus_common.h` (zbus message types).
- Application modules in `src/`:
  - `audio/`, `bluetooth/`, `buttons/`, `Battery/`, `SensorManager/`, `SD_Card/`, `drivers/`, `modules/`, `utils/`, `Wire/`.
- Example app: `unicast_server/` (LE Audio unicast headset) with `main.cpp` and minimal `CMakeLists.txt`.
- Board/overlay and config: `boards/`, `.overlay` files, `prj*.conf`, `sysbuild_fota.conf`, `pm_static_fota.yml`.

## Communication & Patterns
- Zbus messages centralize cross-module events: `le_audio_msg`, `button_msg`, `sdu_ref_msg`, `bt_mgmt_msg`, `volume_msg`, `content_control_msg`.
- Time units convention: variables representing times end with `_us` inside sync modules (see docs); network/app core timers align.
- Sensor/data shapes: `SENSOR_DATA_FIXED_LENGTH` and packed `sensor_data`/`sensor_msg` in `openearable_common.h`.
- Audio datapath: LC3 encode/decode; synchronization and drift/presentation compensation handled by `audio_datapath.c` (described in docs).

## Build Configurations
- Default debug: `prj.conf` (logging, shells, UART/RTT enabled).
- Release: `prj_release.conf` via `-DFILE_SUFFIX=release`.
- FOTA: `prj_fota.conf` + `sysbuild_fota.conf` via `-DFILE_SUFFIX=fota`; enables MCUBoot, external flash DFU, mcumgr.
- Key partitions: `pm_static_fota.yml` (check/update for your device’s external flash layout).
- SDK pin: `west.yml` uses `sdk-nrf@v3.0.1`.

## Build & Flash Workflows
- VS Code (nRF Connect extension): add build config with Toolchain/SDK 3.0.1. For FOTA set `-DFILE_SUFFIX=fota` and use `build_fota` dir.
- CLI examples (CPUAPP headset, release):
  - `west build -b nrf5340_audio_dk/nrf5340/cpuapp --pristine -- -DCONFIG_AUDIO_DEV=1 -DFILE_SUFFIX=release`
  - FOTA: `west build -b nrf5340_audio_dk/nrf5340/cpuapp --pristine -- -DCONFIG_AUDIO_DEV=1 -DFILE_SUFFIX=fota`
- Flash scripts:
  - FOTA build: `tools/flash/flash_fota.sh` (Linux/macOS) or `tools/flash/flash_fota.ps1` (Windows). Writes images and configures UICR.
  - Non-FOTA build: `tools/flash/flash.sh`.
  - Recovery: `tools/flash/recover.sh --snr <SNR>`.
- UICR fields (set by flash scripts):
  - Ear side at `0x00FF80F4` (0 = left, 1 = right)
  - Standalone flag at `0x00FF80FC` (0 = standalone)
  - Hardware version at `0x00FF8100` (value `0xMMmmpp00` from `x.y.z`)

## FOTA/DFU Workflow
- Transport: mcumgr SMP over UART/BT; DFU mode advertises role-specific names.
- Upload (serial example): see `tools/flash/mcumgr_upload.sh` — uploads app/net images from `build_fota`, tests hashes, then resets.
- Docs clarify DFU entry (BTN4 on DK) and multi-image DFU requirements; prefer multi-image to keep app/net compatibility.

## Conventions & Gotchas
- Use zbus to decouple modules; avoid direct cross-module calls when a bus message exists.
- Timekeeping: align network/app core timers; pass SDU reference timestamps to sync module.
- SD card: FAT/exFAT enabled; Zephyr cannot remount — power off before changing cards (see README).
- Device name/product strings set in Kconfig: `CONFIG_BT_DEVICE_NAME="OpenEarable"`, USB product `OpenEarable v2`.
- Builds may use C++ (`CONFIG_STD_CPP17`); keep headers C-compatible where needed.

## Quick References
- Overview & architecture: `doc/firmware_architecture.rst`
- Build & config details: `doc/building.rst`
- FOTA specifics: `doc/fota.rst`
- Scripts: `tools/flash/*`, `tools/buildprog/*`, `tools/uart_terminal/`
