/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/shell/shell_uart.h>
#include <zephyr/usb/usb_device.h>

#include <zephyr/settings/settings.h>

#include "macros_common.h"
#include "openearable_common.h"
#include "streamctrl.h"

#include "../src/Battery/PowerManager.h"
#include "../src/SensorManager/SensorManager.h"

// [關鍵新增] 直接引入 IMU 的標頭檔，確保編譯器認得 IMU::sensor
#include "../src/SensorManager/IMU.h"

#include "../src/utils/StateIndicator.h"

#include "battery_service.h"
#include "button_service.h"
#include "device_info.h"
#include "led_service.h"
#include "meow_ctrl_service.h"
#include "sensor_service.h"

#include "DefaultSensors.h"
#include "SensorScheme.h"

#include "../src/SD_Card/SDLogger/SDLogger.h"

#include "bt_mgmt.h"
#include "uicr.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, CONFIG_MAIN_LOG_LEVEL);

#include <zephyr/bluetooth/services/nus.h>

extern "C" void force_load_switches_on(void);

// [關鍵新增] 宣告定義在 SensorManager.cpp 中的 sensor_queue
// 這樣我們才能在 main 裡面把它傳遞給 IMU 的 init 函數
extern struct k_msgq sensor_queue;

static void i2c_bus_scan(const struct device *bus, const char *name) {
  if (!device_is_ready(bus)) {
    LOG_WRN("I2C bus %s not ready, skip scan", name);
    return;
  }
  LOG_INF("Scanning %s ...", name);
  int found = 0;
  for (uint8_t addr = 0x08; addr < 0x78; addr++) {
    uint8_t dummy;
    int ret = i2c_read(bus, &dummy, 0, addr);
    if (ret == 0) {
      LOG_INF("  %s: device @ 0x%02x", name, addr);
      found++;
    }
  }
  LOG_INF("  %s: %d device(s)", name, found);
}

int main(void) {
  int ret;

  /* FORCE POWER RAILS ON EARLY */
  force_load_switches_on();

  // 动作 2：手动给 PMIC 发指令提压到 3.3V, 等硬件苏醒
  const struct device *i2c1_dev = DEVICE_DT_GET(DT_NODELABEL(i2c1));
  if (device_is_ready(i2c1_dev)) {
    uint8_t reg_val = 0xE4; // 3.3V LDO Output
    int r = i2c_reg_write_byte(i2c1_dev, 0x6A, 0x07, reg_val);
    if (r == 0) {
      printk("MANUAL BQ25120A LDO TO 3.3V SUCCESS!\n");
      k_msleep(50); // Waiting for KTD2026 hardware to wake up
    } else {
      printk("MANUAL BQ25120A LDO WRITE FAILED: %d\n", r);
    }
  }

  LOG_INF("nRF5340 APP core started - scanning I2C buses");

  /* I2C bus scan for hardware discovery */
  i2c_bus_scan(DEVICE_DT_GET(DT_NODELABEL(i2c1)), "i2c1");
  i2c_bus_scan(DEVICE_DT_GET(DT_NODELABEL(i2c2)), "i2c2");
  LOG_INF("I2C scan complete");

  k_msleep(1000); // Wait for J-Link to poll RTT before touching hardware

  LOG_INF("Starting PowerManager::begin()...");
  ret = power_manager.begin();
  LOG_INF("PowerManager::begin() returned %d", ret);
  ERR_CHK(ret);

  uint8_t standalone = uicr_standalone_get();
  LOG_INF("Standalone mode: %i", standalone);

  /* STEP 5.5 - Enable USB */
  if (IS_ENABLED(CONFIG_USB_DEVICE_STACK)) {
    ret = usb_enable(NULL);
    if (ret) {
      LOG_ERR("Failed to enable USB");
      return 0;
    }
  }

  /* Start serial thread now that USB CDC ACM is ready */
  meow_ctrl_start_serial_thread();
  streamctrl_start();

  uint32_t sirk = uicr_sirk_get();
  if (sirk == 0xFFFFFFFFU) {
    state_indicator.set_pairing_state(SET_PAIRING);
  } else if (bonded_device_count > 0 && !oe_boot_state.timer_reset) {
    state_indicator.set_pairing_state(PAIRED);
  } else {
    state_indicator.set_pairing_state(BONDING);
  }

  // 初始化 SensorManager (此時只是準備好佇列與執行緒，還沒操作硬體)
  init_sensor_manager();

  // =========================================================================
  // [系統穩定性核心] 同步硬體喚醒區塊
  // 在藍牙通訊還沒產生大量中斷前，直接在這裡完成耗時的 IMU 喚醒流程。
  // =========================================================================
  LOG_INF("--- Performing Synchronous Hardware Initializations ---");
  LOG_INF("Initializing IMU hardware (blocking wait for PMU)...");

  // 直接呼叫 IMU::sensor.init，完美避開 get_sensor 宣告缺失的問題
  IMU::sensor.init(&sensor_queue);

  LOG_INF("--- Hardware Initializations Complete ---");
  // =========================================================================

  ret = init_led_service();
  ERR_CHK(ret);

  ret = init_battery_service();
  ERR_CHK(ret);

  ret = init_button_service();
  ERR_CHK(ret);

  ret = initParseInfoService(&defaultSensorIds, defaultSensors);
  ERR_CHK(ret);

  ret = init_sensor_service();
  ERR_CHK(ret);

  ret = init_meow_ctrl_service();
  ERR_CHK(ret);

  return 0;
}