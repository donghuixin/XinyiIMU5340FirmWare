/*
 * Meow Sense Tag — BLE Control Service
 *
 * Custom GATT service for external BLE control of IMU data streaming
 * and battery status queries.
 *
 * Commands (write to Command characteristic):
 *   's' — Start IMU data streaming at 104 Hz
 *   'p' — Stop IMU data streaming, put BMX160 in low-power mode
 *   'b' — Query battery status (voltage, SOC, charging state)
 *
 * Notifications:
 *   IMU Data:     header 0xAA 0x55 + 10 × (9 floats) = 362 bytes
 *   Battery Data: header 0xBB 0x66 + voltage(2) + soc(1) + charging(1) = 6
 * bytes
 */

#ifndef MEOW_CTRL_SERVICE_H
#define MEOW_CTRL_SERVICE_H

#include <zephyr/bluetooth/gatt.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the Meow Sense Tag control service.
 *
 * Registers the GATT service and initializes internal state.
 * Must be called after BT is enabled and after power_manager.begin().
 *
 * @return 0 on success, negative errno on failure.
 */
int init_meow_ctrl_service(void);

/**
 * @brief Start the serial console thread.
 *
 * Must be called AFTER usb_enable() so CDC ACM is ready.
 */
void meow_ctrl_start_serial_thread(void);

#ifdef __cplusplus
}
#endif

#endif /* MEOW_CTRL_SERVICE_H */
