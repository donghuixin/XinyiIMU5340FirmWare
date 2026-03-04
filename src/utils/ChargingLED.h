#ifndef _CHARGING_LED_H
#define _CHARGING_LED_H

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief P1.08 single-color LED controller.
 *
 * Modes:
 *   - OFF:       LED is off.
 *   - BLINK_1S:  1-second period blink (500ms on / 500ms off).
 *                Used during BLE advertising when not connected.
 *   - BREATHE_3S: 3-second period software breathing (ramp up / ramp down
 *                 using fast PWM toggle). Used during charging.
 */

enum charging_led_mode {
    CHARGING_LED_OFF,
    CHARGING_LED_BLINK_1S,   /* BLE advertising, not connected */
    CHARGING_LED_BREATHE_3S, /* Charging in progress */
};

/**
 * @brief Initialize the P1.08 LED GPIO.
 * @return 0 on success, negative errno on failure.
 */
int charging_led_init(void);

/**
 * @brief Set the LED operating mode. Thread-safe.
 * @param mode One of enum charging_led_mode.
 */
void charging_led_set_mode(enum charging_led_mode mode);

#ifdef __cplusplus
}
#endif

#endif /* _CHARGING_LED_H */
