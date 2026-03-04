#ifndef _POWER_MANAGER_H
#define _POWER_MANAGER_H

#include <zephyr/kernel.h>

#include "BQ27220.h"
#include "BQ25120a.h"

#include "../bluetooth/gatt_services/battery_service.h"

#include "openearable_common.h"
#include "BootState.h"

#define DEBOUNCE_POWER_MS K_MSEC(1000)

/* Low-battery periodic BLE report interval (5 minutes) */
#define LOW_BAT_REPORT_INTERVAL K_MINUTES(5)
/* Voltage thresholds for simple percentage mapping */
#define BAT_VOLTAGE_MAX  4.2f   /* >= 4.2V → 100% */
#define BAT_VOLTAGE_LOW  3.6f   /* <= 3.6V → 30%  */
#define BAT_PERCENT_MAX  100
#define BAT_PERCENT_LOW  30

class PowerManager {
public:
    int begin();

    int power_down(bool fault = false);
    //bool check_boot_condition();

    //static LoadSwitch v1_8_switch;

    void reboot();

    void get_battery_status(battery_level_status &status);
    void get_energy_status(battery_energy_status &status);
    void get_health_status(battery_health_status &status);

    /** Convert voltage (V) to battery percentage using linear mapping.
     *  >= 4.2V → 100%, <= 3.6V → 30%, linear in between. */
    static uint8_t voltage_to_percent(float voltage);

    void set_error_led(int val = 1);

    static k_work_delayable power_down_work;
private:
    bool power_on = false;
    bool charging_disabled = false;
    uint16_t last_charging_state = 0;
    bool low_bat_timer_running = false;

    enum charging_state last_charging_msg_state = DISCHARGING;

    void charge_task();

    void power_connected();

    bool check_battery();

    k_timeout_t chrg_interval = K_SECONDS(CONFIG_BATTERY_CHARGE_CONTROLLER_NORMAL_INTERVAL_SECONDS);

    static k_work_delayable charge_ctrl_delayable;

    //static k_work power_down_work;
    static k_work fuel_gauge_work;
    static k_work battery_controller_work;

    static void charge_ctrl_work_handler(struct k_work * work);
    static void power_down_work_handler(struct k_work * work);
    static void fuel_gauge_work_handler(struct k_work * work);
    static void battery_controller_work_handler(struct k_work * work);

    /* Periodic low-battery BLE report (every 5 min when V < 3.6V) */
    static k_work_delayable low_bat_report_work;
    static void low_bat_report_work_handler(struct k_work *work);

    static void power_good_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
    static void fuel_gauge_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
    static void battery_controller_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins);

    const battery_settings _battery_settings = {
        3.7, 4.3, 3.0, 2.5,  // Spannungen
        10, 110, 200,        // Ströme
        108,                 // Kapazität
        0, 15, 45, 50        // Temperaturen
    };

    const struct gpio_dt_spec error_led = GPIO_DT_SPEC_GET(DT_NODELABEL(led_error), gpios);

    friend int cmd_setup_fuel_gauge(const struct shell *shell, size_t argc, const char **argv);
};

extern PowerManager power_manager;

#endif