#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h> // ✅ Correct Power Management API
LOG_MODULE_REGISTER(board_init, LOG_LEVEL_DBG);

// #include "nrf5340_audio_common.h"

#include <zephyr/drivers/gpio.h>

#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>

const struct device *const cons = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

struct load_switch_data {
  struct gpio_dt_spec ctrl_pin;
  int delay_us;
  bool default_on;
};

int generic_pm_control(const struct device *dev, enum pm_device_action action) {
  struct load_switch_data *data = dev->data;

  switch (action) {
  case PM_DEVICE_ACTION_SUSPEND:
    /* suspend the device */
    gpio_pin_set_dt(&data->ctrl_pin, 0);
    break;
  case PM_DEVICE_ACTION_RESUME:
    /* resume the device */
    gpio_pin_set_dt(&data->ctrl_pin, 1);
    k_usleep(data->delay_us); // LS: t_on = 250µs, LDO: 500µs
    break;
  default:
    return -ENOTSUP;
  }

  return 0;
}

int init_pm_device(const struct device *dev) {
  struct load_switch_data *data = dev->data;
  int ret;

  ret = device_is_ready(data->ctrl_pin.port);
  if (!ret) {
    printk("Pins not ready.\n");
    return -1;
  }

  ret = gpio_pin_configure_dt(&data->ctrl_pin, data->default_on
                                                   ? GPIO_OUTPUT_ACTIVE
                                                   : GPIO_OUTPUT_INACTIVE);
  if (ret != 0) {
    printk("Failed to setup Load Switch.\n");
    return ret;
  }

  /* Enable runtime PM for this load switch so get/put work properly */
  pm_device_runtime_enable(dev);

  return 0;
}
