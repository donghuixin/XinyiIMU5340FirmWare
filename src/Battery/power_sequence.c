#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(power_sequence, LOG_LEVEL_INF);

#define BQ25120A_LS_LDO_CTRL_REG 0x07
#define BQ25120A_LS_LDO_CTRL_VAL 0xE4

/* P0.27 – wakes BQ25120A from ship mode / asserts LSCTRL */
static const struct gpio_dt_spec ldo_ctrl_gpio =
	GPIO_DT_SPEC_GET(DT_ALIAS(ldo_ctrl), enable_gpios);

/* P0.14 – LS_LBEN: physical enable for BQ25120A load-switch output */
#define LS_EN_NODE DT_CHILD(DT_NODELABEL(bq25120a), load_switch)
static const struct gpio_dt_spec ls_en_gpio =
	GPIO_DT_SPEC_GET(LS_EN_NODE, enable_gpios);

static const struct i2c_dt_spec bq25120a_i2c =
	I2C_DT_SPEC_GET(DT_NODELABEL(bq25120a));

void force_load_switches_on(void)
{
	int ret;

	/* --- Step 1: assert LSCTRL (P0.27) to wake PMIC --- */
	if (!device_is_ready(ldo_ctrl_gpio.port)) {
		LOG_ERR("ldo_ctrl GPIO not ready");
		return;
	}

	ret = gpio_pin_configure_dt(&ldo_ctrl_gpio, GPIO_OUTPUT_ACTIVE);
	if (ret) {
		LOG_ERR("Failed to configure ldo_ctrl: %d", ret);
		return;
	}

	k_msleep(20);

	/* --- Step 2: program BQ25120A LDO to 3.3 V, EN_LS=1 --- */
	if (!i2c_is_ready_dt(&bq25120a_i2c)) {
		LOG_ERR("BQ25120A I2C bus not ready");
		return;
	}

	ret = i2c_reg_write_byte_dt(&bq25120a_i2c, BQ25120A_LS_LDO_CTRL_REG,
				    BQ25120A_LS_LDO_CTRL_VAL);
	if (ret) {
		LOG_ERR("Failed to write BQ25120A reg 0x07: %d", ret);
		return;
	}

	/* --- Step 3: drive LS_LBEN (P0.14) HIGH to enable load-switch output --- */
	if (!device_is_ready(ls_en_gpio.port)) {
		LOG_ERR("LS enable GPIO not ready");
		return;
	}

	ret = gpio_pin_configure_dt(&ls_en_gpio, GPIO_OUTPUT_ACTIVE);
	if (ret) {
		LOG_ERR("Failed to configure LS enable (P0.14): %d", ret);
		return;
	}

	LOG_INF("Load-switch enabled (P0.14 HIGH, EN_LS=1)");

	/* Allow rails to stabilise (datasheet: 600 µs, add margin) */
	k_msleep(50);
}

static int power_sequence_init(void)
{
	force_load_switches_on();
	LOG_INF("Power sequence complete");

	return 0;
}

SYS_INIT(power_sequence_init, POST_KERNEL, 50);
