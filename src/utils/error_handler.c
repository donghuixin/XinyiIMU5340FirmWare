/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/drivers/gpio.h>
#include <zephyr/fatal.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/sys/reboot.h>

/* Print everything from the error handler */
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(error_handler, CONFIG_ERROR_HANDLER_LOG_LEVEL);

#if (defined(CONFIG_BOARD_NRF5340_AUDIO_DK_NRF5340_CPUAPP) && (CONFIG_DEBUG))
/* nRF5340 Audio DK center RGB LED */
static const struct gpio_dt_spec center_led_r =
    GPIO_DT_SPEC_GET(DT_NODELABEL(rgb1_red), gpios);
static const struct gpio_dt_spec center_led_g =
    GPIO_DT_SPEC_GET(DT_NODELABEL(rgb1_green), gpios);
static const struct gpio_dt_spec center_led_b =
    GPIO_DT_SPEC_GET(DT_NODELABEL(rgb1_blue), gpios);
#endif /* (defined(CONFIG_BOARD_NRF5340_AUDIO_DK_NRF5340_CPUAPP) &&            \
          (CONFIG_DEBUG)) */

void error_handler(unsigned int reason, const struct arch_esf *esf) {
  /* DIAG: Always print fault info via printk (goes to RTT) */
  printk("\n\n[FATAL] === SYSTEM ERROR reason=%d ===\n", reason);
  if (esf) {
    printk("[FATAL] r0=0x%08x r1=0x%08x r2=0x%08x r3=0x%08x\n", esf->basic.r0,
           esf->basic.r1, esf->basic.r2, esf->basic.r3);
    printk("[FATAL] r12=0x%08x lr=0x%08x pc=0x%08x xpsr=0x%08x\n",
           esf->basic.r12, esf->basic.lr, esf->basic.pc, esf->basic.xpsr);
  }
  printk("[FATAL] Rebooting in 3 seconds...\n");

#if (CONFIG_DEBUG)
  LOG_ERR("Caught system error -- reason %d", reason);
  LOG_PANIC();
#endif

  /* Delay to allow RTT to flush, then reboot to avoid locking SWD */
  k_busy_wait(3000000); /* 3 seconds */
  sys_reboot(SYS_REBOOT_COLD);

  CODE_UNREACHABLE;
}

void bt_ctlr_assert_handle(char *c, int code) {
  LOG_ERR("BT controller assert: %s, code: 0x%x", c, code);
  error_handler(code, NULL);
}

void k_sys_fatal_error_handler(unsigned int reason,
                               const struct arch_esf *esf) {
  error_handler(reason, esf);
}

void assert_post_action(const char *file, unsigned int line) {
  LOG_ERR("Assert post action: file: %s, line %d", file, line);
  error_handler(0, NULL);
}
