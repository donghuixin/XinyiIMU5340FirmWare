#include "ChargingLED.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(charging_led, LOG_LEVEL_INF);

/* ---- Hardware ---- */
static const struct gpio_dt_spec led_pin =
    GPIO_DT_SPEC_GET(DT_ALIAS(charging_led), gpios);

/* ---- State ---- */
static enum charging_led_mode current_mode = CHARGING_LED_OFF;

/* ---- Blink timer (1-second period: 500 ms on / 500 ms off) ---- */
static void blink_timer_handler(struct k_timer *timer);
K_TIMER_DEFINE(blink_timer, blink_timer_handler, NULL);
static bool blink_state; /* true = LED on */

static void blink_timer_handler(struct k_timer *timer)
{
    blink_state = !blink_state;
    gpio_pin_set_dt(&led_pin, blink_state ? 1 : 0);
}

/* ---- Breathe timer (3-second period software PWM) ----
 *
 * We approximate a "breathing" effect using a fast timer (~10 ms tick) that
 * ramps a software duty-cycle up over 1.5 s then back down over 1.5 s.
 *
 * Total steps per half-cycle  = 1500 ms / 10 ms = 150
 * PWM sub-period              = 10 ms = 10 ticks of 1 ms
 *
 * At step N (0..149) during ramp-up:
 *   duty = N * 10 / 150  (0..~10, out of 10)
 * During ramp-down it mirrors.
 *
 * We use a second fast 1 ms timer for the PWM carrier.
 */
static void breathe_tick_handler(struct k_timer *timer);
K_TIMER_DEFINE(breathe_tick_timer, breathe_tick_handler, NULL);

#define BREATHE_HALF_PERIOD_MS  1500
#define BREATHE_STEP_MS         10
#define BREATHE_STEPS           (BREATHE_HALF_PERIOD_MS / BREATHE_STEP_MS) /* 150 */
#define PWM_SUB_PERIOD          10  /* sub-ticks per BREATHE_STEP_MS */

static uint16_t breathe_step;    /* 0 .. 2*BREATHE_STEPS-1 */

/* PWM carrier (1 ms tick) */
static void pwm_carrier_handler(struct k_timer *timer);
K_TIMER_DEFINE(pwm_carrier_timer, pwm_carrier_handler, NULL);
static uint8_t pwm_duty;    /* 0..PWM_SUB_PERIOD */
static uint8_t pwm_counter; /* 0..PWM_SUB_PERIOD-1 */

static void pwm_carrier_handler(struct k_timer *timer)
{
    gpio_pin_set_dt(&led_pin, (pwm_counter < pwm_duty) ? 1 : 0);
    pwm_counter++;
    if (pwm_counter >= PWM_SUB_PERIOD) {
        pwm_counter = 0;
    }
}

static void breathe_tick_handler(struct k_timer *timer)
{
    uint16_t step = breathe_step;
    uint16_t half = BREATHE_STEPS;
    uint16_t pos;

    if (step < half) {
        pos = step;            /* ramp up */
    } else {
        pos = 2 * half - step; /* ramp down */
    }

    /* Map pos (0..150) to duty (0..PWM_SUB_PERIOD) */
    pwm_duty = (uint8_t)((pos * PWM_SUB_PERIOD) / half);

    breathe_step++;
    if (breathe_step >= 2 * half) {
        breathe_step = 0;
    }
}

/* ---- Stop all timers and turn LED off ---- */
static void stop_all(void)
{
    k_timer_stop(&blink_timer);
    k_timer_stop(&breathe_tick_timer);
    k_timer_stop(&pwm_carrier_timer);
    gpio_pin_set_dt(&led_pin, 0);
    blink_state = false;
    breathe_step = 0;
    pwm_duty = 0;
    pwm_counter = 0;
}

/* ---- Public API ---- */

int charging_led_init(void)
{
    if (!gpio_is_ready_dt(&led_pin)) {
        LOG_ERR("P1.08 LED GPIO not ready");
        return -ENODEV;
    }

    int ret = gpio_pin_configure_dt(&led_pin, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        LOG_ERR("Failed to configure P1.08 LED: %d", ret);
        return ret;
    }

    LOG_INF("P1.08 charging LED initialized");
    return 0;
}

void charging_led_set_mode(enum charging_led_mode mode)
{
    if (mode == current_mode) {
        return;
    }

    stop_all();
    current_mode = mode;

    switch (mode) {
    case CHARGING_LED_OFF:
        LOG_INF("LED mode: OFF");
        /* Already off after stop_all() */
        break;

    case CHARGING_LED_BLINK_1S:
        LOG_INF("LED mode: BLINK 1s");
        blink_state = true;
        gpio_pin_set_dt(&led_pin, 1);
        k_timer_start(&blink_timer, K_MSEC(500), K_MSEC(500));
        break;

    case CHARGING_LED_BREATHE_3S:
        LOG_INF("LED mode: BREATHE 3s");
        breathe_step = 0;
        pwm_duty = 0;
        pwm_counter = 0;
        /* 1 ms carrier for PWM */
        k_timer_start(&pwm_carrier_timer, K_MSEC(1), K_MSEC(1));
        /* 10 ms tick to update duty cycle */
        k_timer_start(&breathe_tick_timer, K_MSEC(BREATHE_STEP_MS),
                       K_MSEC(BREATHE_STEP_MS));
        break;
    }
}
