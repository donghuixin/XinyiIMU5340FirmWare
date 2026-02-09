/*
 * Meow Sense Tag �?BLE Control Service + USB CDC Serial Console (v4)
 *
 * A self-contained module that lets a phone app (via BLE) OR a serial
 * terminal (via USB CDC ACM virtual COM port) control IMU data streaming
 * and query battery status.
 *
 * NOTE: The Meow Sense Tag has NO hardware UART routed to any connector.
 *       UART0 TX (P1.04) is not connected on the PCB.
 *       All serial I/O goes through USB CDC ACM (cdc_acm_uart0).
 *       Plug in USB cable �?open COM port in a terminal (115200 8N1).
 *
 * On boot:
 *   - Waits up to 3 s for a USB terminal to connect (DTR)
 *   - Prints I2C device scan results (bmx160, BQ27220, BQ25120a, KTD2026)
 *   - Reports device addresses and whether each was found
 *
 * Serial commands (type + Enter):
 *   s   �?Start IMU streaming
 *   p   �?Stop IMU streaming
 *   b   �?Query battery
 *   bat �?Query battery (alias)
 *
 * BLE GATT:
 *   Service  UUID: a0e3d901-0c1f-4b5e-8e4a-1a2b3c4d5e6f
 *   Command  UUID: a0e3d902-... (Write)
 *   IMU data UUID: a0e3d903-... (Notify, header 0xAA 0x55)
 *   Battery  UUID: a0e3d904-... (Read + Notify, header 0xBB 0x66)
 */

#include "meow_ctrl_service.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/logging/log.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>

/* Reuse existing drivers */
#include "../../SensorManager/BMX160/DFRobot_BMX160.h"
#include "../../Battery/BQ27220.h"
#include "../../Battery/BQ25120a.h"
#include "openearable_common.h"
#include <TWIM.h>

LOG_MODULE_REGISTER(meow_ctrl, CONFIG_MAIN_LOG_LEVEL);

/* ================================================================== */
/*  UART serial output helpers                                         */
/* ================================================================== */
static const struct device *uart_dev;

/** Print a null-terminated string to UART (blocking, char-by-char). */
static void uart_puts(const char *s)
{
    if (!uart_dev) return;
    while (*s) {
        uart_poll_out(uart_dev, *s++);
    }
}

/** printf-style output to UART0. Max 256 chars per call. */
static void uart_printf(const char *fmt, ...)
{
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    uart_puts(buf);
}

/* ================================================================== */
/*  I2C device probing                                                 */
/* ================================================================== */

/** Probe an I2C address �?returns true if the device ACKs. */
static bool i2c_probe_addr(TWIM *bus, uint8_t addr)
{
    uint8_t dummy;
    bus->aquire();
    int ret = i2c_read(bus->master, &dummy, 1, addr);
    bus->release();
    return (ret == 0);
}

/** Scan known Meow Sense Tag I2C devices and print results to UART. */
static void meow_i2c_scan(void)
{
    /* Make sure I2C buses are initialised (idempotent) */
    I2C1.begin();
    I2C2.begin();

    /* Enable 1.8 V rail so we can probe bmx160 on I2C2 */
    int rc = pm_device_runtime_get(ls_1_8);
    if (rc && rc != -EALREADY) {
        uart_printf("[SCAN] WARN: 1.8V rail enable failed: %d\r\n", rc);
    }
    k_msleep(10);   /* wait for power rail + bmx160 POR */

    uart_printf("\r\n");
    uart_printf("============================================\r\n");
    uart_printf("  Meow Sense Tag �?I2C Device Scan\r\n");
    uart_printf("============================================\r\n");

    /* bmx160 (IMU) �?I2C2 @ 0x68 */
    bool bmi_ok = i2c_probe_addr(&I2C2, 0x68);
    uart_printf("  [I2C2] bmx160  (IMU)         @ 0x68 : %s\r\n",
                bmi_ok ? "FOUND" : "NOT FOUND");

    /* BQ27220 (fuel gauge) �?I2C1 @ 0x55 */
    bool bq27_ok = i2c_probe_addr(&I2C1, 0x55);
    uart_printf("  [I2C1] BQ27220 (Fuel Gauge)  @ 0x55 : %s\r\n",
                bq27_ok ? "FOUND" : "NOT FOUND");

    /* BQ25120a (charger) �?I2C1 @ 0x6A */
    bool bq25_ok = i2c_probe_addr(&I2C1, 0x6A);
    uart_printf("  [I2C1] BQ25120a(Charger)     @ 0x6A : %s\r\n",
                bq25_ok ? "FOUND" : "NOT FOUND");

    /* KTD2026 (LED) �?I2C1 @ 0x30 */
    bool ktd_ok = i2c_probe_addr(&I2C1, 0x30);
    uart_printf("  [I2C1] KTD2026 (LED)         @ 0x30 : %s\r\n",
                ktd_ok ? "FOUND" : "NOT FOUND");

    uart_printf("============================================\r\n");
    uart_printf("  Devices found: %d / 4\r\n",
                (int)bmi_ok + (int)bq27_ok + (int)bq25_ok + (int)ktd_ok);
    uart_printf("============================================\r\n\r\n");

    /* Release 1.8 V rail (will be re-enabled when 's' command arrives) */
    pm_device_runtime_put(ls_1_8);
}

/* ================================================================== */
/*  BLE UUIDs                                                          */
/* ================================================================== */
#define BT_UUID_MEOW_SVC_VAL \
    BT_UUID_128_ENCODE(0xa0e3d901, 0x0c1f, 0x4b5e, 0x8e4a, 0x1a2b3c4d5e6f)
#define BT_UUID_MEOW_CMD_VAL \
    BT_UUID_128_ENCODE(0xa0e3d902, 0x0c1f, 0x4b5e, 0x8e4a, 0x1a2b3c4d5e6f)
#define BT_UUID_MEOW_IMU_VAL \
    BT_UUID_128_ENCODE(0xa0e3d903, 0x0c1f, 0x4b5e, 0x8e4a, 0x1a2b3c4d5e6f)
#define BT_UUID_MEOW_BAT_VAL \
    BT_UUID_128_ENCODE(0xa0e3d904, 0x0c1f, 0x4b5e, 0x8e4a, 0x1a2b3c4d5e6f)

#define BT_UUID_MEOW_SVC   BT_UUID_DECLARE_128(BT_UUID_MEOW_SVC_VAL)
#define BT_UUID_MEOW_CMD   BT_UUID_DECLARE_128(BT_UUID_MEOW_CMD_VAL)
#define BT_UUID_MEOW_IMU   BT_UUID_DECLARE_128(BT_UUID_MEOW_IMU_VAL)
#define BT_UUID_MEOW_BAT   BT_UUID_DECLARE_128(BT_UUID_MEOW_BAT_VAL)

/* ================================================================== */
/*  Packet formats                                                     */
/* ================================================================== */
#define IMU_BATCH_SIZE     5     /* 5 samples × 36 bytes + 2 hdr = 182 B */
#define IMU_FLOATS_PER_S   9    /* ax,ay,az,gx,gy,gz,mx,my,mz       */
#define IMU_SAMPLE_BYTES   (IMU_FLOATS_PER_S * sizeof(float))  /* 36 */
#define IMU_HEADER_SIZE    2    /* 0xAA 0x55                         */
#define IMU_PKT_SIZE       (IMU_HEADER_SIZE + IMU_BATCH_SIZE * IMU_SAMPLE_BYTES)

#define BAT_PKT_SIZE       6    /* 0xBB 0x66 + mV(2) + soc(1) + chg(1) */

#define IMU_SAMPLE_PERIOD_US  9615   /* ~104 Hz */

/* ================================================================== */
/*  Static state                                                       */
/* ================================================================== */
static uint8_t imu_pkt[IMU_PKT_SIZE];
static uint8_t bat_pkt[BAT_PKT_SIZE];
static volatile int imu_sample_idx;

static DFRobot_BMI160 bmx160(&I2C2);
static bool bmx160_active;

static struct k_timer  imu_timer;
static struct k_work   imu_work;
static struct k_work   bat_work;
static struct k_mutex  notify_mutex;

/* How many batches between serial output lines (1 = every batch �?8 ms) */
#define SERIAL_IMU_PRINT_DIVIDER  4   /* print every 4th batch �?5 Hz */
static int serial_batch_counter;

/* ================================================================== */
/*  Forward declarations                                               */
/* ================================================================== */
static ssize_t write_cmd(struct bt_conn *conn,
                         const struct bt_gatt_attr *attr,
                         const void *buf, uint16_t len,
                         uint16_t offset, uint8_t flags);
static ssize_t read_bat(struct bt_conn *conn,
                        const struct bt_gatt_attr *attr,
                        void *buf, uint16_t len, uint16_t offset);
static void imu_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value);
static void bat_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value);

/* ================================================================== */
/*  GATT service definition                                            */
/*  Attr layout: [0]svc [1]cmd-decl [2]cmd-val [3]imu-decl [4]imu-val */
/*               [5]imu-ccc [6]bat-decl [7]bat-val [8]bat-ccc         */
/* ================================================================== */
BT_GATT_SERVICE_DEFINE(meow_ctrl_svc,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_MEOW_SVC),

    BT_GATT_CHARACTERISTIC(BT_UUID_MEOW_CMD,
                           BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                           BT_GATT_PERM_WRITE,
                           NULL, write_cmd, NULL),

    BT_GATT_CHARACTERISTIC(BT_UUID_MEOW_IMU,
                           BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_NONE,
                           NULL, NULL, NULL),
    BT_GATT_CCC(imu_ccc_changed,
                 BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    BT_GATT_CHARACTERISTIC(BT_UUID_MEOW_BAT,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ,
                           read_bat, NULL, NULL),
    BT_GATT_CCC(bat_ccc_changed,
                 BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

#define IMU_NOTIFY_ATTR  (&meow_ctrl_svc.attrs[4])
#define BAT_NOTIFY_ATTR  (&meow_ctrl_svc.attrs[7])

/* ================================================================== */
/*  CCC callbacks                                                      */
/* ================================================================== */
static void imu_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    LOG_INF("IMU CCC �?0x%04x", value);
}
static void bat_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    LOG_INF("BAT CCC �?0x%04x", value);
}

/* ================================================================== */
/*  Battery helpers                                                    */
/* ================================================================== */
static void fill_bat_pkt(void)
{
    float voltage_v = fuel_gauge.voltage();
    float soc       = fuel_gauge.state_of_charge();
    bool  charging  = battery_controller.power_connected();

    uint16_t voltage_mV = (uint16_t)(voltage_v * 1000.0f);
    uint8_t  soc_pct    = (uint8_t)soc;

    bat_pkt[0] = 0xBB;
    bat_pkt[1] = 0x66;
    bat_pkt[2] = (uint8_t)(voltage_mV & 0xFF);
    bat_pkt[3] = (uint8_t)(voltage_mV >> 8);
    bat_pkt[4] = soc_pct;
    bat_pkt[5] = charging ? 1 : 0;

    /* Print to serial */
    uart_printf("[BAT] Voltage=%u mV  SOC=%u%%  Charging=%s\r\n",
                voltage_mV, (unsigned)soc_pct, charging ? "YES" : "NO");
}

static ssize_t read_bat(struct bt_conn *conn,
                        const struct bt_gatt_attr *attr,
                        void *buf, uint16_t len, uint16_t offset)
{
    fill_bat_pkt();
    return bt_gatt_attr_read(conn, attr, buf, len, offset, bat_pkt, BAT_PKT_SIZE);
}

/* ================================================================== */
/*  IMU start / stop                                                   */
/* ================================================================== */
static int imu_start(void)
{
    if (bmx160_active) {
        uart_printf("[IMU] Already running\r\n");
        return 0;
    }

    uart_printf("[IMU] Enabling 1.8V rail...\r\n");
    int ret = pm_device_runtime_get(ls_1_8);
    if (ret && ret != -EALREADY) {
        uart_printf("[IMU] ERROR: 1.8V rail failed: %d\r\n", ret);
        return ret;
    }

    k_msleep(10);   /* POR time for bmx160 */

    bool ok = false;
    for (int attempt = 1; attempt <= 3; attempt++) {
        uart_printf("[IMU] bmx160 begin() attempt %d...\r\n", attempt);
        if (bmx160.begin()) {
            ok = true;
            uart_printf("[IMU] bmx160 init OK (attempt %d)\r\n", attempt);
            break;
        }
        uart_printf("[IMU] bmx160 begin() FAILED (attempt %d/3)\r\n", attempt);
        k_msleep(50);
    }

    if (!ok) {
        uart_printf("[IMU] ERROR: bmx160 init FAILED after 3 attempts\r\n");
        pm_device_runtime_put(ls_1_8);
        return -EIO;
    }

    bmx160.setAccelRange(eAccelRange_2G);
    bmx160.setAccelODR(BMX160_ACCEL_ODR_100HZ);

    imu_pkt[0] = 0xAA;
    imu_pkt[1] = 0x55;
    imu_sample_idx = 0;
    serial_batch_counter = 0;
    bmx160_active = true;

    k_timer_start(&imu_timer, K_NO_WAIT, K_USEC(IMU_SAMPLE_PERIOD_US));

    uart_printf("[IMU] Streaming started (104Hz, batch=%d, pkt=%dB)\r\n",
                IMU_BATCH_SIZE, IMU_PKT_SIZE);
    return 0;
}

static void imu_stop(void)
{
    if (!bmx160_active) {
        uart_printf("[IMU] Already stopped\r\n");
        return;
    }

    k_timer_stop(&imu_timer);
    bmx160.setLowPower();
    bmx160_active = false;
    pm_device_runtime_put(ls_1_8);

    uart_printf("[IMU] Streaming stopped, low-power mode\r\n");
}

/* ================================================================== */
/*  Timer �?work queue                                                 */
/* ================================================================== */
static void imu_timer_handler(struct k_timer *timer)
{
    k_work_submit(&imu_work);
}

static void imu_work_handler(struct k_work *work)
{
    sBmx160SensorData_t mag, gyr, acc;
    bmx160.getAllData(&mag, &gyr, &acc);

    uint8_t *dst = &imu_pkt[IMU_HEADER_SIZE + imu_sample_idx * IMU_SAMPLE_BYTES];
    float sample[IMU_FLOATS_PER_S] = {
        acc.x, acc.y, acc.z,
        gyr.x, gyr.y, gyr.z,
        mag.x, mag.y, mag.z
    };
    memcpy(dst, sample, IMU_SAMPLE_BYTES);

    imu_sample_idx++;

    if (imu_sample_idx >= IMU_BATCH_SIZE) {
        imu_sample_idx = 0;

        /* BLE notify */
        if (k_mutex_lock(&notify_mutex, K_MSEC(20)) == 0) {
            bt_gatt_notify(NULL, IMU_NOTIFY_ATTR, imu_pkt, IMU_PKT_SIZE);
            k_mutex_unlock(&notify_mutex);
        }

        /* Serial output at reduced rate */
        serial_batch_counter++;
        if (serial_batch_counter >= SERIAL_IMU_PRINT_DIVIDER) {
            serial_batch_counter = 0;
            /* Print last sample in the batch using integer-scaled values
             * to avoid %f (saves flash, works without CBPRINTF_FP_SUPPORT).
             * Multiply by 1000 �?milli-units. */
            int ax = (int)(acc.x * 1000);
            int ay = (int)(acc.y * 1000);
            int az = (int)(acc.z * 1000);
            int gx = (int)(gyr.x * 1000);
            int gy = (int)(gyr.y * 1000);
            int gz = (int)(gyr.z * 1000);
            int mx = (int)(mag.x * 1000);
            int my = (int)(mag.y * 1000);
            int mz = (int)(mag.z * 1000);
            uart_printf("[IMU] A:%d,%d,%d G:%d,%d,%d M:%d,%d,%d (x1000)\r\n",
                        ax, ay, az, gx, gy, gz, mx, my, mz);
        }
    }
}

/* ================================================================== */
/*  Battery work handler                                               */
/* ================================================================== */
static void bat_work_handler(struct k_work *work)
{
    fill_bat_pkt();

    if (k_mutex_lock(&notify_mutex, K_MSEC(50)) == 0) {
        int ret = bt_gatt_notify(NULL, BAT_NOTIFY_ATTR,
                                 bat_pkt, BAT_PKT_SIZE);
        k_mutex_unlock(&notify_mutex);
        if (ret == -ENOTCONN) {
            /* no BLE subscriber �?fine, serial already printed */
        } else if (ret) {
            LOG_WRN("Battery notify err: %d", ret);
        }
    }
}

/* ================================================================== */
/*  Shared command processor (used by both BLE write and UART RX)      */
/* ================================================================== */
static void process_command(const char *cmd_str, int len, const char *source)
{
    if (len < 1) return;

    char c0 = cmd_str[0];

    uart_printf("[CMD] '%.*s' (from %s)\r\n", len, cmd_str, source);

    if (c0 == 's' || c0 == 'S') {
        imu_start();
    } else if (c0 == 'p' || c0 == 'P') {
        imu_stop();
    } else if (c0 == 'b' || c0 == 'B') {
        /* 'b', 'B', or 'bat'/'BAT' */
        k_work_submit(&bat_work);
    } else {
        uart_printf("[CMD] Unknown: '%.*s'\r\n", len, cmd_str);
    }
}

/* ================================================================== */
/*  BLE Command write handler                                          */
/* ================================================================== */
static ssize_t write_cmd(struct bt_conn *conn,
                         const struct bt_gatt_attr *attr,
                         const void *buf, uint16_t len,
                         uint16_t offset, uint8_t flags)
{
    if (len < 1 || offset != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    /* Null-terminate for process_command */
    char tmp[16];
    int clen = (len < (int)sizeof(tmp) - 1) ? len : (int)sizeof(tmp) - 1;
    memcpy(tmp, buf, clen);
    tmp[clen] = '\0';

    process_command(tmp, clen, "BLE");

    return len;
}

/* ================================================================== */
/*  UART RX polling thread                                             */
/* ================================================================== */
#define UART_RX_BUF_SIZE  32
static char uart_rx_buf[UART_RX_BUF_SIZE];
static int  uart_rx_pos;

static K_THREAD_STACK_DEFINE(uart_rx_stack, 1024);
static struct k_thread uart_rx_thread_data;

static void uart_rx_thread_fn(void *p1, void *p2, void *p3)
{
    uart_printf("\r\n[USB] Meow Sense Tag serial ready (USB CDC ACM)\r\n");
    uart_printf("[USB] Commands: s=start IMU, p=stop IMU, b/bat=battery\r\n");
    uart_printf("> ");

    uart_rx_pos = 0;

    while (1) {
        uint8_t c;
        int ret = uart_poll_in(uart_dev, &c);
        if (ret != 0) {
            k_msleep(10);  /* no data, yield */
            continue;
        }

        /* Echo character back */
        uart_poll_out(uart_dev, c);

        if (c == '\r' || c == '\n') {
            /* Print newline */
            uart_puts("\r\n");

            if (uart_rx_pos > 0) {
                uart_rx_buf[uart_rx_pos] = '\0';

                /* Trim trailing spaces */
                while (uart_rx_pos > 0 && uart_rx_buf[uart_rx_pos - 1] == ' ') {
                    uart_rx_buf[--uart_rx_pos] = '\0';
                }

                if (uart_rx_pos > 0) {
                    process_command(uart_rx_buf, uart_rx_pos, "UART");
                }
            }

            uart_rx_pos = 0;
            uart_printf("> ");
        } else if (c == 0x7F || c == '\b') {
            /* Backspace */
            if (uart_rx_pos > 0) {
                uart_rx_pos--;
                uart_puts("\b \b");
            }
        } else if (uart_rx_pos < UART_RX_BUF_SIZE - 1 && c >= 0x20) {
            uart_rx_buf[uart_rx_pos++] = (char)c;
        }
    }
}

/* ================================================================== */
/*  Initialization                                                     */
/* ================================================================== */
int init_meow_ctrl_service(void)
{
    /* ---- USB CDC ACM serial setup ---- */
    uart_dev = DEVICE_DT_GET(DT_NODELABEL(cdc_acm_uart0));
    if (!device_is_ready(uart_dev)) {
        LOG_ERR("CDC ACM UART not ready!");
        uart_dev = NULL;
    }

    /* Configure UART parameters (reported to host during CDC negotiation) */
    if (uart_dev) {
        const struct uart_config cfg = {
            .baudrate  = 115200,
            .parity    = UART_CFG_PARITY_NONE,
            .stop_bits = UART_CFG_STOP_BITS_1,
            .data_bits = UART_CFG_DATA_BITS_8,
            .flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
        };
        uart_configure(uart_dev, &cfg);
    }

    /* Wait for a USB terminal to open (DTR signal), up to 3 seconds.
     * If no terminal is connected, we still proceed �?serial output
     * will be lost until someone opens the COM port.                */
    if (uart_dev) {
        uint32_t dtr = 0;
        for (int i = 0; i < 300 && !dtr; i++) {
            uart_line_ctrl_get(uart_dev, UART_LINE_CTRL_DTR, &dtr);
            k_msleep(10);
        }
        if (!dtr) {
            LOG_WRN("CDC ACM: no terminal connected after 3 s, proceeding");
        }
        /* Small extra delay for terminal to finish setup */
        k_msleep(100);
    }

    uart_printf("\r\n");
    uart_printf("**********************************************\r\n");
    uart_printf("*       Meow Sense Tag  v4  Boot             *\r\n");
    uart_printf("*   USB CDC Serial  |  BLE: Meow Sense Tag   *\r\n");
    uart_printf("**********************************************\r\n");

    /* ---- I2C device scan ---- */
    meow_i2c_scan();

    /* ---- Kernel objects ---- */
    k_mutex_init(&notify_mutex);
    k_work_init(&imu_work, imu_work_handler);
    k_work_init(&bat_work, bat_work_handler);
    k_timer_init(&imu_timer, imu_timer_handler, NULL);

    bmx160_active = false;
    imu_sample_idx = 0;
    serial_batch_counter = 0;

    /* ---- Start UART RX thread ---- */
    if (uart_dev) {
        k_thread_create(&uart_rx_thread_data, uart_rx_stack,
                        K_THREAD_STACK_SIZEOF(uart_rx_stack),
                        uart_rx_thread_fn, NULL, NULL, NULL,
                        K_PRIO_PREEMPT(10), 0, K_NO_WAIT);
        k_thread_name_set(&uart_rx_thread_data, "meow_uart_rx");
    }

    uart_printf("[INIT] GATT service registered\r\n");
    uart_printf("[INIT] IMU batch=%d pkt=%dB | BAT pkt=%dB\r\n",
                IMU_BATCH_SIZE, IMU_PKT_SIZE, BAT_PKT_SIZE);

    return 0;
}
