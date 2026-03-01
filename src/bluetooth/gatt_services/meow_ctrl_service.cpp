/*
 * Meow Sense Tag – BLE Control Service + USB CDC Serial Console (v4 -
 * Refactored)
 *
 * A self-contained module that lets a phone app (via BLE) OR a serial
 * terminal (via USB CDC ACM virtual COM port) control IMU data streaming
 * and query battery status via System Modules (SensorManager, PowerManager).
 *
 * NOTE: The Meow Sense Tag has NO hardware UART routed to any connector.
 *       All serial I/O goes through USB CDC ACM (cdc_acm_uart0).
 *
 * On boot:
 *   - Waits up to 3 s for a USB terminal to connect (DTR)
 *   - Prints device status
 *
 * Serial commands (type + Enter):
 *   s   – Start IMU streaming (via SensorManager)
 *   p   – Stop IMU streaming (via SensorManager)
 *   b   – Query battery (via PowerManager)
 *   bat – Query battery (alias)
 *
 * BLE GATT:
 *   Service  UUID: a0e3d901-0c1f-4b5e-8e4a-1a2b3c4d5e6f
 *   Command  UUID: a0e3d902-... (Write)
 *   IMU data UUID: a0e3d903-... (Notify, header 0xAA 0x55)
 *   Battery  UUID: a0e3d904-... (Read + Notify, header 0xBB 0x66)
 */

#include "meow_ctrl_service.h"

#include <ctype.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/zbus/zbus.h>

/* Reuse existing drivers and managers */
#include "../../Battery/PowerManager.h" // For fuel_gauge and battery_controller
#include "../../ParseInfo/SensorScheme.h"      // For DATA_STREAMING flag
#include "../../SensorManager/SensorManager.h" // For config_sensor and sensor_chan
#include "openearable_common.h"                // For struct sensor_msg

LOG_MODULE_REGISTER(meow_ctrl, CONFIG_MAIN_LOG_LEVEL);

/* ================================================================== */
/*  UART serial output helpers                                         */
/* ================================================================== */
static const struct device *uart_dev;

/** Print a null-terminated string to UART (blocking, char-by-char). */
static void uart_puts(const char *s) {
  if (!uart_dev)
    return;
  while (*s) {
    uart_poll_out(uart_dev, *s++);
  }
}

/** printf-style output to UART0. Max 256 chars per call. */
static void uart_printf(const char *fmt, ...) {
  char buf[256];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  uart_puts(buf);
}

/* ================================================================== */
/*  I2C device probing (Skipped - Managed by System)                   */
/* ================================================================== */

/** Scan known Meow Sense Tag I2C devices and print results to UART. */
static void meow_i2c_scan(void) {
  uart_printf("\r\n");
  uart_printf("============================================\r\n");
  uart_printf("  Xinyi_IMU_Left_1 – System Check\r\n");
  uart_printf("============================================\r\n");
  uart_printf("  Sensors and Power managed by system modules.\r\n");
  uart_printf("============================================\r\n\r\n");
}

/* ================================================================== */
/*  BLE UUIDs                                                          */
/* ================================================================== */
#define BT_UUID_MEOW_SVC_VAL                                                   \
  BT_UUID_128_ENCODE(0xa0e3d901, 0x0c1f, 0x4b5e, 0x8e4a, 0x1a2b3c4d5e6f)
#define BT_UUID_MEOW_CMD_VAL                                                   \
  BT_UUID_128_ENCODE(0xa0e3d902, 0x0c1f, 0x4b5e, 0x8e4a, 0x1a2b3c4d5e6f)
#define BT_UUID_MEOW_IMU_VAL                                                   \
  BT_UUID_128_ENCODE(0xa0e3d903, 0x0c1f, 0x4b5e, 0x8e4a, 0x1a2b3c4d5e6f)
#define BT_UUID_MEOW_BAT_VAL                                                   \
  BT_UUID_128_ENCODE(0xa0e3d904, 0x0c1f, 0x4b5e, 0x8e4a, 0x1a2b3c4d5e6f)

#define BT_UUID_MEOW_SVC BT_UUID_DECLARE_128(BT_UUID_MEOW_SVC_VAL)
#define BT_UUID_MEOW_CMD BT_UUID_DECLARE_128(BT_UUID_MEOW_CMD_VAL)
#define BT_UUID_MEOW_IMU BT_UUID_DECLARE_128(BT_UUID_MEOW_IMU_VAL)
#define BT_UUID_MEOW_BAT BT_UUID_DECLARE_128(BT_UUID_MEOW_BAT_VAL)

/* ================================================================== */
/*  Packet formats  (match Arduino Xinyi_IMU protocol)                 */
/* ================================================================== */

/* IMU sample: 6-axis raw int16_t (accel + gyro) */
struct __attribute__((packed)) IMUSample {
  int16_t ax, ay, az; /* accelerometer raw */
  int16_t gx, gy, gz; /* gyroscope raw     */
};

/* Full IMU packet: header + timestamp + 10 samples = 2+4+120 = 126 B */
#define IMU_BATCH_SIZE 10
struct __attribute__((packed)) FullPacket {
  uint8_t header[2];  /* 0xAA 0x55              */
  uint32_t timestamp; /* microseconds           */
  struct IMUSample samples[IMU_BATCH_SIZE];
};
#define IMU_PKT_SIZE sizeof(struct FullPacket) /* 126 */

/* Battery packet: header + voltage(float) + isCharging + SOC = 8 B */
struct __attribute__((packed)) BatteryPacket {
  uint8_t header[2];    /* 0xBB 0x66              */
  float voltage;        /* battery voltage in V   */
  uint8_t isCharging;   /* 1=charging, 0=not      */
  uint8_t batteryLevel; /* 0-100 SOC %            */
};
#define BAT_PKT_SIZE sizeof(struct BatteryPacket) /* 8 */

/* ================================================================== */
/*  Static state                                                       */
/* ================================================================== */
static struct FullPacket imu_pkt;
static struct BatteryPacket bat_pkt;
static volatile int imu_sample_idx;

/* How many batches between serial output lines (1 = every batch – 8 ms) */
#define SERIAL_IMU_PRINT_DIVIDER 4 /* print every 4th batch – 5 Hz */
static int serial_batch_counter;

/* Command flags for deferring BLE commands to serial_thread to avoid deadlock
 */
static volatile int pending_ble_cmd = 0; // 0=none, 1=start, 2=stop, 3=bat

/* Conversion constants (approximate based on standard settings) */
/* Accel: 2G range -> 16384 LSB/g. 1g = 9.81 m/s^2. Factor = 16384 / 9.81 =
 * 1670.1 */
#define ACCEL_SCALE_FACTOR 1670.1f
/* Gyro: 250dps range -> 131.2 LSB/dps. 1dps = 1 degree/s. Factor = 131.2 */
#define GYRO_SCALE_FACTOR 131.2f

/* ================================================================== */
/*  Forward declarations                                               */
/* ================================================================== */
static ssize_t write_cmd(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                         const void *buf, uint16_t len, uint16_t offset,
                         uint8_t flags);
static ssize_t read_bat(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                        void *buf, uint16_t len, uint16_t offset);
static void imu_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value);
static void bat_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value);

/* ================================================================== */
/*  GATT service definition                                            */
/* ================================================================== */
BT_GATT_SERVICE_DEFINE(
    meow_ctrl_svc, BT_GATT_PRIMARY_SERVICE(BT_UUID_MEOW_SVC),

    BT_GATT_CHARACTERISTIC(BT_UUID_MEOW_CMD,
                           BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                           BT_GATT_PERM_WRITE, NULL, write_cmd, NULL),

    BT_GATT_CHARACTERISTIC(BT_UUID_MEOW_IMU, BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_NONE, NULL, NULL, NULL),
    BT_GATT_CCC(imu_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    BT_GATT_CHARACTERISTIC(BT_UUID_MEOW_BAT,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ, read_bat, NULL, NULL),
    BT_GATT_CCC(bat_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE), );

#define IMU_NOTIFY_ATTR (&meow_ctrl_svc.attrs[4])
#define BAT_NOTIFY_ATTR (&meow_ctrl_svc.attrs[7])

/* ================================================================== */
/*  CCC callbacks                                                      */
/* ================================================================== */
static void imu_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value) {
  LOG_INF("IMU CCC – 0x%04x", value);
}
static void bat_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value) {
  LOG_INF("BAT CCC – 0x%04x", value);
}

/* ================================================================== */
/*  Battery helpers                                                    */
/* ================================================================== */
static void fill_bat_pkt(void) {
  /* Use global fuel_gauge from PowerManager */
  float voltage_v = fuel_gauge.voltage();
  float soc = fuel_gauge.state_of_charge();
  bool charging = battery_controller.power_connected();

  bat_pkt.header[0] = 0xBB;
  bat_pkt.header[1] = 0x66;
  bat_pkt.voltage = voltage_v;
  bat_pkt.isCharging = charging ? 1 : 0;
  bat_pkt.batteryLevel = (uint8_t)soc;

  uint16_t voltage_mV = (uint16_t)(voltage_v * 1000.0f);
  uart_printf("[BAT] Voltage=%u mV  SOC=%u%%  Charging=%s\r\n", voltage_mV,
              (unsigned)bat_pkt.batteryLevel, charging ? "YES" : "NO");
}

static ssize_t read_bat(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                        void *buf, uint16_t len, uint16_t offset) {
  fill_bat_pkt();
  return bt_gatt_attr_read(conn, attr, buf, len, offset, &bat_pkt,
                           BAT_PKT_SIZE);
}

/* ================================================================== */
/*  IMU start / stop (via SensorManager)                               */
/* ================================================================== */
static int imu_start(void) {
  uart_printf("[IMU] Starting via SensorManager...\r\n");

  /* Configure IMU (ID=0) with sample rate index 2 (100Hz) and streaming enabled
   */
  struct sensor_config config;
  config.sensorId = ID_IMU;
  config.sampleRateIndex = 2;             // 100Hz
  config.storageOptions = DATA_STREAMING; // Enable BLE streaming

  config_sensor(&config);

  /* Reset batch index */
  imu_sample_idx = 0;

  return 0;
}

static void imu_stop(void) {
  uart_printf("[IMU] Stopping via SensorManager...\r\n");

  /* Configure IMU (ID=0) with streaming disabled */
  struct sensor_config config;
  config.sensorId = ID_IMU;
  config.sampleRateIndex = 0;
  config.storageOptions = 0; // Disable everything
  config_sensor(&config);
}

/* ================================================================== */
/*  Zbus Handler for Sensor Data                                       */
/* ================================================================== */
static void meow_ctrl_zbus_handler(const struct zbus_channel *chan) {
  const struct sensor_msg *msg;
  msg = (const struct sensor_msg *)zbus_chan_const_msg(chan);
  if (!msg) {
    return;
  }

  /* We only care about IMU data for now */
  if (msg->data.id == ID_IMU) {
    /* Parse float data from SensorManager: Accel(3*float) + Gyro(3*float) +
     * Mag(3*float) */
    /* But we only want Accel + Gyro for the Meow packet format */

    float ax_f, ay_f, az_f;
    float gx_f, gy_f, gz_f;

    /* Data layout in msg->data.data:
       0-3: ax, 4-7: ay, 8-11: az
       12-15: gx, 16-19: gy, 20-23: gz
       24-35: mag (ignored)
    */

    memcpy(&ax_f, &msg->data.data[0], sizeof(float));
    memcpy(&ay_f, &msg->data.data[4], sizeof(float));
    memcpy(&az_f, &msg->data.data[8], sizeof(float));

    memcpy(&gx_f, &msg->data.data[12], sizeof(float));
    memcpy(&gy_f, &msg->data.data[16], sizeof(float));
    memcpy(&gz_f, &msg->data.data[20], sizeof(float));

    /* Convert to int16_t */
    int16_t ax = (int16_t)(ax_f * ACCEL_SCALE_FACTOR);
    int16_t ay = (int16_t)(ay_f * ACCEL_SCALE_FACTOR);
    int16_t az = (int16_t)(az_f * ACCEL_SCALE_FACTOR);

    int16_t gx = (int16_t)(gx_f * GYRO_SCALE_FACTOR);
    int16_t gy = (int16_t)(gy_f * GYRO_SCALE_FACTOR);
    int16_t gz = (int16_t)(gz_f * GYRO_SCALE_FACTOR);

    /* Add to batch */
    int idx = imu_sample_idx;
    if (idx < IMU_BATCH_SIZE) {
      imu_pkt.samples[idx].ax = ax;
      imu_pkt.samples[idx].ay = ay;
      imu_pkt.samples[idx].az = az;
      imu_pkt.samples[idx].gx = gx;
      imu_pkt.samples[idx].gy = gy;
      imu_pkt.samples[idx].gz = gz;
      imu_sample_idx++;
    }

    /* If batch full, send notification */
    if (imu_sample_idx >= IMU_BATCH_SIZE) {
      imu_pkt.header[0] = 0xAA;
      imu_pkt.header[1] = 0x55;
      imu_pkt.timestamp = (uint32_t)msg->data.time; // Use sensor timestamp

      /* Send BLE notification */
      int ret = bt_gatt_notify(NULL, IMU_NOTIFY_ATTR, &imu_pkt, IMU_PKT_SIZE);
      if (ret && ret != -ENOTCONN) {
        LOG_WRN("IMU notify failed: %d", ret);
      }

      /* Print to serial occasionally */
      serial_batch_counter++;
      if (serial_batch_counter >= SERIAL_IMU_PRINT_DIVIDER) {
        serial_batch_counter = 0;
        // Print the first sample of the batch
        uart_printf("AX:%5d AY:%5d AZ:%5d GX:%5d GY:%5d GZ:%5d\r\n", ax, ay, az,
                    gx, gy, gz);
      }

      /* Reset batch */
      imu_sample_idx = 0;
    }
  }
}

ZBUS_LISTENER_DEFINE(meow_ctrl_lis, meow_ctrl_zbus_handler);
ZBUS_CHAN_ADD_OBS(sensor_chan, meow_ctrl_lis, 2);

/* ================================================================== */
/*  Command handler (BLE Write)                                        */
/* ================================================================== */
static ssize_t write_cmd(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                         const void *buf, uint16_t len, uint16_t offset,
                         uint8_t flags) {
  printk("RAW BLE WRITE TRIGGERED! Data: %c\n",
         len > 0 ? ((const char *)buf)[0] : ' ');
  LOG_INF("BLE Write received! Length: %d", len); // 看看有没有收到包

  const char *data = (const char *)buf;
  if (len == 0)
    return len;

  char cmd = data[0];
  LOG_INF("Received command: %c", cmd);
  uart_printf("[CMD] Received: %c\r\n", cmd);

  switch (cmd) {
  case 's':
    LOG_INF("Command 's' matched! Deferring IMU thread start...");
    pending_ble_cmd = 1;
    break;
  case 'p':
    pending_ble_cmd = 2;
    break;
  case 'b':
    pending_ble_cmd = 3;
    break;
  /* ==== RGB 颜色指令 ==== */
  case 'R':
    pending_ble_cmd = 20;
    break; // 红色 (Red)
  case 'G':
    pending_ble_cmd = 21;
    break; // 绿色 (Green)
  case 'B':
    pending_ble_cmd = 22;
    break; // 蓝色 (Blue)
  case 'Y':
    pending_ble_cmd = 23;
    break; // 黄色 (Yellow = R+G)
  case 'P':
    pending_ble_cmd = 24;
    break; // 紫色 (Purple = R+B)
  case 'C':
    pending_ble_cmd = 25;
    break; // 青色 (Cyan = G+B)
  case 'W':
    pending_ble_cmd = 26;
    break; // 白色 (White = R+G+B)
  case 'O':
    pending_ble_cmd = 27;
    break; // 关闭 (Off)
  /* ====================== */
  default:
    uart_printf("[CMD] Unknown command: %c\r\n", cmd);
    break;
  }

  return len;
}

/* ================================================================== */
/*  USB Serial Thread                                                  */
/* ================================================================== */
static void serial_thread(void) {
  /* Wait for DTR (terminal connected) */
  uart_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
  if (!device_is_ready(uart_dev)) {
    LOG_ERR("UART device not found!");
    return;
  }

  uint32_t dtr = 0;
  for (int i = 0; i < 30; i++) {
    uart_line_ctrl_get(uart_dev, UART_LINE_CTRL_DTR, &dtr);
    if (dtr)
      break;
    k_msleep(100);
  }

  if (dtr) {
    uart_printf("USB Serial Connected!\r\n");
  }

  /* Scan I2C devices */
  meow_i2c_scan();

  // 新增：记录上一次执行电量检查的时间戳
  int64_t last_bat_check_time = k_uptime_get();

  /* Main loop: read chars from UART and run pending BLE commands */
  while (1) {
    if (pending_ble_cmd == 1) {
      pending_ble_cmd = 0;
      imu_start();
    } else if (pending_ble_cmd == 2) {
      pending_ble_cmd = 0;
      imu_stop();
    } else if (pending_ble_cmd == 3) {
      pending_ble_cmd = 0;
      fill_bat_pkt();
      bt_gatt_notify(NULL, BAT_NOTIFY_ATTR, &bat_pkt, BAT_PKT_SIZE);
    } else if (pending_ble_cmd >= 20 && pending_ble_cmd <= 27) {
      int color_cmd = pending_ble_cmd;
      pending_ble_cmd = 0;

      const struct device *i2c1_dev = DEVICE_DT_GET(DT_NODELABEL(i2c1));
      if (device_is_ready(i2c1_dev)) {
        uint8_t r = 0, g = 0, b = 0;
        uint8_t bright = 0x55;

        switch (color_cmd) {
        case 20:
          r = bright;
          break; // 红
        case 21:
          g = bright;
          break; // 绿
        case 22:
          b = bright;
          break; // 蓝
        case 23:
          r = bright;
          g = bright;
          break; // 黄
        case 24:
          r = bright;
          b = bright;
          break; // 紫
        case 25:
          g = bright;
          b = bright;
          break; // 青
        case 26:
          r = bright;
          g = bright;
          b = bright;
          break; // 白
        case 27:
          break; // 关
        }

        if (color_cmd == 27) {
          i2c_reg_write_byte(i2c1_dev, 0x30, 0x00, 0x00); // 芯片休眠关闭
          uart_printf("[LED] Turned OFF\r\n");
        } else {
          i2c_reg_write_byte(i2c1_dev, 0x30, 0x00,
                             0x08); // 唤醒芯片进入Normal模式 (Bit 4:3 = 01)
          i2c_reg_write_byte(i2c1_dev, 0x30, 0x06, r); // I_R (Red)
          i2c_reg_write_byte(i2c1_dev, 0x30, 0x07, g); // I_G (Green)
          i2c_reg_write_byte(i2c1_dev, 0x30, 0x08, b); // I_B (Blue)
          i2c_reg_write_byte(i2c1_dev, 0x30, 0x04,
                             0x15); // EN_CH (010101: All CH Linear Mode)
          uart_printf("[LED] Color Set (R:%02X, G:%02X, B:%02X)\r\n", r, g, b);
        }
      }
    }
  }
}

/* ========================================================== */
/* ==== 新增：低电量 (<30%) 每 5 秒主动推送机制 ==== */
/* ========================================================== */
int64_t now = k_uptime_get();
if (now - last_bat_check_time >= 5000) { // 5000 毫秒 = 5 秒
  last_bat_check_time = now;             // 重置计时器

  // 查询电量计当前的真实 SOC (State of Charge)
  float current_soc = fuel_gauge.state_of_charge();

  // 如果电量低于 30%，则主动打包并发送蓝牙 Notify
  if (current_soc < 30.0f) {
    fill_bat_pkt(); // 调用你已有的函数，更新 bat_pkt 结构体

    // 发送给所有订阅了电量特征值的蓝牙主机 (网页)
    int ret = bt_gatt_notify(NULL, BAT_NOTIFY_ATTR, &bat_pkt, BAT_PKT_SIZE);
    if (ret == 0) {
      // 可选：在串口也打印一个低电量警告
      uart_printf("[WARNING] Battery low (%.1f%%), auto-notified via BLE!\r\n",
                  current_soc);
    }
  }
}
/* ========================================================== */

unsigned char c;
if (uart_poll_in(uart_dev, &c) == 0) {
  if (c == 's') {
    imu_start();
  } else if (c == 'p') {
    imu_stop();
  } else if (c == 'b') {
    fill_bat_pkt();
    bt_gatt_notify(NULL, BAT_NOTIFY_ATTR, &bat_pkt, BAT_PKT_SIZE);
  } else if (c == 'R') {
    pending_ble_cmd = 20;
  } else if (c == 'G') {
    pending_ble_cmd = 21;
  } else if (c == 'B') {
    pending_ble_cmd = 22;
  } else if (c == 'Y') {
    pending_ble_cmd = 23;
  } else if (c == 'P') {
    pending_ble_cmd = 24;
  } else if (c == 'C') {
    pending_ble_cmd = 25;
  } else if (c == 'W') {
    pending_ble_cmd = 26;
  } else if (c == 'O') {
    pending_ble_cmd = 27;
  }
}
k_sleep(K_MSEC(10));
}
}

/* Manual thread definition for deferred start — started AFTER USB enable */
static K_THREAD_STACK_DEFINE(meow_serial_stack, 2048);
static struct k_thread meow_serial_thread_data;

void meow_ctrl_start_serial_thread(void) {
  k_thread_create(&meow_serial_thread_data, meow_serial_stack,
                  K_THREAD_STACK_SIZEOF(meow_serial_stack),
                  (k_thread_entry_t)serial_thread, NULL, NULL, NULL, 7, 0,
                  K_NO_WAIT);
  k_thread_name_set(&meow_serial_thread_data, "meow_serial");
}

/* ================================================================== */
/*  Init                                                               */
/* ================================================================== */
int init_meow_ctrl_service(void) {
  /* Initialize packet headers */
  imu_pkt.header[0] = 0xAA;
  imu_pkt.header[1] = 0x55;
  bat_pkt.header[0] = 0xBB;
  bat_pkt.header[1] = 0x66;

  LOG_INF("Meow Control Service initialized (System Managed)");
  return 0;
}
