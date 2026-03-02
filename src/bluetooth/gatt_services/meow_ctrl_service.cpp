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

extern "C" int bt_mgmt_adv_start(uint8_t flags, const struct bt_data *ad,
                                 size_t ad_len, const struct bt_data *sd,
                                 size_t sd_len, bool enable_unrecog);

static int imu_start(void);
static void imu_stop(void);

LOG_MODULE_REGISTER(meow_ctrl, CONFIG_MAIN_LOG_LEVEL);

/* ================================================================== */
/* 1. 系统状态机定义                                                  */
/* ================================================================== */
typedef enum {
  SYS_STATE_PAIRING,   // 蓝灯闪烁 (1秒周期)
  SYS_STATE_CONNECTED, // 蓝牙已连接 (灭灯)
  SYS_STATE_CHARGING,  // 充电中 (红灯呼吸，2秒周期)
  SYS_STATE_FULL,      // 充满电 (绿灯常亮)
  SYS_STATE_SLEEP      // 深度休眠 (灭灯，等待敲击唤醒)
} meow_sys_state_t;

volatile meow_sys_state_t current_state = SYS_STATE_PAIRING;

/* ================================================================== */
/* 2. KTD2026 状态机 LED 驱动                                         */
/* ================================================================== */
void update_system_led() {
  const struct device *i2c1_dev = DEVICE_DT_GET(DT_NODELABEL(i2c1));
  if (!device_is_ready(i2c1_dev))
    return;

  // 先全面复位 KTD2026
  i2c_reg_write_byte(i2c1_dev, 0x30, 0x00, 0x00); // Reset/Sleep
  i2c_reg_write_byte(i2c1_dev, 0x30, 0x04, 0x00); // 关红
  i2c_reg_write_byte(i2c1_dev, 0x30, 0x05, 0x00); // 关绿
  i2c_reg_write_byte(i2c1_dev, 0x30, 0x06, 0x00); // 关蓝

  switch (current_state) {
  case SYS_STATE_CHARGING:
    // 红灯呼吸 (2秒周期) - 利用 KTD2026 内部 Timer/Fade
    i2c_reg_write_byte(i2c1_dev, 0x30, 0x01, 0x66); // 设置 Timer Flash Period
    i2c_reg_write_byte(i2c1_dev, 0x30, 0x06,
                       0x55); // CH1 红灯基础亮度 (Reg 0x06)
    i2c_reg_write_byte(i2c1_dev, 0x30, 0x00, 0x0A); // 模式：开启 PWM/Timer
    break;

  case SYS_STATE_FULL:
    // 绿灯常亮
    i2c_reg_write_byte(i2c1_dev, 0x30, 0x07, 0x55); // CH2 绿灯亮度 (Reg 0x07)
    i2c_reg_write_byte(i2c1_dev, 0x30, 0x00, 0x08); // 模式：Normal
    break;

  case SYS_STATE_PAIRING:
    // 蓝灯闪烁 (1秒周期)
    i2c_reg_write_byte(i2c1_dev, 0x30, 0x01, 0x31); // 1Hz 闪烁速率
    i2c_reg_write_byte(i2c1_dev, 0x30, 0x08, 0x55); // CH3 蓝灯亮度 (Reg 0x08)
    i2c_reg_write_byte(i2c1_dev, 0x30, 0x00, 0x0A); // 模式：开启 PWM/Timer
    break;

  case SYS_STATE_CONNECTED:
  case SYS_STATE_SLEEP:
    // 灭灯 (保持 0x00 即可)
    i2c_reg_write_byte(i2c1_dev, 0x30, 0x00, 0x00);
    break;
  }
}

/* ================================================================== */
/* 3. 30秒无连接休眠倒计时                                            */
/* ================================================================== */
static struct k_work_delayable auto_sleep_work;

static void auto_sleep_handler(struct k_work *work) {
  if (current_state == SYS_STATE_PAIRING) {
    printk("[SYS] 30s timeout. Entering Micro-Sleep...\n");
    current_state = SYS_STATE_SLEEP;
    update_system_led();

    // 停止蓝牙广播以省电
    bt_le_adv_stop();

    // 停止 IMU 高频采样，进入低功耗监听模式
    imu_stop();

    // 此处通过 I2C 向 0x68 发指令，配置 BMI160 开启单次/双次敲击中断
    const struct device *i2c2_dev = DEVICE_DT_GET(DT_NODELABEL(i2c2));
    if (device_is_ready(i2c2_dev)) {
      i2c_reg_write_byte(i2c2_dev, 0x68, 0x40,
                         0x30); // INT_EN_0: Enable single and double tap
      i2c_reg_write_byte(i2c2_dev, 0x68, 0x42,
                         0x0A); // INT_OUT_CTRL: INT1 enable, open drain
      i2c_reg_write_byte(i2c2_dev, 0x68, 0x2B,
                         0x01); // TAP_PARAM: standard tap logic
    }
  }
}

// 供外部调用的刷新休眠定时器函数
void reset_sleep_timer() {
  if (current_state != SYS_STATE_CHARGING && current_state != SYS_STATE_FULL) {
    printk("[SYS] Sleep timer reset to 120s.\n");
    k_work_reschedule(&auto_sleep_work, K_SECONDS(120));
  } else {
    printk("[SYS] Charging/Full: auto-sleep disabled.\n");
    k_work_cancel_delayable(&auto_sleep_work); // 充电时不休眠
  }
}

/* ================================================================== */
/* 4. 微睡眠轮询线程 (解决没有硬件 INT 中断线的问题)                  */
/* ================================================================== */
static void micro_sleep_polling_thread(void) {
  const struct device *i2c2_dev = DEVICE_DT_GET(DT_NODELABEL(i2c2));

  int tap_count = 0;
  int64_t last_tap_time = 0;

  while (1) {
    if (current_state == SYS_STATE_SLEEP) {
      uint8_t int_status = 0;
      // 读取 BMI160 的 INT_STATUS_0 寄存器 (0x1C)
      if (i2c_reg_read_byte(i2c2_dev, 0x68, 0x1C, &int_status) == 0) {
        if (int_status & 0x30) { // 检测到单次或双次敲击 (Bit 5 or Bit 4)
          int64_t now = k_uptime_get();

          // 如果距离上次敲击超过 1.5 秒，重新计数
          if (now - last_tap_time > 1500) {
            tap_count = 0;
          }

          tap_count++;
          last_tap_time = now;
          printk("[SYS] Tap detected! Count: %d\n", tap_count);

          if (tap_count >= 3) {
            printk("[SYS] Triple tap confirmed! Waking up...\n");
            tap_count = 0;
            current_state = SYS_STATE_PAIRING;
            update_system_led();

            // 重新开启蓝牙广播
            bt_mgmt_adv_start(0, NULL, 0, NULL, 0, true);

            // 重新开启 30 秒倒计时
            reset_sleep_timer();
          }
        }
      }
      // 极限省电：休眠 100ms 再查一次 (1秒查10次，既省电又不会漏掉敲击)
      k_msleep(100);
    } else {
      // 如果不在休眠状态，这个线程彻底挂起休息 1 秒，完全不占 CPU
      k_msleep(1000);
    }
  }
}

// 注册极低优先级的后台轮询线程
K_THREAD_DEFINE(polling_id, 1024, micro_sleep_polling_thread, NULL, NULL, NULL,
                8, 0, 0);

/* ================================================================== */
/* 5. 蓝牙连接回调挂钩                                                */
/* ================================================================== */
static void meow_connected(struct bt_conn *conn, uint8_t err) {
  if (err)
    return;
  printk("[BLE] Connected!\n");
  k_work_cancel_delayable(&auto_sleep_work); // 连上后取消休眠倒计时
  current_state = SYS_STATE_CONNECTED;
  update_system_led();
}

static void meow_disconnected(struct bt_conn *conn, uint8_t reason) {
  printk("[BLE] Disconnected!\n");
  current_state = SYS_STATE_PAIRING;
  update_system_led();
  reset_sleep_timer(); // 断开后重新开始 30 秒倒计时
}

BT_CONN_CB_DEFINE(meow_conn_callbacks) = {
    .connected = meow_connected,
    .disconnected = meow_disconnected,
};

/* ================================================================== */
/* 6. 初始化状态机                                                    */
/* ================================================================== */
void init_meow_state_machine() {
  printk("\n[SYS] init_meow_state_machine() called! Starting sleep timer...\n");
  k_work_init_delayable(&auto_sleep_work, auto_sleep_handler);

  // 初始化时默认为配对模式，开启蓝灯闪烁和 30 秒倒计时
  current_state = SYS_STATE_PAIRING;
  update_system_led();
  reset_sleep_timer();
}

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
    printk("[BLE] Cmd R -> Red\n");
    break; // 红色 (Red)
  case 'G':
    pending_ble_cmd = 21;
    printk("[BLE] Cmd G -> Green\n");
    break; // 绿色 (Green)
  case 'B':
    pending_ble_cmd = 22;
    printk("[BLE] Cmd B -> Blue\n");
    break; // 蓝色 (Blue)
  case 'Y':
    pending_ble_cmd = 23;
    printk("[BLE] Cmd Y -> Yellow\n");
    break; // 黄色 (Yellow = R+G)
  case 'P':
    pending_ble_cmd = 24;
    printk("[BLE] Cmd P -> Purple\n");
    break; // 紫色 (Purple = R+B)
  case 'C':
    pending_ble_cmd = 25;
    printk("[BLE] Cmd C -> Cyan\n");
    break; // 青色 (Cyan = G+B)
  case 'W':
    pending_ble_cmd = 26;
    printk("[BLE] Cmd W -> White\n");
    break; // 白色 (White = R+G+B)
  case 'O':
    pending_ble_cmd = 27;
    printk("[BLE] Cmd O -> OFF\n");
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
    /* ========================================================== */
    /* ==== 状态机：充电检测及断开恢复机制 ==== */
    /* ========================================================== */
    bool is_charging = battery_controller.power_connected();
    float soc = fuel_gauge.state_of_charge();

    if (is_charging) {
      if (soc >= 99.0f && current_state != SYS_STATE_FULL) {
        current_state = SYS_STATE_FULL;
        update_system_led();
      } else if (soc < 99.0f && current_state != SYS_STATE_CHARGING) {
        current_state = SYS_STATE_CHARGING;
        update_system_led();
      }
    } else if (current_state == SYS_STATE_CHARGING ||
               current_state == SYS_STATE_FULL) {
      // 拔掉充电线，恢复配对模式
      current_state = SYS_STATE_PAIRING;
      update_system_led();
      reset_sleep_timer();
    }
    /* ========================================================== */

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

      printk("\n--- [I2C DEBUG] 准备操作 KTD2026 (0x30) ---\n");

      // 1. 获取设备句柄
      const struct device *i2c1_dev = DEVICE_DT_GET(DT_NODELABEL(i2c1));
      if (!device_is_ready(i2c1_dev)) {
        printk("❌ [ERROR] i2c1_dev 未就绪！\n");
        continue;
      }

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

      int err = 0;
      if (color_cmd == 27) {
        err = i2c_reg_write_byte(i2c1_dev, 0x30, 0x00, 0x00);
      } else {
        // 连续写入，并捕获第一次写入的错误码
        err = i2c_reg_write_byte(i2c1_dev, 0x30, 0x00, 0x08);
        i2c_reg_write_byte(i2c1_dev, 0x30, 0x04, 0x15);
        i2c_reg_write_byte(i2c1_dev, 0x30, 0x06, r);
        i2c_reg_write_byte(i2c1_dev, 0x30, 0x07, g);
        i2c_reg_write_byte(i2c1_dev, 0x30, 0x08, b);
      }

      if (err == 0) {
        printk("✅ [SUCCESS] I2C 写入成功！LED "
               "应该亮了。如果没亮，请检查硬件是否接在 i2c2 上。\n");
      } else {
        printk("❌ [FAILED] I2C 写入失败！错误码: %d\n", err);
        printk("👉 [诊断] 如果错误码是 -5 (EIO)，说明 KTD2026 "
               "没电，或总线上根本没这个芯片！\n");
      }
      printk("----------------------------------------\n\n");
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
          uart_printf(
              "[WARNING] Battery low (%.1f%%), auto-notified via BLE!\r\n",
              (double)current_soc);
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

  init_meow_state_machine();

  LOG_INF("Meow Control Service initialized (System Managed)");
  return 0;
}
