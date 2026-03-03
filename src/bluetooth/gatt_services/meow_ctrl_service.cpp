/*
 * Meow Sense Tag – BLE Control Service + USB CDC Serial Console
 *
 * A self-contained module that lets a phone app (via BLE) OR a serial
 * terminal (via USB CDC ACM virtual COM port) control IMU data streaming
 * and query battery status via System Modules (SensorManager, PowerManager).
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

/* 引用系統模組 */
#include "../../Battery/PowerManager.h"
#include "../../ParseInfo/SensorScheme.h"
#include "../../SensorManager/IMU.h"
#include "../../SensorManager/SensorManager.h"
#include "openearable_common.h"

LOG_MODULE_REGISTER(meow_ctrl, CONFIG_MAIN_LOG_LEVEL);

/* ================================================================== */
/* UART 序列輸出輔助函數                                             */
/* ================================================================== */
static const struct device *uart_dev;

static void uart_puts(const char *s) {
  if (!uart_dev)
    return;
  while (*s) {
    uart_poll_out(uart_dev, *s++);
  }
}

static void uart_printf(const char *fmt, ...) {
  char buf[256];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  uart_puts(buf);
}

static void meow_i2c_scan(void) {
  uart_printf("\r\n============================================\r\n");
  uart_printf("  Meow Sense - System Check\r\n");
  uart_printf("============================================\r\n\r\n");
}

/* ================================================================== */
/* BLE UUID 定義與封包格式                                           */
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

/* 定義 IMU 單筆樣本結構 (相容 Arduino Xinyi_IMU 協定) */
struct __attribute__((packed)) IMUSample {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
};

/* 定義 IMU 批次封包結構 (包含標頭、時間戳與 10 筆樣本) */
#define IMU_BATCH_SIZE 10
struct __attribute__((packed)) FullPacket {
  uint8_t header[2];
  uint32_t timestamp;
  struct IMUSample samples[IMU_BATCH_SIZE];
};
#define IMU_PKT_SIZE sizeof(struct FullPacket)

/* 定義電池狀態封包結構 */
struct __attribute__((packed)) BatteryPacket {
  uint8_t header[2];
  float voltage;
  uint8_t isCharging;
  uint8_t batteryLevel;
};
#define BAT_PKT_SIZE sizeof(struct BatteryPacket)

/* 靜態變數儲存目前封包狀態 */
static struct FullPacket imu_pkt;
static struct BatteryPacket bat_pkt;
static volatile int imu_sample_idx;

/* 用於降低 UART 印出頻率的計數器 (避免印出太快卡死系統) */
#define SERIAL_IMU_PRINT_DIVIDER 4
static int serial_batch_counter;

/* BLE 指令暫存旗標 (將指令交由 serial_thread 處理，避免 BLE 回呼阻塞) */
static volatile int pending_ble_cmd = 0;

/* 資料轉換比例常數 */
#define ACCEL_SCALE_FACTOR 1670.1f // 16G 範圍下的轉換因子
#define GYRO_SCALE_FACTOR 131.2f   // 250dps 範圍下的轉換因子

static ssize_t write_cmd(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                         const void *buf, uint16_t len, uint16_t offset,
                         uint8_t flags);
static ssize_t read_bat(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                        void *buf, uint16_t len, uint16_t offset);
static void imu_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value);
static void bat_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value);

/* 定義 GATT 服務與特徵值 */
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

/* 取得通知特徵值的指標 */
#define IMU_NOTIFY_ATTR (&meow_ctrl_svc.attrs[4])
#define BAT_NOTIFY_ATTR (&meow_ctrl_svc.attrs[7])

static void imu_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value) {
  LOG_INF("IMU CCC – 0x%04x", value);
}
static void bat_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value) {
  LOG_INF("BAT CCC – 0x%04x", value);
}

/* ================================================================== */
/* 電池狀態讀取輔助函數                                              */
/* ================================================================== */
static void fill_bat_pkt(void) {
  float voltage_v = fuel_gauge.voltage();
  float soc = fuel_gauge.state_of_charge();
  bool charging = battery_controller.power_connected();

  bat_pkt.header[0] = 0xBB;
  bat_pkt.header[1] = 0x66;
  bat_pkt.voltage = voltage_v;
  bat_pkt.isCharging = charging ? 1 : 0;
  bat_pkt.batteryLevel = (uint8_t)soc;
}

static ssize_t read_bat(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                        void *buf, uint16_t len, uint16_t offset) {
  fill_bat_pkt();
  return bt_gatt_attr_read(conn, attr, buf, len, offset, &bat_pkt,
                           BAT_PKT_SIZE);
}

/* ================================================================== */
/* IMU 串流啟動 / 停止 (單純控制資料流，不碰硬體 PMU)                 */
/* ================================================================== */
static int imu_start(void) {
  uart_printf("[IMU] Instructing SensorManager to start streaming...\r\n");

  /* 設定 SensorManager 開始串流。硬體喚醒已在 IMU::init 完成 */
  struct sensor_config config;
  config.sensorId = ID_IMU;
  config.sampleRateIndex = 2;             // 設定 ODR 為 100Hz
  config.storageOptions = DATA_STREAMING; // 啟用 BLE 串流旗標
  config_sensor(&config);

  imu_sample_idx = 0; // 重置批次索引

  // 發送一個成功的電池封包格式通知 App：IMU 已啟動
  bat_pkt.header[0] = 'I';
  bat_pkt.header[1] = 0x15; // 模擬成功的 PMU (00010101)
  bat_pkt.voltage = 0;      // 無錯誤
  bat_pkt.isCharging = 0;
  bat_pkt.batteryLevel = 1; // 1 = 成功
  bt_gatt_notify(NULL, BAT_NOTIFY_ATTR, &bat_pkt, BAT_PKT_SIZE);

  return 0;
}

static void imu_stop(void) {
  uart_printf("[IMU] Stopping via SensorManager...\r\n");
  struct sensor_config config;
  config.sensorId = ID_IMU;
  config.sampleRateIndex = 0;
  config.storageOptions = 0; // 關閉所有資料處理
  config_sensor(&config);
}

/* ================================================================== */
/* Zbus 事件回呼處理 (接收來自 SensorManager 的感測器數據)            */
/* ================================================================== */
static void meow_ctrl_zbus_handler(const struct zbus_channel *chan) {
  const struct sensor_msg *msg =
      (const struct sensor_msg *)zbus_chan_const_msg(chan);
  if (!msg)
    return;

  if (msg->data.id == ID_IMU) {
    float ax_f, ay_f, az_f, gx_f, gy_f, gz_f;

    // 從 Zbus 訊息中提取浮點數資料
    memcpy(&ax_f, &msg->data.data[0], sizeof(float));
    memcpy(&ay_f, &msg->data.data[4], sizeof(float));
    memcpy(&az_f, &msg->data.data[8], sizeof(float));
    memcpy(&gx_f, &msg->data.data[12], sizeof(float));
    memcpy(&gy_f, &msg->data.data[16], sizeof(float));
    memcpy(&gz_f, &msg->data.data[20], sizeof(float));

    // 將浮點數轉換為帶有比例因子的 16-bit 整數
    int16_t ax = (int16_t)(ax_f * ACCEL_SCALE_FACTOR);
    int16_t ay = (int16_t)(ay_f * ACCEL_SCALE_FACTOR);
    int16_t az = (int16_t)(az_f * ACCEL_SCALE_FACTOR);
    int16_t gx = (int16_t)(gx_f * GYRO_SCALE_FACTOR);
    int16_t gy = (int16_t)(gy_f * GYRO_SCALE_FACTOR);
    int16_t gz = (int16_t)(gz_f * GYRO_SCALE_FACTOR);

    // 將轉換後的資料存入批次封包中
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

    // 若批次滿了 (收集滿 10 筆)，則發送 BLE 通知
    if (imu_sample_idx >= IMU_BATCH_SIZE) {
      imu_pkt.header[0] = 0xAA;
      imu_pkt.header[1] = 0x55;
      imu_pkt.timestamp = (uint32_t)msg->data.time;

      bt_gatt_notify(NULL, IMU_NOTIFY_ATTR, &imu_pkt, IMU_PKT_SIZE);

      // 每隔幾次批次印出一次除錯資訊到序列埠，避免洗頻
      serial_batch_counter++;
      if (serial_batch_counter >= SERIAL_IMU_PRINT_DIVIDER) {
        serial_batch_counter = 0;
        uart_printf("AX:%5d AY:%5d AZ:%5d GX:%5d GY:%5d GZ:%5d\r\n", ax, ay, az,
                    gx, gy, gz);
      }
      imu_sample_idx = 0; // 重置索引
    }
  }
}

/* 註冊 Zbus 監聽器 */
ZBUS_LISTENER_DEFINE(meow_ctrl_lis, meow_ctrl_zbus_handler);
ZBUS_CHAN_ADD_OBS(sensor_chan, meow_ctrl_lis, 2);

/* ================================================================== */
/* BLE 寫入指令回呼處理                                              */
/* ================================================================== */
static ssize_t write_cmd(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                         const void *buf, uint16_t len, uint16_t offset,
                         uint8_t flags) {
  const char *data = (const char *)buf;
  if (len == 0)
    return len;

  char cmd = data[0];
  LOG_INF("Received command: %c", cmd);

  // 解析指令並設定旗標，實際執行會交由 serial_thread 處理
  switch (cmd) {
  case 's':
    pending_ble_cmd = 1;
    break; // 啟動 IMU
  case 'p':
    pending_ble_cmd = 2;
    break; // 停止 IMU
  case 'b':
    pending_ble_cmd = 3;
    break; // 查詢電池
  case 'R':
    pending_ble_cmd = 20;
    break; // LED: 紅
  case 'G':
    pending_ble_cmd = 21;
    break; // LED: 綠
  case 'U':
    pending_ble_cmd = 22;
    break; // LED: 藍
  case 'Y':
    pending_ble_cmd = 23;
    break; // LED: 黃
  case 'P':
    pending_ble_cmd = 24;
    break; // LED: 紫
  case 'C':
    pending_ble_cmd = 25;
    break; // LED: 青
  case 'W':
    pending_ble_cmd = 26;
    break; // LED: 白
  case 'O':
    pending_ble_cmd = 27;
    break; // LED: 關閉
  default:
    break;
  }
  return len;
}

/* ================================================================== */
/* 主處理執行緒 (處理序列埠輸入、BLE 指令旗標與定時任務)              */
/* ================================================================== */
static void serial_thread(void) {
  uart_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
  if (!device_is_ready(uart_dev))
    return;

  // 等待終端機連線 (DTR 訊號)
  uint32_t dtr = 0;
  for (int i = 0; i < 30; i++) {
    uart_line_ctrl_get(uart_dev, UART_LINE_CTRL_DTR, &dtr);
    if (dtr)
      break;
    k_msleep(100);
  }

  meow_i2c_scan();
  int64_t last_bat_check_time = k_uptime_get();

  while (1) {
    // 1. 處理待辦的 BLE 指令
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

      // 處理 LED 控制邏輯 (透過 I2C1 寫入暫存器)
      const struct device *i2c1_dev = DEVICE_DT_GET(DT_NODELABEL(i2c1));
      if (device_is_ready(i2c1_dev)) {
        uint8_t r = 0, g = 0, b = 0, bright = 0x55;
        switch (color_cmd) {
        case 20:
          r = bright;
          break;
        case 21:
          g = bright;
          break;
        case 22:
          b = bright;
          break;
        case 23:
          r = g = bright;
          break;
        case 24:
          r = b = bright;
          break;
        case 25:
          g = b = bright;
          break;
        case 26:
          r = g = b = bright;
          break;
        case 27:
          break;
        }
        if (color_cmd == 27) {
          i2c_reg_write_byte(i2c1_dev, 0x30, 0x00, 0x00); // 關閉 LED
        } else {
          i2c_reg_write_byte(i2c1_dev, 0x30, 0x00, 0x08); // 喚醒 LED 控制晶片
          i2c_reg_write_byte(i2c1_dev, 0x30, 0x06, r);
          i2c_reg_write_byte(i2c1_dev, 0x30, 0x07, g);
          i2c_reg_write_byte(i2c1_dev, 0x30, 0x08, b);
          i2c_reg_write_byte(i2c1_dev, 0x30, 0x04, 0x15);
        }
      }
    }

    // 2. 定期檢查電池狀態 (每 5 秒)，低電量主動推播
    int64_t now = k_uptime_get();
    if (now - last_bat_check_time >= 5000) {
      last_bat_check_time = now;
      if (fuel_gauge.state_of_charge() < 30.0f) {
        fill_bat_pkt();
        bt_gatt_notify(NULL, BAT_NOTIFY_ATTR, &bat_pkt, BAT_PKT_SIZE);
      }
    }

    // 3. 處理來自 USB 序列埠的輸入指令
    unsigned char c;
    if (uart_poll_in(uart_dev, &c) == 0) {
      if (c == 's')
        imu_start();
      else if (c == 'p')
        imu_stop();
      else if (c == 'b') {
        fill_bat_pkt();
        bt_gatt_notify(NULL, BAT_NOTIFY_ATTR, &bat_pkt, BAT_PKT_SIZE);
      }
    }

    // 短暫休眠讓出 CPU 資源給其他執行緒
    k_sleep(K_MSEC(10));
  }
}

/* 宣告並啟動主執行緒 */
static K_THREAD_STACK_DEFINE(meow_serial_stack, 2048);
static struct k_thread meow_serial_thread_data;

void meow_ctrl_start_serial_thread(void) {
  k_thread_create(&meow_serial_thread_data, meow_serial_stack,
                  K_THREAD_STACK_SIZEOF(meow_serial_stack),
                  (k_thread_entry_t)serial_thread, NULL, NULL, NULL, 7, 0,
                  K_NO_WAIT);
  k_thread_name_set(&meow_serial_thread_data, "meow_serial");
}

/* 模組初始化函數 */
int init_meow_ctrl_service(void) {
  // 設定封包標頭，相容於既有協定
  imu_pkt.header[0] = 0xAA;
  imu_pkt.header[1] = 0x55;
  bat_pkt.header[0] = 0xBB;
  bat_pkt.header[1] = 0x66;
  LOG_INF("Meow Control Service initialized (Sync Config Mode)");
  return 0;
}