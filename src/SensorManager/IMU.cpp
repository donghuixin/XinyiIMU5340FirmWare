#include "IMU.h"
#include "SensorManager.h"

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/zbus/zbus.h>

LOG_MODULE_DECLARE(BMI160_DFR);

static struct sensor_msg msg_imu;

/* Meow Sense Tag: BMI160 預設連接在 I2C2 上 */
DFRobot_BMI160 IMU::imu(&I2C2);
IMU IMU::sensor;

/* 外部引用 SensorManager 的專屬工作佇列，避免佔用系統佇列卡死藍牙 */
extern struct k_work_q sensor_work_q;

/* 檔案層級的靜態變數 */
static struct k_work_delayable async_wake_work;
static int wake_retry_count = 0;
static int stored_sample_rate_idx = 0;
static bool is_waking_up = false;

const SampleRateSetting<6> IMU::sample_rates = {
    {BMX160_GYRO_ODR_25HZ, BMX160_GYRO_ODR_50HZ, BMX160_GYRO_ODR_100HZ,
     BMX160_GYRO_ODR_200HZ, BMX160_GYRO_ODR_400HZ, BMX160_GYRO_ODR_800HZ},
    {25, 50, 100, 200, 400, 800},
    {25.0, 50.0, 100.0, 200.0, 400.0, 800.0}};

void IMU::update_sensor(struct k_work *work) {
  int ret;
  sBmx160SensorData_t magno_data;
  sBmx160SensorData_t gyro_data;
  sBmx160SensorData_t accel_data;

  imu.getAllData(&magno_data, &gyro_data, &accel_data);

  static int print_divider = 0;
  if (++print_divider >= 100) {
    print_divider = 0;
    LOG_INF("IMU 1Hz Heartbeat: Accel X: %f, Y: %f, Z: %f",
            (double)accel_data.x, (double)accel_data.y, (double)accel_data.z);
  }

  size_t size = 3 * sizeof(float);
  msg_imu.sd = sensor._sd_logging;
  msg_imu.stream = sensor._ble_stream;
  msg_imu.data.id = ID_IMU;
  msg_imu.data.size = 3 * size;
  msg_imu.data.time = micros();

  memcpy(msg_imu.data.data, &accel_data, size);
  memcpy(msg_imu.data.data + size, &gyro_data, size);
  memcpy(msg_imu.data.data + 2 * size, &magno_data, size);

  ret = k_msgq_put(sensor_queue, &msg_imu, K_NO_WAIT);
  if (ret) {
    LOG_WRN("sensor msg queue full");
  }
}

void IMU::sensor_timer_handler(struct k_timer *dummy) {
  k_work_submit_to_queue(&sensor_work_q, &sensor.sensor_work);
};

void IMU::imu_async_rescue_handler(struct k_work *work) {
  const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c2));
  if (!device_is_ready(i2c_dev)) {
    LOG_ERR("I2C2 not ready! Cannot rescue IMU.");
    is_waking_up = false;
    return;
  }

  wake_retry_count++;
  LOG_INF("--- [Async] IMU Wake & Rescue Attempt %d/3 ---", wake_retry_count);

  uint8_t pmu = 0, err = 0;
  i2c_reg_read_byte(i2c_dev, 0x68, 0x03, &pmu);
  i2c_reg_read_byte(i2c_dev, 0x68, 0x02, &err);

  LOG_INF("[Async] Pre-check -> PMU: 0x%02X, ERR: 0x%02X", pmu, err);

  bool acc_normal = ((pmu >> 4) & 0x03) == 0x01;
  bool gyr_normal = ((pmu >> 2) & 0x03) == 0x01;

  if (err != 0 || !acc_normal || !gyr_normal) {
    LOG_WRN("[Async] State invalid/locked. Forcing Soft Reset (0xB6) to clear "
            "state...");
    i2c_reg_write_byte(i2c_dev, 0x68, 0x7E, 0xB6);
    k_msleep(50); // increased from 20ms to 50ms to ensure full reset

    uint8_t dummy;
    i2c_reg_read_byte(i2c_dev, 0x68, 0x7F,
                      &dummy); // Dummy read to stabilize I2C
    k_msleep(2);

    i2c_reg_read_byte(i2c_dev, 0x68, 0x02, &err); // Clear any reset errors
  }

  // 執行標準喚醒時序
  i2c_reg_write_byte(i2c_dev, 0x68, 0x7E, 0x11); // Accel Normal
  k_msleep(55); // increased Accel startup delay (max 5ms + margin)

  i2c_reg_write_byte(i2c_dev, 0x68, 0x7E, 0x15); // Gyro Normal
  k_msleep(150); // increased Gyro startup delay (max 80ms + margin)

  // 再次讀取確認狀態
  i2c_reg_read_byte(i2c_dev, 0x68, 0x03, &pmu);
  i2c_reg_read_byte(i2c_dev, 0x68, 0x02, &err);
  acc_normal = ((pmu >> 4) & 0x03) == 0x01;
  gyr_normal = ((pmu >> 2) & 0x03) == 0x01;

  if (acc_normal && gyr_normal && err == 0) {
    LOG_INF("[Async] Rescue Success! PMU verified (0x%02X).", pmu);

    // 這裡才呼叫 begin 來同步庫的內部狀態 (因為確認晶片活著，就不會卡死)
    IMU::imu.begin();
    IMU::imu.setAccelRange(eAccelRange_16G);

    IMU::sensor._active = true;
    is_waking_up = false;

    LOG_INF("[Async] Resuming IMU data stream...");
    IMU::sensor.start(stored_sample_rate_idx);
    return;
  } else {
    LOG_ERR("[Async] Rescue Failed! PMU: 0x%02X, ERR: 0x%02X", pmu, err);
  }

  // [關鍵修改] 改將重試任務提交給專屬的 sensor_work_q，絕對不干擾藍牙
  if (wake_retry_count < 3) {
    LOG_WRN("[Async] Yielding thread. Will retry in 3 seconds...");
    int ret = k_work_reschedule_for_queue(&sensor_work_q, &async_wake_work,
                                          K_SECONDS(3));
    if (ret < 0) {
      LOG_ERR("[Async] Failed to reschedule work item (err: %d)!", ret);
      is_waking_up = false;
    }
  } else {
    LOG_ERR("[Async] FATAL: IMU hardware dead after 3 attempts.");
    is_waking_up = false; // 解鎖，讓您可以再次按 's' 觸發新的一輪重試
    IMU::sensor._active = false;
  }
}

bool IMU::init(struct k_msgq *queue) {
  LOG_INF(">>> IMU::init() CALLED (Setting up queues & handlers) <<<");

  sensor_queue = queue;

  static bool delayable_init = false;
  if (!delayable_init) {
    k_work_init_delayable(&async_wake_work, imu_async_rescue_handler);
    delayable_init = true;
  }

  k_work_init(&sensor.sensor_work, update_sensor);
  k_timer_init(&sensor.sensor_timer, sensor_timer_handler, NULL);

  _active = false;
  return true;
}

void IMU::start(int sample_rate_idx) {
  stored_sample_rate_idx = sample_rate_idx;

  if (!_active) {
    if (!is_waking_up) {
      LOG_ERR("IMU INITIALIZATION FAILED / NOT ACTIVE! Initiating Async Rescue "
              "& Wakeup "
              "sequence...");
      is_waking_up = true;
      wake_retry_count = 0;
      // [關鍵修改] 改提交給專屬工作佇列
      k_work_reschedule_for_queue(&sensor_work_q, &async_wake_work, K_NO_WAIT);
    } else {
      LOG_ERR("IMU STUCK ERROR: Currently running async recovery. Command "
              "ignored.");
    }
    return;
  }

  k_timeout_t t = K_USEC(1e6 / sample_rates.true_sample_rates[sample_rate_idx]);

  imu.setAccelODR(sample_rates.reg_vals[sample_rate_idx]);
  imu.setGyroODR(sample_rates.reg_vals[sample_rate_idx]);
  imu.setMagnODR(sample_rates.reg_vals[sample_rate_idx]);

  _running = true;
  k_timer_start(&sensor.sensor_timer, K_NO_WAIT, t);
  LOG_INF("IMU Timer started successfully at index %d", sample_rate_idx);
}

void IMU::stop() {
  if (!_active)
    return;
  _running = false;
  k_timer_stop(&sensor.sensor_timer);
  LOG_INF("IMU Timer stopped.");
}