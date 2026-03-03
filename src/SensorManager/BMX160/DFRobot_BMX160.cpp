/*!
 * @file DFRobot_BMI160.cpp
 * @brief define DFRobot_BMI160 class infrastructure, the implementation of
 * basic methods
 * @copyright	Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT License (MIT)
 * @author [luoyufeng] (yufeng.luo@dfrobot.com)
 * @maintainer [Fary](feng.yang@dfrobot.com)
 * @version  V1.0
 * @date  2021-10-20
 * @url https://github.com/DFRobot/DFRobot_BMI160
 */
#include "DFRobot_BMX160.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(BMI160_DFR, CONFIG_MAIN_LOG_LEVEL);

#define delay(ms) k_msleep(ms)
#define malloc(a) k_malloc(a)

DFRobot_BMI160::DFRobot_BMI160(TWIM *i2c) : _i2c(i2c) {
  Obmx160 = (sBmx160Dev_t *)malloc(sizeof(sBmx160Dev_t));
  Oaccel = (sBmx160SensorData_t *)malloc(sizeof(sBmx160SensorData_t));
  Ogyro = (sBmx160SensorData_t *)malloc(sizeof(sBmx160SensorData_t));
  Omagn = (sBmx160SensorData_t *)malloc(sizeof(sBmx160SensorData_t));
}

const uint8_t int_mask_lookup_table[13] = {BMX160_INT1_SLOPE_MASK,
                                           BMX160_INT1_SLOPE_MASK,
                                           BMX160_INT2_LOW_STEP_DETECT_MASK,
                                           BMX160_INT1_DOUBLE_TAP_MASK,
                                           BMX160_INT1_SINGLE_TAP_MASK,
                                           BMX160_INT1_ORIENT_MASK,
                                           BMX160_INT1_FLAT_MASK,
                                           BMX160_INT1_HIGH_G_MASK,
                                           BMX160_INT1_LOW_G_MASK,
                                           BMX160_INT1_NO_MOTION_MASK,
                                           BMX160_INT2_DATA_READY_MASK,
                                           BMX160_INT2_FIFO_FULL_MASK,
                                           BMX160_INT2_FIFO_WM_MASK};

bool DFRobot_BMI160::begin() {
  _i2c->begin();

  if (scan() == true) {
    // 1. 執行軟重置 (Soft Reset)
    if (!softReset()) {
      LOG_ERR("Soft reset command failed.");
      return false;
    }

    // [關鍵] 軟重置後強制等待，確保晶片內部邏輯重啟完畢 (手冊要求至少 15ms)
    delay(20);

    // [關鍵] 進行一次 Dummy Read 來喚醒並穩定 I2C 介面
    uint8_t dummy;
    readReg(0x7F, &dummy, 1);
    delay(2);

    uint8_t err_reg = 0;
    // 清除任何剛開機或重置殘留的錯誤標誌
    readReg(BMX160_ERROR_REG_ADDR, &err_reg, 1);

    // 2. 喚醒加速度計至 Normal 模式
    writeBmxReg(BMX160_COMMAND_REG_ADDR, 0x11);
    delay(55); // 加速度計啟動時間最大 5ms，給予 55ms 非常安全

    // 再次檢查是否有錯誤
    readReg(BMX160_ERROR_REG_ADDR, &err_reg, 1);
    if (err_reg) {
      LOG_DBG("ERR_REG after Accel init: 0x%02X", err_reg);
    }

    // 3. 喚醒陀螺儀至 Normal 模式
    writeBmxReg(BMX160_COMMAND_REG_ADDR, 0x15);
    delay(150); // 陀螺儀啟動時間最大 80ms，等待 150ms 非常安全

    // 4. 驗證 PMU 狀態 (檢查是否真正進入 Normal 模式)
    uint8_t pmu_status = 0;

    // 安全的重試迴圈
    for (int i = 0; i < 5; i++) {
      readReg(BMX160_PMU_STATUS_ADDR, &pmu_status, 1);

      // 檢查 Accel (bits 1:0) 和 Gyro (bits 3:2) 是否都是 0b01 (Normal mode)
      bool accel_ready = ((pmu_status & 0x03) == 0x01);
      bool gyro_ready = (((pmu_status >> 2) & 0x03) == 0x01);

      if (accel_ready && gyro_ready) {
        LOG_INF("IMU initialized successfully (PMU Status: 0x%02X)",
                pmu_status);

        // ==========================================
        // 5. 喚醒成功後，進行陀螺儀配置
        // ==========================================

        // 配置陀螺儀 ODR 與頻寬 (暫存器 0x42: BMX160_GYRO_CONFIG_ADDR)
        // 0x28 代表 ODR = 100Hz, Normal filter (可依您的專案需求修改)
        writeBmxReg(BMX160_GYRO_CONFIG_ADDR, 0x28);
        delay(2);

        // 配置陀螺儀量程 Range (暫存器 0x43)
        // 0x00 代表量程 = ±2000°/s (可依您的專案需求修改)
        // 若您的程式庫中已有 BMX160_GYRO_RANGE_ADDR 的巨集，也可替換掉 0x43
        writeBmxReg(0x43, 0x00);
        delay(2);

        // (如果需要配置加速度計 0x40/0x41，也可以繼續加在這裡)

        return true; // 初始化且配置成功
      }

      // 如果狀態不對，讀取錯誤代碼並印出警告
      readReg(BMX160_ERROR_REG_ADDR, &err_reg, 1);
      LOG_WRN("IMU not ready (PMU: 0x%02X, ERR: 0x%02X), wait and retry...",
              pmu_status, err_reg);

      // 狀態未達標時先等待，不要瘋狂重發指令以免鎖死 PMU
      delay(50);

      // 只有在等待兩次都沒反應時，才嘗試重發一次喚醒指令
      if (i == 2) {
        LOG_WRN("Re-issuing Gyro normal command...");
        writeBmxReg(BMX160_COMMAND_REG_ADDR, 0x15);
        delay(100);
      }
    }

    // 如果跑完 5 次迴圈還是失敗，攔截並回傳 false，防止回傳全為 0 的假數據
    LOG_ERR("Fatal: IMU failed to enter normal mode. Halting initialization.");
    return false;

  } else {
    LOG_ERR("I2C Scan failed, device not found on bus.");
    return false;
  }
}

bool DFRobot_BMI160::begin_async() {
  _i2c->begin();

  if (scan() == true) {
    if (!softReset())
      return false;

    uint8_t err_reg = 0;
    // Clear any pending error flags before starting
    readReg(BMX160_ERROR_REG_ADDR, &err_reg, 1);

    // Accel normal mode (takes up to 5ms)
    writeBmxReg(BMX160_COMMAND_REG_ADDR, 0x11);
    delay(50);

    // Clear any error from changing accel mode
    readReg(BMX160_ERROR_REG_ADDR, &err_reg, 1);
    if (err_reg) {
      LOG_DBG("ERR_REG after Accel init: 0x%02X", err_reg);
    }

    // Start Gyro and Mag but DO NOT BLOCK with long retries
    writeBmxReg(BMX160_COMMAND_REG_ADDR, 0x15);
    delay(100); // Minimal delay just to let the I2C bus breathe
    writeBmxReg(BMX160_COMMAND_REG_ADDR, 0x19);
    delay(10);

    return true;
  } else {
    return false;
  }
}

bool DFRobot_BMI160::check_pmu_status(uint8_t *pmu_val, uint8_t *err_val) {
  uint8_t pmu = 0, err = 0;
  readReg(BMX160_PMU_STATUS_ADDR, &pmu, 1);
  readReg(BMX160_ERROR_REG_ADDR, &err, 1);

  if (pmu_val)
    *pmu_val = pmu;
  if (err_val)
    *err_val = err;

  // PMU GYR (bits 3:2) == 0b01 is normal mode
  bool gyro_ready = (((pmu >> 2) & 0x03) == 0x01);

  return gyro_ready;
}

void DFRobot_BMI160::retry_gyro_mag_wakeup() {
  // Send wake up commands again quickly
  writeBmxReg(BMX160_COMMAND_REG_ADDR, 0x15);
  // We only sleep a tiny bit here to not block the current thread. The actual
  // wait time will be handled by the delayed work queue.
  k_busy_wait(1000); // 1ms
  writeBmxReg(BMX160_COMMAND_REG_ADDR, 0x19);
}

void DFRobot_BMI160::setLowPower() {
  // Ignored for this circuit:
  // softReset();
  // delay(100);
  // writeBmxReg(BMX160_COMMAND_REG_ADDR, 0x12); // Accel low power
  // delay(100);
  // /* Set gyro to fast startup mode */
  // writeBmxReg(BMX160_COMMAND_REG_ADDR, 0x15);
  // delay(100);
}

void DFRobot_BMI160::wakeUp() {
  softReset();

  uint8_t err_reg = 0;
  readReg(BMX160_ERROR_REG_ADDR, &err_reg, 1);

  writeBmxReg(BMX160_COMMAND_REG_ADDR, 0x11); // Accel normal mode
  delay(10);

  readReg(BMX160_ERROR_REG_ADDR, &err_reg, 1);

  /* Set gyro to normal mode */
  writeBmxReg(BMX160_COMMAND_REG_ADDR, 0x15);
  delay(100);

  // Quick fallback check
  uint8_t pmu_status = 0;
  readReg(BMX160_PMU_STATUS_ADDR, &pmu_status, 1);
  if (((pmu_status >> 2) & 0x03) != 0x01) {
    readReg(BMX160_ERROR_REG_ADDR, &err_reg, 1);
    LOG_WRN("wakeUp failed to set Gyro normal mode (PMU: 0x%02X, ERR: 0x%02X)",
            pmu_status, err_reg);
  }
}

bool DFRobot_BMI160::softReset() {
  int8_t rslt = BMX160_OK;
  if (Obmx160 == NULL) {
    rslt = BMX160_E_NULL_PTR;
  }
  rslt = _softReset(Obmx160);
  if (rslt == 0)
    return true;
  else
    return false;
}

int8_t DFRobot_BMI160::_softReset(sBmx160Dev_t *dev) {
  int8_t rslt = BMX160_OK;
  uint8_t data = BMX160_SOFT_RESET_CMD;
  if (dev == NULL) {
    rslt = BMX160_E_NULL_PTR;
  }
  writeBmxReg(BMX160_COMMAND_REG_ADDR, data);
  delay(BMX160_SOFT_RESET_DELAY_MS);
  if (rslt == BMX160_OK) {
    DFRobot_BMI160::defaultParamSettg(dev);
  }
  return rslt;
}

void DFRobot_BMI160::defaultParamSettg(sBmx160Dev_t *dev) {
  // Initializing accel and gyro params with
  dev->gyroCfg.bw = BMX160_GYRO_BW_NORMAL_MODE;
  dev->gyroCfg.odr = BMX160_GYRO_ODR_100HZ;
  dev->gyroCfg.power = BMX160_GYRO_SUSPEND_MODE;
  dev->gyroCfg.range = BMX160_GYRO_RANGE_2000_DPS;

  dev->accelCfg.bw = BMX160_ACCEL_BW_NORMAL_AVG4;
  dev->accelCfg.odr = BMX160_ACCEL_ODR_100HZ;
  dev->accelCfg.power = BMX160_ACCEL_SUSPEND_MODE;
  dev->accelCfg.range = BMX160_ACCEL_RANGE_2G;

  dev->magnCfg.odr = BMX160_MAGN_ODR_100HZ;
  dev->magnCfg.power = BMX160_MAGN_SUSPEND_MODE;

  dev->prevMagnCfg = dev->magnCfg;
  dev->prevGyroCfg = dev->gyroCfg;
  dev->prevAccelCfg = dev->accelCfg;
}

void DFRobot_BMI160::setMagnConf() {
  // puts magnetometer into mag_if setup mode
  writeBmxReg(BMX160_MAGN_IF_0_ADDR, 0x80);
  delay(50);
  // Sleep mode
  writeBmxReg(BMX160_MAGN_IF_3_ADDR, 0x01);
  writeBmxReg(BMX160_MAGN_IF_2_ADDR, 0x4B);
  // REPXY regular preset
  writeBmxReg(BMX160_MAGN_IF_3_ADDR, 0x04);
  writeBmxReg(BMX160_MAGN_IF_2_ADDR, 0x51);
  // REPZ regular preset
  writeBmxReg(BMX160_MAGN_IF_3_ADDR, 0x0E);
  writeBmxReg(BMX160_MAGN_IF_2_ADDR, 0x52);
  // Prepare MAG_IF[1-3] for mag_if data mode
  writeBmxReg(BMX160_MAGN_IF_3_ADDR, 0x02);
  writeBmxReg(BMX160_MAGN_IF_2_ADDR, 0x4C);
  writeBmxReg(BMX160_MAGN_IF_1_ADDR, 0x42);
  // sets the sampling rate t0 100Hz
  writeBmxReg(BMX160_MAGN_CONFIG_ADDR, 0x08);
  // puts magnetometer into mag_if data mode sets data length of read burst
  // operation to 8 bytes
  writeBmxReg(BMX160_MAGN_IF_0_ADDR, 0x03);
  delay(50);
}

void DFRobot_BMI160::setGyroPowerMode(uint8_t mode) {
  writeBmxReg(BMX160_COMMAND_REG_ADDR, mode);
}

void DFRobot_BMI160::setAccelPowerMode(uint8_t mode) {
  writeBmxReg(BMX160_COMMAND_REG_ADDR, mode);
}

void DFRobot_BMI160::setGyroRange(eGyroRange_t bits) {
  switch (bits) {
  case eGyroRange_125DPS:
    gyroRange = BMX160_GYRO_SENSITIVITY_125DPS;
    break;
  case eGyroRange_250DPS:
    gyroRange = BMX160_GYRO_SENSITIVITY_250DPS;
    break;
  case eGyroRange_500DPS:
    gyroRange = BMX160_GYRO_SENSITIVITY_500DPS;
    break;
  case eGyroRange_1000DPS:
    gyroRange = BMX160_GYRO_SENSITIVITY_1000DPS;
    break;
  case eGyroRange_2000DPS:
    gyroRange = BMX160_GYRO_SENSITIVITY_2000DPS;
    break;
  default:
    gyroRange = BMX160_GYRO_SENSITIVITY_250DPS;
    break;
  }
}

void DFRobot_BMI160::setAccelRange(eAccelRange_t bits) {
  switch (bits) {
  case eAccelRange_2G:
    accelRange = BMX160_ACCEL_MG_LSB_2G * EARTH_ACC;
    break;
  case eAccelRange_4G:
    accelRange = BMX160_ACCEL_MG_LSB_4G * EARTH_ACC;
    break;
  case eAccelRange_8G:
    accelRange = BMX160_ACCEL_MG_LSB_8G * EARTH_ACC;
    break;
  case eAccelRange_16G:
    accelRange = BMX160_ACCEL_MG_LSB_16G * EARTH_ACC;
    break;
  default:
    accelRange = BMX160_ACCEL_MG_LSB_2G * EARTH_ACC;
    break;
  }

  writeBmxReg(BMX160_ACCEL_RANGE_ADDR, bits);
}

void DFRobot_BMI160::setMagnODR(uint8_t val) {
  writeBmxReg(BMX160_MAGN_CONFIG_ADDR, BMX160_MAGN_ODR_MASK & val);
}

void DFRobot_BMI160::setGyroODR(uint8_t val) {
  writeBmxReg(BMX160_GYRO_CONFIG_ADDR, BMX160_GYRO_ODR_MASK & val);
}

void DFRobot_BMI160::setAccelODR(uint8_t val) {
  writeBmxReg(BMX160_ACCEL_CONFIG_ADDR, BMX160_ACCEL_ODR_MASK & val);
}

void DFRobot_BMI160::getAllData(sBmx160SensorData_t *magn,
                                sBmx160SensorData_t *gyro,
                                sBmx160SensorData_t *accel) {

  // BMI160: read 12 bytes from 0x0C (6 bytes gyro + 6 bytes accel)
  uint8_t data[12] = {0};
  int16_t x = 0, y = 0, z = 0;

  readReg(BMX160_GYRO_DATA_ADDR, data, 12);

  if (magn) {
    // No magnetometer on BMI160 — zero out
    magn->x = 0;
    magn->y = 0;
    magn->z = 0;
  }
  if (gyro) {
    x = (int16_t)(((uint16_t)data[1] << 8) | data[0]);
    y = (int16_t)(((uint16_t)data[3] << 8) | data[2]);
    z = (int16_t)(((uint16_t)data[5] << 8) | data[4]);
    gyro->x = x * gyroRange;
    gyro->y = y * gyroRange;
    gyro->z = z * gyroRange;
  }
  if (accel) {
    // Accel data follows gyro at offset 6
    x = (int16_t)(((uint16_t)data[7] << 8) | data[6]);
    y = (int16_t)(((uint16_t)data[9] << 8) | data[8]);
    z = (int16_t)(((uint16_t)data[11] << 8) | data[10]);
    accel->x = x * accelRange;
    accel->y = y * accelRange;
    accel->z = z * accelRange;
  }
}

void DFRobot_BMI160::writeBmxReg(uint8_t reg, uint8_t value) {
  _i2c->aquire();
  int ret = i2c_reg_write_byte(_i2c->master, _addr, reg, value);
  if (ret)
    LOG_WRN("I2C writeBmxReg failed: %d", ret);
  _i2c->release();
}

void DFRobot_BMI160::writeReg(uint8_t reg, uint8_t *pBuf, uint16_t len) {
  _i2c->aquire();

  int ret = i2c_burst_write(_i2c->master, _addr, reg, pBuf, len);
  if (ret)
    LOG_WRN("I2C write failed: %d", ret);

  _i2c->release();
}

void DFRobot_BMI160::readReg(uint8_t reg, uint8_t *pBuf, uint16_t len) {
  _i2c->aquire();

  int ret = i2c_burst_read(_i2c->master, _addr, reg, pBuf, len);
  if (ret)
    LOG_WRN("I2C read failed: %d", ret);

  _i2c->release();
}

bool DFRobot_BMI160::scan() {
  _i2c->aquire();

  uint8_t chip_id = 0;
  int ret_68 = -1, ret_69 = -1;
  uint8_t found_addr = 0;

  if (!device_is_ready(_i2c->master)) {
    LOG_ERR("I2C bus not ready for BMI160 scan");
    _i2c->release();
    return false;
  }

  // Try probing up to 10 times for wake up
  for (int i = 0; i < 10; i++) {
    ret_68 = i2c_reg_read_byte(_i2c->master, 0x68, 0x00, &chip_id);
    if (ret_68 == 0) {
      found_addr = 0x68;
      break;
    }

    ret_69 = i2c_reg_read_byte(_i2c->master, 0x69, 0x00, &chip_id);
    if (ret_69 == 0) {
      found_addr = 0x69;
      break;
    }
    LOG_DBG("IMU scan attempt %d failed (ret_68=%d, ret_69=%d)", i, ret_68,
            ret_69);
    k_usleep(10000); // 10ms wait
  }

  _i2c->release();

  if (found_addr != 0) {
    _addr = found_addr; // Update runtime I2C address
    LOG_INF("IMU successfully scanned at address 0x%02X, Chip ID: 0x%02X",
            _addr, chip_id);
    if (chip_id == 0xD1 || chip_id == 0xD8) {
      return true;
    } else {
      LOG_WRN("IMU Unknown Chip ID: 0x%02X", chip_id);
      return false;
    }
  } else {
    LOG_ERR("I2C read for IMU scan failed for both 0x68 and 0x69 (ret_68: %d, "
            "ret_69: %d)",
            ret_68, ret_69);
    return false;
  }
}
