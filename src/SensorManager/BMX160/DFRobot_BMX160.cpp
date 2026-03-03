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
    debugDumpRegisters("begin:pre_reset");
    softReset();
    debugDumpRegisters("begin:post_reset");
    writeBmxReg(BMX160_COMMAND_REG_ADDR, 0x11);
    delay(50);
    debugDumpRegisters("begin:after_accel_normal");
    /* Set gyro to normal mode */
    writeBmxReg(BMX160_COMMAND_REG_ADDR, 0x15);
    delay(100);
    debugDumpRegisters("begin:after_gyro_normal");
    return true;
  } else
    return false;
}

void DFRobot_BMI160::debugDumpRegisters(const char *tag) {
  uint8_t chip_id = 0;
  uint8_t err = 0;
  uint8_t pmu = 0;
  uint8_t status = 0;
  uint8_t acc_cfg = 0;
  uint8_t acc_rng = 0;
  uint8_t gyr_cfg = 0;
  uint8_t gyr_rng = 0;

  readReg(BMX160_CHIP_ID_ADDR, &chip_id, 1);
  readReg(BMX160_ERROR_REG_ADDR, &err, 1);
  readReg(0x03, &pmu, 1); /* PMU_STATUS */
  readReg(BMX160_STATUS_ADDR, &status, 1);
  readReg(BMX160_ACCEL_CONFIG_ADDR, &acc_cfg, 1);
  readReg(BMX160_ACCEL_RANGE_ADDR, &acc_rng, 1);
  readReg(BMX160_GYRO_CONFIG_ADDR, &gyr_cfg, 1);
  readReg(BMX160_GYRO_RANGE_ADDR, &gyr_rng, 1);

  LOG_INF("[%s] BMI160 regs: CHIP=0x%02X ERR=0x%02X PMU=0x%02X ST=0x%02X "
          "ACC_CFG=0x%02X ACC_RNG=0x%02X GYR_CFG=0x%02X GYR_RNG=0x%02X",
          tag ? tag : "dump", chip_id, err, pmu, status, acc_cfg, acc_rng,
          gyr_cfg, gyr_rng);
}

void DFRobot_BMI160::setLowPower() {
  softReset();
  delay(100);
  // setMagnConf();  // BMI160 has no magnetometer
  // delay(100);
  writeBmxReg(BMX160_COMMAND_REG_ADDR, 0x12); // Accel low power
  delay(100);
  /* Set gyro to fast startup mode */
  writeBmxReg(BMX160_COMMAND_REG_ADDR, 0x17);
  delay(100);
  // writeBmxReg(BMX160_COMMAND_REG_ADDR, 0x1B); // No mag on BMI160
  // delay(100);
}

void DFRobot_BMI160::wakeUp() {
  softReset();
  delay(100);
  // setMagnConf();  // BMI160 has no magnetometer
  // delay(100);
  writeBmxReg(BMX160_COMMAND_REG_ADDR, 0x11); // Accel normal mode
  delay(100);
  /* Set gyro to normal mode */
  writeBmxReg(BMX160_COMMAND_REG_ADDR, 0x15);
  delay(100);
  // writeBmxReg(BMX160_COMMAND_REG_ADDR, 0x19); // No mag on BMI160
  // delay(100);
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

  /* Explicitly configure gyro range in hardware and verify. */
  writeBmxReg(BMX160_GYRO_RANGE_ADDR, bits);
  uint8_t readback = 0;
  readReg(BMX160_GYRO_RANGE_ADDR, &readback, 1);
  LOG_INF("Set gyro range reg=0x%02X readback=0x%02X", bits, readback);
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
  uint8_t current = 0;
  readReg(BMX160_GYRO_CONFIG_ADDR, &current, 1);
  uint8_t updated =
      (current & ~BMX160_GYRO_ODR_MASK) | (val & BMX160_GYRO_ODR_MASK);
  writeBmxReg(BMX160_GYRO_CONFIG_ADDR, updated);
  uint8_t readback = 0;
  readReg(BMX160_GYRO_CONFIG_ADDR, &readback, 1);
  LOG_INF("Set gyro ODR val=0x%02X cfg old=0x%02X new=0x%02X rb=0x%02X", val,
          current, updated, readback);
}

void DFRobot_BMI160::setAccelODR(uint8_t val) {
  uint8_t current = 0;
  readReg(BMX160_ACCEL_CONFIG_ADDR, &current, 1);
  uint8_t updated =
      (current & ~BMX160_ACCEL_ODR_MASK) | (val & BMX160_ACCEL_ODR_MASK);
  writeBmxReg(BMX160_ACCEL_CONFIG_ADDR, updated);
  uint8_t readback = 0;
  readReg(BMX160_ACCEL_CONFIG_ADDR, &readback, 1);
  LOG_INF("Set accel ODR val=0x%02X cfg old=0x%02X new=0x%02X rb=0x%02X", val,
          current, updated, readback);
}

void DFRobot_BMI160::getAllData(sBmx160SensorData_t *magn,
                                sBmx160SensorData_t *gyro,
                                sBmx160SensorData_t *accel) {

  // BMI160: read 12 bytes from 0x0C (6 bytes gyro + 6 bytes accel)
  uint8_t data[12] = {0};
  int16_t x = 0, y = 0, z = 0;
  int16_t gx_raw = 0, gy_raw = 0, gz_raw = 0;
  int16_t ax_raw = 0, ay_raw = 0, az_raw = 0;
  static uint32_t sample_count = 0;
  static uint32_t gyro_zero_streak = 0;

  readReg(BMX160_GYRO_DATA_ADDR, data, 12);

  if (magn) {
    // No magnetometer on BMI160 — zero out
    magn->x = 0; magn->y = 0; magn->z = 0;
  }
  if (gyro) {
    x = (int16_t)(((uint16_t)data[1] << 8) | data[0]);
    y = (int16_t)(((uint16_t)data[3] << 8) | data[2]);
    z = (int16_t)(((uint16_t)data[5] << 8) | data[4]);
    gx_raw = x;
    gy_raw = y;
    gz_raw = z;
    gyro->x = x * gyroRange;
    gyro->y = y * gyroRange;
    gyro->z = z * gyroRange;
  }
  if (accel) {
    // Accel data follows gyro at offset 6
    x = (int16_t)(((uint16_t)data[7] << 8) | data[6]);
    y = (int16_t)(((uint16_t)data[9] << 8) | data[8]);
    z = (int16_t)(((uint16_t)data[11] << 8) | data[10]);
    ax_raw = x;
    ay_raw = y;
    az_raw = z;
    accel->x = x * accelRange;
    accel->y = y * accelRange;
    accel->z = z * accelRange;
  }

  sample_count++;

  bool gyro_all_zero = (gx_raw == 0 && gy_raw == 0 && gz_raw == 0);
  if (gyro_all_zero) {
    gyro_zero_streak++;
  } else {
    gyro_zero_streak = 0;
  }

  if ((sample_count % 100) == 0 ||
      (gyro_zero_streak > 0 && (gyro_zero_streak % 50) == 0)) {
    uint8_t pmu = 0;
    uint8_t err = 0;
    uint8_t gyr_cfg = 0;
    uint8_t gyr_rng = 0;
    readReg(0x03, &pmu, 1); /* PMU_STATUS */
    readReg(BMX160_ERROR_REG_ADDR, &err, 1);
    readReg(BMX160_GYRO_CONFIG_ADDR, &gyr_cfg, 1);
    readReg(BMX160_GYRO_RANGE_ADDR, &gyr_rng, 1);

    LOG_INF("IMU sample#%u raw A[%d,%d,%d] G[%d,%d,%d] zero_streak=%u "
            "PMU=0x%02X ERR=0x%02X GCFG=0x%02X GRNG=0x%02X",
            sample_count, ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw,
            gyro_zero_streak, pmu, err, gyr_cfg, gyr_rng);
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
    LOG_DBG("IMU scan attempt %d failed (ret_68=%d, ret_69=%d)", i, ret_68, ret_69);
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
