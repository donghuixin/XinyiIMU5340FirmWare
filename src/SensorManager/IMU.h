#ifndef IMU_H
#define IMU_H

#include "EdgeMLSensor.h"
#include <zephyr/kernel.h>

#include "BMX160/DFRobot_BMX160.h"
#include "openearable_common.h"

class IMU : public EdgeMlSensor {
public:
  static IMU sensor;
  static DFRobot_BMI160 imu;

  bool init(struct k_msgq *queue) override;
  void start(int sample_rate_idx) override;
  void stop() override;

  const static SampleRateSetting<6> sample_rates;

private:
  // const static int num_sample_rates = 6;
  // const static sample_rate_setting sample_rates[num_sample_rates];

  static void sensor_timer_handler(struct k_timer *dummy);

  static void update_sensor(struct k_work *work);
  static void imu_async_rescue_handler(struct k_work *work);
  bool _active = false;
};

#endif