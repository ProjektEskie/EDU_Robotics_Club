#ifndef DEFINITIONS_H
#define DEFINITIONS_H

#include <arduino.h>
#include "car.hpp"
#include "IMU.hpp"




enum _op_status_bits
{
  IS_WIFI_CONNECTED = 0,
  IS_MOVING,
};

typedef struct _operation_data
{
  unsigned int status;
  uint32_t time_now;
  uint32_t time_since_last_telemetry;

  imu_data imu;
  car_data car;

} operation_data;

#endif