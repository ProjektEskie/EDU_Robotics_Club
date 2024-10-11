#ifndef DEFINITIONS_H
#define DEFINITIONS_H

#include <arduino.h>
#include "car.hpp"
#include "Arduino_NineAxesMotion.h"

#define CMD_INPUT_BUFFER_LEN 256
#define CMD_OUTPUT_BUFFER_LEN 256



typedef struct _imu_data
{
  NineAxesMotion mySensor; 
  uint32_t last_update_time;
  uint32_t update_interval;
  uint16_t n_updates_counter;
  int system_calibration_status;
  float euler_heading;
  float euler_pitch;
  float euler_roll;
} imu_data;

enum _op_status_bits
{
  IS_WIFI_CONNECTED = 0,
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