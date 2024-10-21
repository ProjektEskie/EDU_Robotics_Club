#ifndef RCLUB_IMU_H
#define RCLUB_IMU_H

#include <arduino.h>
#include "Arduino_NineAxesMotion.h"

typedef struct _imu_data
{
  NineAxesMotion mySensor; 
  uint32_t last_update_time;
  uint32_t update_interval;
  uint16_t n_updates_counter;

  int system_calibration_status;
  int gryo_calibration_status;
  int mag_calibration_status;
  int accel_calibration_status;
  
  float euler_heading;
  float euler_pitch;
  float euler_roll;

  float linaccel_x;
  float linaccel_y;
  float linaccel_z;

} imu_data;

void IMU_Init();
void IMU_update();
void IMU_reset_n_updates_counter();

#endif