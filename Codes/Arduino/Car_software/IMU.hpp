#ifndef RCLUB_IMU_H
#define RCLUB_IMU_H

#include <arduino.h>
#include "Arduino_NineAxesMotion.h"

#define RELATIVE_MODE OPERATION_MODE_IMUPLUS
#define ABSOLUTE_MODE OPERATION_MODE_NDOF

typedef struct _imu_data
{
  NineAxesMotion mySensor; 
  uint16_t n_updates_counter;

  int system_calibration_status;
  int gryo_calibration_status;
  int mag_calibration_status;
  int accel_calibration_status;
  
  float euler_heading;
  float euler_pitch;
  float euler_roll;
  
  float gyro_z;

  float linaccel_x;
  float linaccel_y;
  float linaccel_z;

} imu_data;

void IMU_Init();
void IMU_update();
void IMU_reset_n_updates_counter();
void API_IMU_set_absolute_mode(bool is_absolute);

#endif