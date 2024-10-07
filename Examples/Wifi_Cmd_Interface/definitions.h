#ifndef DEFINITIONS_H
#define DEFINITIONS_H

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

typedef enum _car_mode
{
  CAR_MODE_IDLE = 0,
  CAR_MODE_MANUAL,
  CAR_MODE_HEADING_KEEP,
  CAR_MODE_WAYPOINT,
} car_mode;

typedef struct _car_data
{
  int left_speed;
  int right_speed;

  car_mode mode;
  int manual_mode_left_speed;
  int mnaual_mode_right_speed;
  uint32_t manual_move_duration;
  uint32_t _manual_move_start_time;
} car_data;

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