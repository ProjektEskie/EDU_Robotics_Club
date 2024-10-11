#ifndef CAR_DEFINITIONS_H
#define CAR_DEFINITIONS_H

#define IS_EGLOO_PLATFORM 1

#define PIN_BATTERY_SENSE A0

#if IS_EGLOO_PLATFORM
  #define left_direction_pin A3
  #define left_direction_pin_in2 8
  #define right_direction_pin 9
  #define right_direction_pin_in4 11
  #define left_speed_pin 5
  #define right_speed_pin 6
#else
  #define left_direction_pin 4
  #define right_direction_pin 3
  #define left_speed_pin 6
  #define right_speed_pin 5
#endif

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

#endif