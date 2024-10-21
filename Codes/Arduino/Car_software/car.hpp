#ifndef RCLUB_CAR_H
#define RCLUB_CAR_H

#include <arduino.h>

#define IS_EGLOO_PLATFORM 0

#define PIN_BATTERY_SENSE A0

#if IS_EGLOO_PLATFORM
  #define LEFT_DIRECTION_PIN A3
  #define LEFT_DIRECTION_PIN_IN2 8
  #define RIGHT_DIRECTION_PIN 9
  #define RIGHT_DIRECTION_PIN_IN4 11
  #define LEFT_SPEED_PIN 5
  #define RIGHT_SPEED_PIN 6
#else
  #define LEFT_DIRECTION_PIN 4
  #define RIGHT_DIRECTION_PIN 3
  #define LEFT_SPEED_PIN 6
  #define RIGHT_SPEED_PIN 5
#endif

typedef enum _car_mode
{
  CAR_MODE_IDLE = 0,
  CAR_MODE_MANUAL,
  CAR_MODE_HEADING_KEEP,
  CAR_MODE_WAYPOINT,
} car_mode;

typedef struct _car_manual_mode_data
{
  int mm_left_speed;
  int mm_right_speed;
  uint32_t mm_duration;
  uint32_t _mm_start_time;
} car_manual_mode_data;

typedef struct _car_data
{
  int left_speed;
  int right_speed;

  car_mode mode;

  car_manual_mode_data mm_data;
} car_data;

void CAR_init();
void CAR_update();

#endif