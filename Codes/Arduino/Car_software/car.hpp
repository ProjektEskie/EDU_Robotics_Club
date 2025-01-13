#ifndef RCLUB_CAR_H
#define RCLUB_CAR_H

#include <arduino.h>
#include <Servo.h>

#define IS_EGLOO_PLATFORM 1 // Freenove if 0

#define PIN_BATTERY_SENSE A0

#if IS_EGLOO_PLATFORM
  #define LEFT_DIRECTION_PIN A3
  #define LEFT_DIRECTION_PIN_IN2 8
  #define RIGHT_DIRECTION_PIN 9
  #define RIGHT_DIRECTION_PIN_IN4 11
  #define LEFT_SPEED_PIN 5
  #define RIGHT_SPEED_PIN 6
  #define SERVO_PIN 3
#else
  #define LEFT_DIRECTION_PIN 4
  #define RIGHT_DIRECTION_PIN 3
  #define LEFT_SPEED_PIN 6
  #define RIGHT_SPEED_PIN 5
  #define SERVO_PIN 2
#endif

typedef enum _car_mode
{
  CAR_MODE_IDLE = 0,
  CAR_MODE_MANUAL,
  CAR_MODE_HEADING_KEEP,
  CAR_MODE_PNG,  // Point and go mode.
  N_CAR_MODES
} car_mode;

typedef struct _car_manual_mode_data
{
  int mm_left_speed;
  int mm_right_speed;
  uint32_t mm_duration;
  uint32_t _mm_start_time;
} car_manual_mode_data;

typedef struct _car_heading_keep_mode_data
{
  float target_heading;
  float _heading_difference;
} car_heading_keep_mode_data;

typedef enum _car_png_mode_steps
{
  CAR_PNG_GOTO_HEADING,
  CAR_PNG_LINEAR_TRAVEL,
  CAR_PNG_DONE
} car_png_mode_steps;

typedef struct _car_point_and_go_mode_data
{
  car_png_mode_steps step;
  float target_heading;
  int straight_line_speed;
  uint32_t straight_line_start_time;
  uint32_t straight_line_duration;
} car_png_mode_data;

typedef struct _car_data
{
  int left_speed;
  int right_speed;
  int servo_angle;
  int servo_angle_offset;

  car_mode mode;
  car_mode _prev_mode;
  bool is_new_mode;

  Servo front_servo;

  car_manual_mode_data mm_data;
  car_heading_keep_mode_data hk_data;
  car_png_mode_data png_data;
} car_data;

void CAR_init();
void CAR_update();

void CAR_API_car_m_move(int left_speed, int right_speed, uint32_t duration);
void CAR_API_set_mode(uint8_t requested_mode);
void CAR_API_set_heading(float requested_heading);
void CAR_API_set_PNG_settings(float target_heading, int line_speed, uint32_t line_duration);
void CAR_API_set_Servo_angle(int angle);

#endif