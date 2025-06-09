#ifndef RCLUB_CAR_H
#define RCLUB_CAR_H

#include <arduino.h>
#include <Servo.h>
#include "personal_config.hpp"

#define PIN_BATTERY_SENSE A0
#define MAX_ECHO_DISTANCE_CM 200
#define SPEED_OF_SOUND 343 // m/s
#define ECHO_TIMEOUT_US 10000
#define RANGING_DATA_SIZE 7

#define OBSTACLE_AVOID_DURATION 450

#if IS_EGLOO_PLATFORM
  #define LEFT_DIRECTION_PIN A3
  #define LEFT_DIRECTION_PIN_IN2 8
  #define RIGHT_DIRECTION_PIN 9
  #define RIGHT_DIRECTION_PIN_IN4 11
  #define LEFT_SPEED_PIN 5
  #define RIGHT_SPEED_PIN 6
  #define SERVO_PIN 3
  #define ECHO_TRIGGER_PIN A2
  #define ECHO_RESULT_PIN A1
#else
  #define LEFT_DIRECTION_PIN 4
  #define RIGHT_DIRECTION_PIN 3
  #define LEFT_SPEED_PIN 6
  #define RIGHT_SPEED_PIN 5
  #define SERVO_PIN 9
  #define ECHO_TRIGGER_PIN 10
  #define ECHO_RESULT_PIN 11
#endif

typedef enum _car_mode
{
  CAR_MODE_IDLE = 0,
  CAR_MODE_MANUAL,
  CAR_MODE_HEADING_KEEP,
  CAR_MODE_PNG,  // Point and go mode.
  CAR_MODE_AUTO,
  N_CAR_MODES
} car_mode;

typedef struct _ranging_data
{
  const int scan_angle[RANGING_DATA_SIZE] = {-85, -60, -30, 0, 30, 60, 85};
  const int scan_dewell_time[RANGING_DATA_SIZE] = {350, 200, 200, 200, 200, 200, 200};
  float distance[RANGING_DATA_SIZE];
} ranging_data;

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

typedef enum _car_auto_mode_steps
{
  CAR_AUTO_INIT = 0,
  CAR_AUTO_GOTO_HEADING,
  CAR_AUTO_HEADING_SETTLE,
  CAR_AUTO_LINEAR_TRAVEL,
  CAR_AUTO_OBSTACLE_DETECTED,
  CAR_ATUO_BRAKE_START,
  CAR_AUTO_BRAKE_COMPLETE,
  CAR_AUTO_DELAY_START,
  CAR_AUTO_OBSTACLE_AVOID_TURN,
  CAR_AUTO_OBSTACLE_AVOID_ADVANCE,
  CAR_AUTO_OBSTACLE_AVOID_CHECK,
  CAR_AUTO_RETURN_TURN,
  CAR_AUTO_RETURN_ADVANCE,
  CAR_AUTO_DONE
} car_auto_mode_steps;

typedef struct _auto_mode_saved_data
{
  car_auto_mode_steps step_at_state_change;
  int forward_speed;
  float target_heading_absuolute;
  uint32_t forward_time_remaining;
} auto_mode_saved_data;

typedef struct _car_auto_mode_data
{
  car_auto_mode_steps step;
  float starting_heading;
  int forward_speed;
  float target_heading_delta;
  float target_heading_absuolute;
  float obstacle_avoid_heading;
  uint32_t obstacle_avoid_start_time;
  float return_heading;
  uint32_t return_duration;
  uint32_t forward_duration;
  uint32_t forward_time_remaining;
  int reverse_speed;
  uint32_t reverse_duration;
  uint8_t _allow_heading_realignment;
  uint32_t _heading_turn_complete_time;
  uint32_t _forward_start_time;
  uint32_t _reverse_start_time;
  uint32_t _turn_start_time;
  uint32_t _delay_start_time;
  uint32_t _delay_duration;
  int32_t _time_traveled_in_target_heading;
  uint32_t _prev_loop_timestamp;
  uint32_t _return_start_time;
  int _n_advance_completed;
  car_auto_mode_steps post_delay_step;

  int range_infront;

  auto_mode_saved_data on_delay_exit;

} car_auto_mode_data;

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
  bool is_ranging_data_ready;
  bool is_ranging_requested;
  int ranging_data_index;
  ranging_data rd;

  car_manual_mode_data mm_data;
  car_heading_keep_mode_data hk_data;
  car_png_mode_data png_data;
  car_auto_mode_data am_data;
} car_data;

void CAR_init();
void CAR_update();
void CAR_servo_update();
int CAR_echo_range_cm(); // Returns the distance infront of the ranging sensor in cm

void CAR_API_car_m_move(int left_speed, int right_speed, uint32_t duration);
void CAR_API_set_mode(uint8_t requested_mode);
void CAR_API_set_heading(float requested_heading);
void CAR_API_set_PNG_settings(float target_heading, int line_speed, uint32_t line_duration);
void CAR_API_set_auto_settings(int forward_speed, float target_heading, uint32_t forward_duration);
void CAR_API_set_Servo_angle(int angle);
void CAR_API_start_ranging_scan();
#endif