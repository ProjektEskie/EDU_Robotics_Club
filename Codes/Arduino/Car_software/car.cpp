#include "car.hpp"
#include "definitions.hpp"
#include "helpers.hpp"

bool CAR_turn_to_heading(float target_heading);
void CAR_stop();
void CAR_commit_speed();

extern operation_data op_data;


void CAR_init()
{
  pinMode(PIN_BATTERY_SENSE, INPUT);
  pinMode(LEFT_DIRECTION_PIN, OUTPUT);
  pinMode(RIGHT_DIRECTION_PIN, OUTPUT);
  pinMode(LEFT_SPEED_PIN, OUTPUT);
  pinMode(RIGHT_SPEED_PIN, OUTPUT);

  #if IS_EGLOO_PLATFORM
  pinMode(LEFT_DIRECTION_PIN_IN2, OUTPUT);
  pinMode(RIGHT_DIRECTION_PIN_IN4, OUTPUT);
  #endif

  op_data.car.mode = CAR_MODE_IDLE;
}

void CAR_update()
{

  if (op_data.car.mode == CAR_MODE_IDLE)
  {
      if (op_data.car.is_new_mode)
      {
        op_data.car.is_new_mode = false;
      }
      op_data.car.left_speed = 0;
      op_data.car.right_speed = 0;
  }
  else if (op_data.car.mode == CAR_MODE_MANUAL)
  {
      if (op_data.car.is_new_mode)
      {
        op_data.car.is_new_mode = false;
        op_data.car.left_speed = op_data.car.mm_data.mm_left_speed;
        op_data.car.right_speed = op_data.car.mm_data.mm_right_speed;
      }

    if ((op_data.time_now - op_data.car.mm_data._mm_start_time) > op_data.car.mm_data.mm_duration)
    {
      CAR_API_set_mode(op_data.car._prev_mode);
      op_data.car.left_speed = 0;
      op_data.car.right_speed = 0;
      helper_queue_messages("Info: manual move complete");
    }
  }
  else if (op_data.car.mode == CAR_MODE_HEADING_KEEP)
  {
    if (op_data.car.is_new_mode)
    {
      op_data.car.is_new_mode = false;
    }

    CAR_turn_to_heading(op_data.car.hk_data.target_heading);
  }
  else if (op_data.car.mode == CAR_MODE_PNG)
  {
    if (op_data.car.is_new_mode)
    {
      op_data.car.png_data.step = CAR_PNG_GOTO_HEADING;
      op_data.car.is_new_mode = false;
    }

    if (op_data.car.png_data.step == CAR_PNG_GOTO_HEADING)
    {
      if (CAR_turn_to_heading(op_data.car.png_data.target_heading))
      {
        op_data.car.png_data.step = CAR_PNG_LINEAR_TRAVEL;
        op_data.car.png_data.straight_line_start_time = op_data.time_now;
        op_data.car.left_speed = op_data.car.png_data.straight_line_speed;
        op_data.car.right_speed = op_data.car.png_data.straight_line_speed;
      }
    }
    else if (op_data.car.png_data.step == CAR_PNG_LINEAR_TRAVEL)
    {
      if ((op_data.time_now - op_data.car.png_data.straight_line_start_time) > op_data.car.png_data.straight_line_duration)
      {
        op_data.car.png_data.step = CAR_PNG_DONE;
      }
    }
    else if (op_data.car.png_data.step == CAR_PNG_DONE)
    {
      CAR_stop();
    }
    else
    {
      CAR_API_set_mode(CAR_MODE_IDLE);
      helper_queue_messages("CAR, ERROR: unexpected step in PNG mode");
    }

  }
  else
  {
  }

  CAR_commit_speed();
}

void CAR_API_car_m_move(int left_speed, int right_speed, uint32_t duration)
{
  CAR_API_set_mode(CAR_MODE_MANUAL);
  op_data.car.mm_data.mm_left_speed = left_speed;
  op_data.car.mm_data.mm_right_speed = right_speed;
  op_data.car.mm_data.mm_duration = duration;
  op_data.car.mm_data._mm_start_time = op_data.time_now;
}

void CAR_API_set_mode(uint8_t requested_mode)
{
  if (requested_mode < N_CAR_MODES)
  {
    car_mode req_mode = (car_mode)requested_mode;
    if (req_mode != op_data.car.mode)
    {
      op_data.car._prev_mode = op_data.car.mode;
      op_data.car.mode = (car_mode)requested_mode;
      op_data.car.is_new_mode = true;
    }
  }
  else
  {
    helper_queue_messages("Error: car_set_mode, invalid mode");
  }
}

void CAR_API_set_heading(float requested_heading)
{
  if ((requested_heading >= 0.0) && (requested_heading < 360.0))
  {
    op_data.car.hk_data.target_heading = requested_heading;
  }
  else
  {
    helper_queue_messages("Error: car_set_heading, heading must be between 0 and 360");
  }
}

// Generate the speed the car needs to head turn to the target heading
// Commit_speed() still need to be called to move the car
// returns true when the car is facing the target heading
bool CAR_turn_to_heading(float target_heading)
{
  bool is_done = false;
  float _dff = helper_angle_diff(op_data.imu.euler_heading, target_heading);

  int turn_speed = 220;
  int angle_tolerance = 2.5;

  if (_dff > angle_tolerance)
  {
    op_data.car.left_speed = -turn_speed;
    op_data.car.right_speed = turn_speed;
  }
  else if (_dff < -angle_tolerance)
  {
    op_data.car.left_speed = turn_speed;
    op_data.car.right_speed = -turn_speed;
  }
  else
  {
    CAR_stop();
    is_done = true;
  }
  return is_done;
}

void CAR_stop()
{
  op_data.car.left_speed = 0;
  op_data.car.right_speed = 0;
  CAR_commit_speed();
}

void CAR_commit_speed()
{
  int CAR_left_speed = op_data.car.left_speed;
  int CAR_right_speed = op_data.car.right_speed;
  int abs_left_speed;
  int abs_right_speed;
  abs_left_speed = abs(CAR_left_speed);
  abs_right_speed = abs(CAR_right_speed);

  if (CAR_left_speed >= 0)
  {
    #if IS_EGLOO_PLATFORM
      digitalWrite(LEFT_DIRECTION_PIN, HIGH);
      digitalWrite(LEFT_DIRECTION_PIN_IN2, LOW);
    #else
      digitalWrite(LEFT_DIRECTION_PIN, HIGH);
    #endif
  }
  else
  {
    
    #if IS_EGLOO_PLATFORM
      digitalWrite(LEFT_DIRECTION_PIN, LOW);
      digitalWrite(LEFT_DIRECTION_PIN_IN2, HIGH);
    #else
      digitalWrite(LEFT_DIRECTION_PIN, LOW);
    #endif
  }

  if (CAR_right_speed >= 0)
  {
    
    #if IS_EGLOO_PLATFORM
      digitalWrite(RIGHT_DIRECTION_PIN, LOW);
      digitalWrite(RIGHT_DIRECTION_PIN_IN4, HIGH);
    #else
      digitalWrite(RIGHT_DIRECTION_PIN, LOW);
    #endif
  }
  else
  {
    
    #if IS_EGLOO_PLATFORM
      digitalWrite(RIGHT_DIRECTION_PIN, HIGH);
      digitalWrite(RIGHT_DIRECTION_PIN_IN4, LOW);
    #else
      digitalWrite(RIGHT_DIRECTION_PIN, HIGH);
    #endif
  }
  analogWrite(LEFT_SPEED_PIN, abs_left_speed);
  analogWrite(RIGHT_SPEED_PIN, abs_right_speed);
}