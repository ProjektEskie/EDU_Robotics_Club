#include "car.hpp"

void CAR_stop(car_data * cd);
void CAR_commit_speed(car_data * cd);

void CAR_init(car_data * cd)
{
  pinMode(PIN_BATTERY_SENSE, INPUT);
  pinMode(left_direction_pin, OUTPUT);
  pinMode(right_direction_pin, OUTPUT);
  pinMode(left_speed_pin, OUTPUT);
  pinMode(right_speed_pin, OUTPUT);

  #if IS_EGLOO_PLATFORM
  pinMode(left_direction_pin_in2, OUTPUT);
  pinMode(right_direction_pin_in4, OUTPUT);
  #endif

  cd->mode = CAR_MODE_IDLE;
}

void CAR_update(car_data * cd, uint32_t time_now)
{

  if (cd->mode == CAR_MODE_IDLE)
  {
      cd->left_speed = 0;
      cd->right_speed = 0;
  }
  else if (cd->mode == CAR_MODE_MANUAL)
  {
    cd->left_speed = cd->manual_mode_left_speed;
    cd->right_speed = cd->mnaual_mode_right_speed;
    if ((time_now - cd->_manual_move_start_time) > cd->manual_move_duration)
    {
      cd->mode = CAR_MODE_IDLE;
      cd->left_speed = 0;
      cd->right_speed = 0;
      helper_queue_messages("Info: manual move complete");
    }
  }
  else
  {
  }

  CAR_commit_speed(cd);
}

void CAR_stop(car_data * cd)
{
  cd->left_speed = 0;
  cd->right_speed = 0;
  CAR_commit_speed(cd);
}

void CAR_commit_speed(car_data * cd)
{
  int CAR_left_speed = cd->left_speed;
  int CAR_right_speed = cd->right_speed;
  int abs_left_speed;
  int abs_right_speed;
  abs_left_speed = abs(CAR_left_speed);
  abs_right_speed = abs(CAR_right_speed);

  if (CAR_left_speed >= 0)
  {
    #if IS_EGLOO_PLATFORM
      digitalWrite(left_direction_pin, HIGH);
      digitalWrite(left_direction_pin_in2, LOW);
    #else
      digitalWrite(left_direction_pin, HIGH);
    #endif
  }
  else
  {
    
    #if IS_EGLOO_PLATFORM
      digitalWrite(left_direction_pin, LOW);
      digitalWrite(left_direction_pin_in2, HIGH);
    #else
      digitalWrite(left_direction_pin, LOW);
    #endif
  }

  if (CAR_right_speed >= 0)
  {
    
    #if IS_EGLOO_PLATFORM
      digitalWrite(right_direction_pin, LOW);
      digitalWrite(right_direction_pin_in4, HIGH);
    #else
      digitalWrite(right_direction_pin, LOW);
    #endif
  }
  else
  {
    
    #if IS_EGLOO_PLATFORM
      digitalWrite(right_direction_pin, HIGH);
      digitalWrite(right_direction_pin_in4, LOW);
    #else
      digitalWrite(right_direction_pin, HIGH);
    #endif
  }
  analogWrite(left_speed_pin, abs_left_speed);
  analogWrite(right_speed_pin, abs_right_speed);
}