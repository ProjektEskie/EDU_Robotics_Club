#include "car.hpp"
#include "definitions.hpp"
#include "helpers.hpp"

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
      op_data.car.left_speed = 0;
      op_data.car.right_speed = 0;
  }
  else if (op_data.car.mode == CAR_MODE_MANUAL)
  {
    op_data.car.left_speed = op_data.car.mm_data.mm_left_speed;
    op_data.car.right_speed = op_data.car.mm_data.mm_right_speed;
    if ((op_data.time_now - op_data.car.mm_data._mm_start_time) > op_data.car.mm_data.mm_duration)
    {
      op_data.car.mode = CAR_MODE_IDLE;
      op_data.car.left_speed = 0;
      op_data.car.right_speed = 0;
      helper_queue_messages("Info: manual move complete");
    }
  }
  else
  {
  }

  CAR_commit_speed();
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