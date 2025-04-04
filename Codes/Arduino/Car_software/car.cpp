#include "car.hpp"
#include "definitions.hpp"
#include "helpers.hpp"

bool CAR_turn_to_heading(float target_heading);
void CAR_stop();
void CAR_commit_speed();
void CAR_auto_mode();

extern operation_data op_data;

void CAR_init()
{
  pinMode(PIN_BATTERY_SENSE, INPUT);
  pinMode(LEFT_DIRECTION_PIN, OUTPUT);
  pinMode(RIGHT_DIRECTION_PIN, OUTPUT);
  pinMode(LEFT_SPEED_PIN, OUTPUT);
  pinMode(RIGHT_SPEED_PIN, OUTPUT);

  //Echo sensor
  pinMode(ECHO_TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_RESULT_PIN, INPUT);
  digitalWrite(ECHO_TRIGGER_PIN, LOW);

  #if IS_EGLOO_PLATFORM
  pinMode(LEFT_DIRECTION_PIN_IN2, OUTPUT);
  pinMode(RIGHT_DIRECTION_PIN_IN4, OUTPUT);
  #endif

  op_data.car.mode = CAR_MODE_IDLE;
  op_data.car.servo_angle = 0;
  op_data.car.servo_angle_offset = 0;

  op_data.car.front_servo.attach(SERVO_PIN);
  op_data.car.front_servo.write( -1 * op_data.car.servo_angle + op_data.car.servo_angle_offset + 90);
}

void CAR_update()
{
  CAR_servo_update();

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
  else if (op_data.car.mode == CAR_MODE_AUTO)
  {
    CAR_auto_mode();
  }
  else
  {
  }

  CAR_commit_speed();
}

void CAR_auto_mode()
{

  if (op_data.car.is_new_mode)
  {
    op_data.car.is_new_mode = false;
    op_data.car.am_data.step = CAR_AUTO_INIT;
  }

  if (op_data.sync.pulse_100ms)
  {
    op_data.car.am_data.range_infront = CAR_echo_range_cm();
  }
  
  if (op_data.car.am_data.range_infront < 0)
  {
    op_data.car.am_data.range_infront = 255;
  }

  op_data.car.am_data.range_infront = constrain(op_data.car.am_data.range_infront, 0, 255);


  if (op_data.car.am_data.step == CAR_AUTO_INIT)
  {
    op_data.car.am_data.step = CAR_AUTO_GOTO_HEADING;
    op_data.car.am_data.reverse_speed = -250;
    op_data.car.am_data.reverse_duration = 100;
    op_data.car.am_data.starting_heading = op_data.imu.euler_heading;
    op_data.car.am_data._delay_start_time = 0;
    op_data.car.am_data._delay_duration = 0;
    op_data.car.am_data.post_delay_step = CAR_AUTO_DONE;

    // Convert the target heading (-180 to 180 delta from current) to absolute heading (0 to 360)
    float absolute_heading = op_data.car.am_data.starting_heading + op_data.car.am_data.target_heading_delta;
    if (absolute_heading > 360.0)
    {
      absolute_heading -= 360.0;
    }
    else if (absolute_heading < 0.0)
    {
      absolute_heading += 360.0;
    }
    op_data.car.am_data.target_heading_absuolute = absolute_heading;
    helper_queue_formatted_message("Auto mode: ready to turn to %f", op_data.car.am_data.target_heading_absuolute);
  }
  else if (op_data.car.am_data.step == CAR_AUTO_GOTO_HEADING)
  {
    if (CAR_turn_to_heading(op_data.car.am_data.target_heading_absuolute))
    {
      op_data.car.am_data.step = CAR_AUTO_LINEAR_TRAVEL;
      op_data.car.am_data._forward_start_time = op_data.time_now;
      op_data.car.left_speed = op_data.car.am_data.forward_speed;
      op_data.car.right_speed = op_data.car.am_data.forward_speed;
      helper_queue_formatted_message("Auto mode: heading reached, moving forward at %i for %i ms",
        op_data.car.am_data.forward_speed,
        op_data.car.am_data.forward_duration);
    }
  }
  else if (op_data.car.am_data.step == CAR_AUTO_LINEAR_TRAVEL)
  {
    if ((op_data.time_now - op_data.car.am_data._forward_start_time) > op_data.car.am_data.forward_duration)
    {
      op_data.car.am_data.step = CAR_AUTO_DONE;
      helper_queue_formatted_message("Auto mode: forward travel complete as normal, stopping");
    }
    else if (op_data.car.am_data.range_infront < 20)
    {
      op_data.car.am_data._delay_start_time = op_data.time_now;
      op_data.car.am_data._delay_duration = 200;
      CAR_stop();
      op_data.car.am_data.post_delay_step = CAR_AUTO_DONE;
      op_data.car.am_data.step = CAR_AUTO_DELAY_START;
      helper_queue_formatted_message("Auto mode: obstacle detected, stopping and delaying for %i ms",
        op_data.car.am_data._delay_duration);
    }
  }
  else if (op_data.car.am_data.step == CAR_ATUO_BRAKE_START)
  {
    op_data.car.left_speed = op_data.car.am_data.reverse_speed;
    op_data.car.right_speed = op_data.car.am_data.reverse_speed;
    op_data.car.am_data._reverse_start_time = op_data.time_now;
    op_data.car.am_data.step = CAR_AUTO_BRAKE_COMPLETE;
  }
  else if (op_data.car.am_data.step == CAR_AUTO_BRAKE_COMPLETE)
  {
    if ((op_data.time_now - op_data.car.am_data._reverse_start_time) > op_data.car.am_data.reverse_duration)
    {
      op_data.car.am_data.step = CAR_AUTO_DONE;
    }
  }
  else if (op_data.car.am_data.step == CAR_AUTO_DELAY_START)
  {
    if ((op_data.time_now - op_data.car.am_data._delay_start_time) > op_data.car.am_data._delay_duration)
    {
      op_data.car.am_data.step = op_data.car.am_data.post_delay_step;
    }
  }
  else if (op_data.car.am_data.step == CAR_AUTO_DONE)
  {
    CAR_stop();
    CAR_API_set_mode(CAR_MODE_IDLE);
    helper_queue_messages("Auto mode: done, returning to idle mode.");

  }
  else
  {}
  
}

void CAR_servo_update()
{
  static int _prev_servo_angle = 0;
  static uint32_t _servo_update_time = 0;

  int servo_output = -1 * op_data.car.servo_angle + op_data.car.servo_angle_offset + 90;
  if (servo_output != _prev_servo_angle)
  {
    _prev_servo_angle = servo_output;
    _servo_update_time = op_data.time_now;
    op_data.car.front_servo.write(servo_output);
  }
  
  if (op_data.car.is_ranging_requested)
  {

    if ((op_data.time_now - _servo_update_time) > op_data.car.rd.scan_dewell_time[op_data.car.ranging_data_index])
    {
      
      int range = CAR_echo_range_cm();
      CAR_API_set_Servo_angle(op_data.car.rd.scan_angle[op_data.car.ranging_data_index]);
      float range_f;
      if (range < 0)
      {
        range_f = -1.0;
      }
      else
      {
        range_f = (float)range / 100.0;
      }

      op_data.car.rd.distance[op_data.car.ranging_data_index] = range_f;
      op_data.car.ranging_data_index++;
      if (op_data.car.ranging_data_index >= RANGING_DATA_SIZE)
      {
        op_data.car.is_ranging_data_ready = true;
        op_data.car.is_ranging_requested = false;
        helper_queue_ranging_data(&op_data.car.rd);
        CAR_API_set_Servo_angle(0);
      }
    }
  }

}

int CAR_echo_range_cm()
{
  int range = -1;
  long duration;
  digitalWrite(ECHO_TRIGGER_PIN, HIGH);
  delayMicroseconds(12);
  digitalWrite(ECHO_TRIGGER_PIN, LOW);
  duration = pulseIn(ECHO_RESULT_PIN, HIGH, 10000);

  if (duration > 0)
  {
    range = (int)(((float)duration) * 34300.0 / 2.0 / 1000000.0);
  }
  return range;
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

void CAR_API_set_PNG_settings(float target_heading, int line_speed, uint32_t line_duration)
{
  op_data.car.png_data.target_heading = target_heading;
  op_data.car.png_data.straight_line_speed = line_speed;
  op_data.car.png_data.straight_line_duration = line_duration;
}

void CAR_API_set_Servo_angle(int angle)
{
  op_data.car.servo_angle = constrain(angle, -90, 90);
}

void CAR_API_start_ranging_scan()
{
  op_data.car.is_ranging_data_ready = false;
  op_data.car.ranging_data_index = 0;
  op_data.car.is_ranging_requested = true;
  CAR_API_set_Servo_angle(op_data.car.rd.scan_angle[op_data.car.ranging_data_index]);
}

void CAR_API_set_auto_settings(int forward_speed, float target_heading, uint32_t forward_duration)
{
  op_data.car.am_data.target_heading_delta = target_heading;
  op_data.car.am_data.forward_speed = forward_speed;
  op_data.car.am_data.forward_duration = forward_duration;
  op_data.car.is_new_mode = true;
  op_data.car.mode = CAR_MODE_AUTO;
  op_data.car.am_data.step = CAR_AUTO_INIT;
}

// Generate the speed the car needs to head turn to the target heading
// Commit_speed() still need to be called to move the car
// returns true when the car is facing the target heading
bool CAR_turn_to_heading(float target_heading)
{
  bool is_done = false;
  float _dff = helper_angle_diff(op_data.imu.euler_heading, target_heading);

  int turn_speed = 220;
  float abs_diff;
  abs_diff = abs(_dff);

  // if (abs_diff > 80)
  // {
  //   turn_speed = 255;
  // }
  // else if ((abs_diff > 25) and (abs_diff <= 80))
  // {
  //   turn_speed = 180;
  // }
  // else
  // {
  //   turn_speed = 110;
  // }

  turn_speed = (int)((abs_diff/180.0) * 165.0 + 90.0 - 1.0);

  int angle_tolerance = 3;

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