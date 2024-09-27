
#include "Arduino_NineAxesMotion.h"        //Contains the bridge code between the API and the Arduino Environment
#include <Wire.h>

#define TELEMETRY_ENABLE 1
#define TELEMETRY_STREAM_INTERVAL 100

#define IS_EGLOO_PLATFORM 1

#define PIN_BATTERY_SENSE A0

NineAxesMotion mySensor;           //Object that for the sensor
unsigned long sensor_update_interval_ms = 50;

float euler_he  ading, euler_roll, euler_pitch;

char _str_buffer[128];

#if IS_EGLOO_PLATFORM
  const int left_direction_pin = 12;
  const int left_direction_pin_in2 = 8;
  const int right_direction_pin = 9;
  const int right_direction_pin_in4 = 11;
  const int left_speed_pin = 5;
  const int right_speed_pin = 6;
#else
  const int left_direction_pin = 4;
  const int right_direction_pin = 3;
  const int left_speed_pin = 6;
  const int right_speed_pin = 5;
#endif

float target_bearing = 0.0;

int CAR_left_speed = 0;
int CAR_right_speed = 0;

float CAR_battery_voltage = 0;

void setup() //This code is executed once
{

  //Peripheral Initialization
  Serial.begin(115200);   
  Wire.begin();                    //Initialize I2C communication to the let the library communicate with the sensor.

  //Sensor Initialization
  IMU_Init();

  //Car Initialization
  CAR_setup();

}

void CAR_setup()
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

  CAR_stop();
}

void CAR_update()
{

  CAR_commit_speed();
}

void CAR_stop()
{
  CAR_left_speed = 0;
  CAR_right_speed = 0;
  CAR_commit_speed();
}

void CAR_commit_speed()
{
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

void IMU_Init()
{
  mySensor.initSensor();          //The I2C Address can be changed here inside this function in the library
  mySensor.setOperationMode(OPERATION_MODE_IMUPLUS);   // IMU Mode (no magnetometer)
  euler_heading = 0;
  euler_roll = 0;
  euler_pitch = 0;
}

uint32_t IMU_Loop()
{
  static uint32_t current_time;
  static uint32_t _last_update_time = 0;
  if ((millis() - _last_update_time) > sensor_update_interval_ms)
  {
    _last_update_time = millis();

    euler_heading = mySensor.readEulerHeading();
    euler_roll = mySensor.readEulerRoll();
    euler_pitch = mySensor.readEulerPitch();
  }

  return _last_update_time;
}


// Calcualte the angle between two points on the compass
// Returns: difference in angles in degrees, positve is in the clockwise direction
// Rounded to 2 decimal places
float helper_angle_diff(float current_angle, float target_angle)
{
  int angle_diff = 0;

  angle_diff = int((current_angle - target_angle) * 100);
  angle_diff = (angle_diff + 18000 + 36000) % 36000 - 18000;

  return float(angle_diff/100.0);
}

void loop() //This code is looped forever
{
  static uint32_t current_time;
  static uint32_t _last_telemetry_time = 0;

  uint32_t last_euler_update_time;

  float angle_diff = 0;
  float slew_rate = 0;

  current_time = millis();

  // Reading in the sensor inputs
  last_euler_update_time = IMU_Loop();

  // How fast to turn the car 
  angle_diff = helper_angle_diff(euler_heading, target_bearing);
  slew_rate = angle_diff / 180.0 * (-1.0);

  // scale the slew rate to numbers in terms of absolute left and right speed
  // Min power is 180, max power is 250
  // Stop moving when abs(slew_rate) is less than 0.05
  if (abs(slew_rate) < 0.05)
  {
    CAR_left_speed = 0;
    CAR_right_speed = 0;
  }
  else
  {
    float fspeed = (250.0 - 180.0) * abs(slew_rate) + 180.0;
    int speed = int(fspeed);

    if (slew_rate >= 0)
    {
      CAR_left_speed = speed;
      CAR_right_speed = speed * -1;
    }
    else
    {
      CAR_left_speed = speed * -1;
      CAR_right_speed = speed;
    }
  }

  CAR_update();

  // For printing out diagnostic information
  if ((current_time - _last_telemetry_time) >= TELEMETRY_STREAM_INTERVAL)
  {
    _last_telemetry_time = current_time;

    if (TELEMETRY_ENABLE)
    {

      // sprintf(_str_buffer, "t:%8i, s_read:%8i, heading:%6.3f, roll:%6.3f, pitch:%6.3f",
      //       current_time, last_euler_update_time,
      //       euler_heading, euler_roll, euler_pitch);

      sprintf(_str_buffer, "target:%6.3f, current_heading:%6.3f, slew_rate:%6.3f, left_speed:%4i, right_speed:%4i",
            target_bearing, euler_heading, slew_rate,
            CAR_left_speed, CAR_right_speed);
      Serial.println(_str_buffer);
    }

  }
}