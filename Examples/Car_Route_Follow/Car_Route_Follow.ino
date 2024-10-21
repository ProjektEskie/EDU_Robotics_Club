
#include "Arduino_NineAxesMotion.h"        //Contains the bridge code between the API and the Arduino Environment
#include <Wire.h>
#include "route.h"

#define TELEMETRY_ENABLE 1
#define TELEMETRY_STREAM_INTERVAL 100

#define IS_EGLOO_PLATFORM 1

#define PIN_BATTERY_SENSE A0

#define IMU_ACCEL_THRESHOLD 0.05
#define IMU_VELOCITY_THRESHOLD 0.05

NineAxesMotion mySensor;           //Object that for the sensor
unsigned long sensor_update_interval_ms = 25;

uint8_t IMU_has_new_data;
float euler_heading, euler_roll, euler_pitch;
float linear_accel_x, linear_accel_y, linear_accel_z;
float prev_linear_accel_x, prev_linear_accel_y, prev_linear_accel_z;
float velocity_x, prev_velocity_x;
float distance_x, prev_distance_x;
float IMU_dt_s;
uint32_t acceleration_zero_time_ms = 400;
uint32_t velocity_zero_time_ms = 500;

char _str_buffer[128];

#if IS_EGLOO_PLATFORM
  const int left_direction_pin = 12;
  const int left_direction_pin_in2 = 8;
  const int right_direction_pin = 9;
  const int right_direction_pin_in4 = 11;
  const int left_speed_pin = 5;
  const int right_speed_pin = 6;
  const int servo_pin = 3;
#else
  const int left_direction_pin = 4;
  const int right_direction_pin = 3;
  const int left_speed_pin = 6;
  const int right_speed_pin = 5;
  const int servo_pin = 2;
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

  // Serial.println(route_n_waypoints);
  // for (uint8_t i= 0l; i < route_n_waypoints; i++ )
  // {

  //   Serial.println("Loaded the following waypoints:");
  //   sprintf(_str_buffer, "waypoint_no:%3i, distance:%6.3f, power:%3i, heading:%6.2f",
  //         i, route[i].distance_m, route[i].drive_power,
  //         route[i].heading_angle);
  //   Serial.println(_str_buffer);
  // }
}

void CAR_setup()
{
  pinMode(PIN_BATTERY_SENSE, INPUT);
  pinMode(left_direction_pin, OUTPUT);
  pinMode(right_direction_pin, OUTPUT);
  pinMode(left_speed_pin, OUTPUT);
  pinMode(right_speed_pin, OUTPUT);
  pinMode(servo_pin, OUTPUT);

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
  mySensor.setOperationMode(OPERATION_MODE_NDOF);   // 9 Degrees of freedom fusion mode
  euler_heading = 0;
  euler_roll = 0;
  euler_pitch = 0;
  velocity_x = 0;
  distance_x = 0;
  prev_velocity_x = 0;
  prev_distance_x = 0;
  IMU_has_new_data = 0;
}

uint32_t IMU_Loop()
{
  static uint32_t current_time;
  static uint32_t _last_update_time = 0;
  if ((millis() - _last_update_time) > sensor_update_interval_ms)
  {
    uint32_t _dt = millis() - _last_update_time;
    IMU_dt_s = _dt/ 1000.0;
    _last_update_time = millis();
    IMU_has_new_data = 1;

    euler_heading = mySensor.readEulerHeading();
    euler_roll = mySensor.readEulerRoll();
    euler_pitch = mySensor.readEulerPitch();

    prev_linear_accel_x = linear_accel_x;
    // mySensor.readAccelerometer(linear_accel_x, linear_accel_y, linear_accel_z);
    linear_accel_x = -1.0 * mySensor.readLinearAccelX();

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

// Numbeical integration with the trapzoid method
float helper_integrate(float current_accumator, float previous_val,
                      float current_val, float dt_s)
{
  current_accumator += ((previous_val + current_val) / 2.0) * dt_s;
  return current_accumator; 
}

// Apply thresholding to supress small values
float helper_threshold(float input, float low_thresh, float high_thresh)
{
  if ((input > low_thresh) && (input < high_thresh))
  {
    return 0.0;
  }
  else
  {
    return input;
  }
}

void integrator(void)
{
  static uint32_t car_last_runnint_time = 0;
  static uint32_t time_now;
  time_now = millis();


  // Override integrators with information from the car motor

  // Stop integration if car is not powered
  if (!CAR_left_speed && !CAR_right_speed)
  {
    if ((time_now - car_last_runnint_time) > acceleration_zero_time_ms)
    {
      linear_accel_x = 0.0;
    }
    
    if ((time_now - car_last_runnint_time) > velocity_zero_time_ms)
    {
      velocity_x = 0.0;
    }
  }
  else
  {
    car_last_runnint_time = time_now;
  }


  linear_accel_x = helper_threshold(linear_accel_x, -IMU_ACCEL_THRESHOLD, IMU_ACCEL_THRESHOLD);

  prev_velocity_x = velocity_x;
  velocity_x = helper_integrate(velocity_x, prev_linear_accel_x, linear_accel_x, IMU_dt_s);
  // velocity_x = helper_threshold(velocity_x, -IMU_VELOCITY_THRESHOLD, IMU_VELOCITY_THRESHOLD);
  prev_distance_x = distance_x;
  distance_x = helper_integrate(distance_x, prev_velocity_x, velocity_x, IMU_dt_s);
}

void loop() //This code is looped forever
{
  static uint32_t current_time;
  static uint32_t _last_telemetry_time = 0;

  uint32_t last_euler_update_time;

  float angle_diff = 0;
  float slew_rate = 0;

  static const uint32_t car_on_time = millis();
  static uint8_t _has_car_run = 0;

  current_time = millis();

  // Reading in the sensor inputs
  last_euler_update_time = IMU_Loop();

  if (IMU_has_new_data)
  {
    IMU_has_new_data = 0;
    integrator();
  }

  if (!_has_car_run)
  {
    if ((current_time - car_on_time) > 500)
    {
      CAR_left_speed = 0;
      CAR_right_speed = 0;
      _has_car_run = 1;
    }
    else
    {
      CAR_left_speed = 180;
      CAR_right_speed = 180;
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

      // sprintf(_str_buffer, "target:%6.3f, current_heading:%6.3f, slew_rate:%6.3f, left_speed:%4i, right_speed:%4i",
      //       target_bearing, euler_heading, slew_rate,
      //       CAR_left_speed, CAR_right_speed);

      sprintf(_str_buffer, "t:%6i, current_heading:%6.3f, xposition:%6.3f, xvelocity:%6.3f, xaccel:%6.3f",
            current_time, euler_heading,
            distance_x, velocity_x, linear_accel_x);
      Serial.println(_str_buffer);
    }

  }
}