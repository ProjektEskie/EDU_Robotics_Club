
#include "Arduino_NineAxesMotion.h"        //Contains the bridge code between the API and the Arduino Environment
#include <Wire.h>

#define TELEMETRY_ENABLE 1
#define TELEMETRY_STREAM_INTERVAL 100

NineAxesMotion mySensor;           //Object that for the sensor
unsigned long sensor_update_interval_ms = 25;

float euler_heading, euler_roll, euler_pitch;

char _str_buffer[128];

void setup() //This code is executed once
{

  //Peripheral Initialization
  Serial.begin(115200);   
  Wire.begin();                    //Initialize I2C communication to the let the library communicate with the sensor.

  //Sensor Initialization
  IMU_Init();

}

void IMU_Init()
{
  mySensor.initSensor();          //The I2C Address can be changed here inside this function in the library
  mySensor.setOperationMode(OPERATION_MODE_NDOF);   // 9 Degrees of freedom fusion mode
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

  current_time = millis();

  // Reading in the sensor inputs
  last_euler_update_time = IMU_Loop();



  // For printing out diagnostic information
  if ((current_time - _last_telemetry_time) >= TELEMETRY_STREAM_INTERVAL)
  {
    _last_telemetry_time = current_time;

    if (TELEMETRY_ENABLE)
    {
<<<<<<< HEAD
      // sprintf(_str_buffer, "t:%8i, s_read:%8i, heading:%6.3f, roll:%6.3f, pitch:%6.3f",
      //      current_time, last_euler_update_time,
      //      euler_heading, euler_roll, euler_pitch);
      // Serial.println(_str_buffer);

      Serial.print(current_time);
      Serial.print(",");
      Serial.print(last_euler_update_time);
      Serial.print(",");
      Serial.print(euler_heading);
      Serial.print(",");
      Serial.print(euler_roll);
      Serial.print(",");
      Serial.println(euler_pitch);

=======
      float angle_diff = helper_angle_diff(euler_heading, 15);
      sprintf(_str_buffer, "t:%8i, s_read:%8i, heading:%6.3f, angle_diff:%6.3f",
            current_time, last_euler_update_time,
            euler_heading, angle_diff);

      // sprintf(_str_buffer, "t:%8i, s_read:%8i, heading:%6.3f, roll:%6.3f, pitch:%6.3f",
      //       current_time, last_euler_update_time,
      //       euler_heading, euler_roll, euler_pitch);
      Serial.println(_str_buffer);
>>>>>>> 462428deb4e3d0ae74209c76d5a907d598b7411e
    }

  }
}