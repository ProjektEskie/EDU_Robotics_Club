/****************************************************************************
* Copyright (C) 2011 - 2014 Bosch Sensortec GmbH
*
* Accelerometer.ino
* Date: 2014/09/09
* Revision: 3.0 $
*
* Usage:        Example code to stream Accelerometer data
*
****************************************************************************
/***************************************************************************
* License:
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*   Redistributions of source code must retain the above copyright
*   notice, this list of conditions and the following disclaimer.
*
*   Redistributions in binary form must reproduce the above copyright
*   notice, this list of conditions and the following disclaimer in the
*   documentation and/or other materials provided with the distribution.
*
*   Neither the name of the copyright holder nor the names of the
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
*
* The information provided is believed to be accurate and reliable.
* The copyright holder assumes no responsibility for the consequences of use
* of such information nor for any infringement of patents or
* other rights of third parties which may result from its use.
* No license is granted by implication or otherwise under any patent or
* patent rights of the copyright holder.
*/

#include "Arduino_NineAxesMotion.h"        //Contains the bridge code between the API and the Arduino Environment
#include <Wire.h>

#define ACCEL_LOW_THRESHOLD -0.08
#define ACCEL_HIGH_THRESHOLD 0.08

NineAxesMotion mySensor;                 //Object that for the sensor
unsigned long lastStreamTime = 0;     //To store the last streamed time stamp
const int streamPeriod = 40;          //To stream at 25Hz without using additional timers (time period(ms) =1000/frequency(Hz))
bool updateSensorData = true;         //Flag to update the sensor data. Default is true to perform the first read before the first stream

float _prev_x_acceleration = 0;
float current_x_accel = 0;
unsigned long last_read_time = 0;
float current_velocity_x = 0;

void setup() //This code is executed once
{
  //Peripheral Initialization
  Serial.begin(9600);           //Initialize the Serial Port to view information on the Serial Monitor
  Wire.begin();                    //Initialize I2C communication to the let the library communicate with the sensor.
  //Sensor Initialization
  mySensor.initSensor();          //The I2C Address can be changed here inside this function in the library
  mySensor.setOperationMode(OPERATION_MODE_NDOF);   //Can be configured to other operation modes as desired
  mySensor.setUpdateMode(MANUAL);	//The default is AUTO. Changing to manual requires calling the relevant update functions prior to calling the read functions
  //Setting to MANUAL requires lesser reads to the sensor
  mySensor.updateAccelConfig();
  updateSensorData = true;
  Serial.println();
  Serial.println("Default accelerometer configuration settings...");
  Serial.print("Range: ");
  Serial.println(mySensor.readAccelRange());
  Serial.print("Bandwidth: ");
  Serial.println(mySensor.readAccelBandwidth());
  Serial.print("Power Mode: ");
  Serial.println(mySensor.readAccelPowerMode());
  Serial.println("Streaming in ...");	//Countdown
  Serial.print("3...");
  delay(1000);	//Wait for a second
  Serial.print("2...");
  delay(1000);	//Wait for a second
  Serial.println("1...");
  delay(1000);	//Wait for a second
}

float accel_to_vel(float current_vel, float previous_accel,
                    float current_accel, float dt_s)
{
  current_vel += ((previous_accel + current_accel) / 2.0) * dt_s;
  return current_vel; 
}

float threshold(float input, float low_thresh, float high_thresh)
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

void loop() //This code is looped forever
{

  uint8_t accel_cal_status = 0;

  if (updateSensorData)  //Keep the updating of data as a separate task
  {
    mySensor.updateAccel();        //Update the Accelerometer data
    mySensor.updateLinearAccel();  //Update the Linear Acceleration data
    mySensor.updateGravAccel();    //Update the Gravity Acceleration data
    mySensor.updateCalibStatus();  //Update the Calibration Status
    updateSensorData = false;
  }
  if ((millis() - lastStreamTime) >= streamPeriod)
  {
    last_read_time = lastStreamTime;
    lastStreamTime = millis();

    unsigned long time_elapsed = lastStreamTime - last_read_time;
    float dt_s = time_elapsed/ 1000.0;

    _prev_x_acceleration = current_x_accel;
    current_x_accel = mySensor.readLinearAcceleration(X_AXIS);
    current_x_accel = threshold(current_x_accel, ACCEL_LOW_THRESHOLD, ACCEL_HIGH_THRESHOLD);

    accel_cal_status = mySensor.readAccelCalibStatus();    

    current_velocity_x = accel_to_vel(current_velocity_x, _prev_x_acceleration,
        current_x_accel, dt_s);

    Serial.print(current_x_accel); //Linear Acceleration X-Axis data
    Serial.print(",");
    Serial.print(current_velocity_x); //X-Axis velocity by integration
    Serial.print(",");
    Serial.print(accel_cal_status); //Accelerometer calibration status
    Serial.println();

    updateSensorData = true;
  }
}
