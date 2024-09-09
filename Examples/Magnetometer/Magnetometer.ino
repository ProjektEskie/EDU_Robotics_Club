
#include "Arduino_NineAxesMotion.h"        //Contains the bridge code between the API and the Arduino Environment
#include <Wire.h>

#define TELEMETRY_ENABLE 1

NineAxesMotion mySensor;           //Object that for the sensor
unsigned long lastStreamTime = 0;     //To store the last streamed time stamp
const int streamPeriod = 10;          
bool updateSensorData = true;         //Flag to update the sensor data. Default is true to perform the first read before the first stream

char _str_buffer[128];

float mag_x = 0;
float mag_y = 0;
float mag_z = 0;

float mag_magnitude = 0;

void setup() //This code is executed once
{
  // IO COnfig
  analogWriteResolution(12);

  //Peripheral Initialization
  Serial.begin(115200);   
  Wire.begin();                    //Initialize I2C communication to the let the library communicate with the sensor.
  //Sensor Initialization
  mySensor.initSensor();          //The I2C Address can be changed here inside this function in the library
  mySensor.setOperationMode(OPERATION_MODE_M4G);   // Magnetomoter only
  // mySensor.setUpdateMode(MANUAL);

}

void loop() //This code is looped forever
{
  static int analog_out = 0;
  static uint32_t current_time;

  current_time = millis();

  if (updateSensorData)  //Keep the updating of data as a separate task
  {
    mySensor.updateMag();        //Update the Accelerometer data
    updateSensorData = false;
  }

  if ((current_time - lastStreamTime) >= streamPeriod)
  {
    lastStreamTime = current_time;
    updateSensorData = true;

    mySensor.readMagnetometer(mag_x, mag_y, mag_z);

    // mag_x = mag_x/2048.0;
    // mag_y = mag_y/2048.0;
    // mag_z = mag_z/2048.0;

    mag_magnitude = sqrt(mag_x*mag_x + mag_y*mag_y + mag_z*mag_z);

    mag_magnitude = mag_magnitude/3547.24; // normalize magnitude to 1

    analog_out = int(mag_magnitude*4096.0);

    analogWrite(A0, mag_magnitude*4096.0);

    if (TELEMETRY_ENABLE)
    {
      sprintf(_str_buffer, "t:%8i, x:%06.4f, y:%06.4f, z:%06.4f, mag:%06.6f, A0:%6i",
            current_time, mag_x, mag_y, mag_z, mag_magnitude, analog_out);
      Serial.println(_str_buffer);
    }

  }
}