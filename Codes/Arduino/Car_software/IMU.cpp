#include "IMU.hpp"
#include "definitions.hpp"

extern operation_data op_data;

void IMU_Init()
{
  op_data.imu.mySensor.initSensor();          //The I2C Address can be changed here inside this function in the library

  IMU_reset_n_updates_counter();

  op_data.imu.mySensor.setOperationMode(OPERATION_MODE_IMUPLUS);   // IMU Mode (no magnetometer)
  op_data.imu.mySensor.setUpdateMode(MANUAL);	
  op_data.imu.mySensor.disableAnyMotion();


  op_data.imu.euler_heading = 0;
  op_data.imu.euler_pitch = 0;
  op_data.imu.euler_roll = 0;

  op_data.imu.system_calibration_status = 0;
  op_data.imu.gryo_calibration_status = 0;
  op_data.imu.mag_calibration_status = 0;
  op_data.imu.accel_calibration_status = 0;

  op_data.imu.linaccel_x = 0;
  op_data.imu.linaccel_y = 0;
  op_data.imu.linaccel_z = 0;
}

void IMU_update()
{
  if (op_data.sync.pulse_10ms)
  {
    op_data.imu.n_updates_counter++;

    op_data.imu.mySensor.updateGyro();          //Update the Gyro data into the structure of the object
    op_data.imu.gyro_z = op_data.imu.mySensor.readGyroZ() * -1.0;  //Read the Gyro Z value and invert it
    // Serial.print("IMU - Gyro Z: \t");
    // Serial.println(op_data.imu.gyro_z);
  }

  if (op_data.sync.pulse_100ms)
  {
    op_data.imu.mySensor.updateAccel();
    op_data.imu.mySensor.updateLinearAccel();

    if ( ALTERNATIVE_IMU_ORIENTATION ) {
      op_data.imu.linaccel_x = op_data.imu.mySensor.readLinearAccelY() * -1.0;
      op_data.imu.linaccel_y = op_data.imu.mySensor.readLinearAccelX();
    }
    else {
      op_data.imu.linaccel_x = op_data.imu.mySensor.readLinearAccelX() * -1.0;
      op_data.imu.linaccel_y = op_data.imu.mySensor.readLinearAccelY();
    }
    op_data.imu.linaccel_z = op_data.imu.mySensor.readLinearAccelZ();

    op_data.imu.mySensor.updateEuler();        //Update the Euler data into the structure of the object
    op_data.imu.euler_heading = op_data.imu.mySensor.readEulerHeading();
    op_data.imu.euler_roll = op_data.imu.mySensor.readEulerRoll();
    op_data.imu.euler_pitch = op_data.imu.mySensor.readEulerPitch() * -1.0;
  }

  if (op_data.sync.pulse_1000ms)
  {
    op_data.imu.mySensor.updateCalibStatus();  //Update the Calibration Status
    op_data.imu.mySensor.updateMag();

    op_data.imu.system_calibration_status = op_data.imu.mySensor.readSystemCalibStatus();
    op_data.imu.accel_calibration_status = op_data.imu.mySensor.readAccelCalibStatus();
    op_data.imu.gryo_calibration_status = op_data.imu.mySensor.readGyroCalibStatus();
    op_data.imu.mag_calibration_status = op_data.imu.mySensor.readMagCalibStatus();
  }
}

void IMU_reset_n_updates_counter()
{
  op_data.imu.n_updates_counter = 0;
}