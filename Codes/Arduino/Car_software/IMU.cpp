#include "IMU.hpp"
#include "definitions.hpp"

extern operation_data op_data;

void IMU_Init()
{
  op_data.imu.mySensor.initSensor();          //The I2C Address can be changed here inside this function in the library

  op_data.imu.update_interval = 50;
  op_data.imu.last_update_time = 0;
  IMU_reset_n_updates_counter();

  op_data.imu.mySensor.setOperationMode(OPERATION_MODE_IMUPLUS);   // IMU Mode (no magnetometer)
  op_data.imu.mySensor.setUpdateMode(MANUAL);	

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
  if ((op_data.time_now - op_data.imu.last_update_time) > op_data.imu.update_interval)
  {
    op_data.imu.last_update_time = op_data.time_now;
    op_data.imu.n_updates_counter++;

    op_data.imu.mySensor.updateEuler();        //Update the Euler data into the structure of the object
    op_data.imu.mySensor.updateCalibStatus();  //Update the Calibration Status
    op_data.imu.mySensor.updateLinearAccel();
    // op_data.imu.mySensor.updateMag();

    op_data.imu.euler_heading = op_data.imu.mySensor.readEulerHeading();
    op_data.imu.euler_roll = op_data.imu.mySensor.readEulerRoll();
    op_data.imu.euler_pitch = op_data.imu.mySensor.readEulerPitch();

    op_data.imu.system_calibration_status = op_data.imu.mySensor.readSystemCalibStatus();
    op_data.imu.accel_calibration_status = op_data.imu.mySensor.readAccelCalibStatus();
    op_data.imu.gryo_calibration_status = op_data.imu.mySensor.readGyroCalibStatus();
    op_data.imu.mag_calibration_status = op_data.imu.mySensor.readMagCalibStatus();

    op_data.imu.mySensor.readLinearAcceleration(
      op_data.imu.linaccel_x,
      op_data.imu.linaccel_y,
      op_data.imu.linaccel_z);
  }
}

void IMU_reset_n_updates_counter()
{
  op_data.imu.n_updates_counter = 0;
}