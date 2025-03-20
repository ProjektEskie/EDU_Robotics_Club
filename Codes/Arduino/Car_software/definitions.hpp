#ifndef DEFINITIONS_H
#define DEFINITIONS_H

#include <arduino.h>
#include "car.hpp"
#include "IMU.hpp"
#include "BLE_Comm.hpp"
#include "personal_config.hpp"

#define JSON_BUFFER_LEN 480
#define BLE_IO_SERVICE_BUFFER_LEN 240
#define CMD_INPUT_BUFFER_LEN BLE_IO_SERVICE_BUFFER_LEN
#define CMD_OUTPUT_BUFFER_LEN BLE_IO_SERVICE_BUFFER_LEN
#define TELEMETRY_UPDATE_INTERNVAL 500
#define BLE_OUTPUT_REFRESH_INTERVAL 50

#define DEFAULT_BLE_CAR_NAME "RClub_Car"

#define EEPROM_START_BYTE 0b10101010
#define EEPROM_ADDRESS 10

enum _op_status_bits
{
  IS_WIFI_CONNECTED = 0,
  IS_MOVING,
};

typedef struct _operation_data
{
  unsigned int status;
  uint32_t time_now;
  uint32_t last_telemetry_time;
  uint32_t time_since_last_telemetry;
  uint8_t has_new_telemetry;
  uint32_t n_cycles_since_last_telemetry;

  imu_data imu;
  car_data car;
  ble_data ble;

} operation_data;

#endif