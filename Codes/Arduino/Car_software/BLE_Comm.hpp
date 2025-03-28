#ifndef BLE_COMM_H
#define BLE_COMM_H

#include <ArduinoBLE.h>
#include "personal_config.hpp"
#include "tracker.hpp"

#define BLE_N_TRACKER_POINTS_PER_TELEMETRY 10


typedef struct _ble_telemetry_avail_data
{
  uint8_t telemetry_avail_flag;
  uint8_t n_tracker_points;
  uint8_t spare_bytes[2];
  uint32_t sys_time;
  int left_speed;
  int right_speed;
  uint8_t spare_bytes_2[4];
  track_point tracker_data[BLE_N_TRACKER_POINTS_PER_TELEMETRY];
} ble_telemetry_avail_data; 

typedef struct _ble_data
{
  int rssi;
  uint8_t is_connected;
  char car_name[20] = BLE_DEFAULT_CAR_NAME;
} ble_data;

void BLE_Comm_init();
void BLE_Comm_update();

#endif