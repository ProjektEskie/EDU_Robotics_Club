#ifndef BLE_COMM_H
#define BLE_COMM_H

#include <ArduinoBLE.h>
#include "personal_config.hpp"
#include "tracker.hpp"

#define BLE_N_TRACKER_POINTS_PER_TELEMETRY 8


typedef struct _ble_telemetry_avail_data
{
  uint8_t telemetry_avail_flag;
  uint8_t n_tracker_points;
  // uint8_t spare_bytes[2];
  uint32_t sys_time;
  int16_t left_speed;
  int16_t right_speed;
  int16_t heading;  // in 0.1 degrees
  uint8_t spare_bytes[16];
  // The tracker data is an array of track_point structures
  // uint32_t seperator = 0xACCE55ED; // This is used to separate the telemetry data from the tracker data
  track_point tracker_data[BLE_N_TRACKER_POINTS_PER_TELEMETRY];
} __attribute__((packed)) ble_telemetry_avail_data; 

typedef struct _ble_data
{
  int rssi;
  uint8_t is_connected;
  char car_name[20] = BLE_DEFAULT_CAR_NAME;
} ble_data;

void BLE_Comm_init();
void BLE_Comm_update();

#endif