#ifndef BLE_COMM_H
#define BLE_COMM_H

#include <ArduinoBLE.h>
#include "personal_config.hpp"

typedef struct _ble_data
{
  int rssi;
  uint8_t is_connected;
  char car_name[20] = BLE_DEFAULT_CAR_NAME;
} ble_data;

void BLE_Comm_init();
void BLE_Comm_update();

#endif