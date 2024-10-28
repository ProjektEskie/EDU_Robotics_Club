#ifndef BLE_COMM_H
#define BLE_COMM_H

#include <ArduinoBLE.h>

typedef struct _ble_data
{
  int rssi;
  uint8_t is_connected;
} ble_data;

void BLE_Comm_init();
void BLE_Comm_update();

#endif