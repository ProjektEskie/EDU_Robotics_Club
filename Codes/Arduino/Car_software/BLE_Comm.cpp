
#include <Arduino.h>
#include <cppQueue.h> 
#include "definitions.hpp"

#include "BLE_Comm.hpp"
#include "helpers.hpp"

extern operation_data op_data;
extern char _input_buffer[BLE_IO_SERVICE_BUFFER_LEN];
extern char _output_buffer[BLE_IO_SERVICE_BUFFER_LEN];
extern char _json_buffer[JSON_BUFFER_LEN];
extern cppQueue _output_queue;

BLEService car_service("19B10000-E8F2-537E-4F6C-D104768A1214");
BLEByteCharacteristic telemetry_available_characteristic("19B10001-E8F2-537E-4F6C-D104768A1214", BLENotify);
BLECharacteristic telemetry_characteristic("7b0db1df-67ed-46ef-b091-b4472119ef6d", BLERead, JSON_BUFFER_LEN, true);
BLECharacteristic input_characteristic("99924646-b9d6-4a51-bda9-ef084d793abf", BLEWrite, BLE_IO_SERVICE_BUFFER_LEN, true);
BLECharacteristic output_characteristic("8cf10e3b-0e9c-4809-b94a-5217ed9d6902", BLERead | BLENotify, BLE_IO_SERVICE_BUFFER_LEN, true);
BLEDescriptor telemetry_descriptor("5078b4d3-9eb6-43f6-b55b-861ac78f388b", "Telemetry Downlink");
BLEDescriptor input_descriptor("081db19d-d936-40c5-8cac-06d26f7e7a11", "Input commands");
BLEDescriptor output_descriptor("69a085e4-6ae1-4bb8-9e35-9f51fa664f92", "Outgoing messages");

void callback_characteristic_written(BLEDevice central, BLECharacteristic characteristic);

uint8_t _telemetry_avail_flag = 0;

void BLE_Comm_init()
{
  if (!BLE.begin()) {
    Serial.println("starting Bluetooth® Low Energy module failed!");

    while (1);
  }

  BLE.setLocalName(BLE_CAR_NAME);
  

  telemetry_characteristic.addDescriptor(telemetry_descriptor);
  input_characteristic.addDescriptor(input_descriptor);
  output_characteristic.addDescriptor(output_descriptor);

  input_characteristic.setEventHandler(BLEWritten, callback_characteristic_written);

  memset(_json_buffer, 0, JSON_BUFFER_LEN);
  telemetry_characteristic.writeValue(_json_buffer, JSON_BUFFER_LEN, true);

  car_service.addCharacteristic(telemetry_available_characteristic);
  car_service.addCharacteristic(telemetry_characteristic);
  car_service.addCharacteristic(input_characteristic);
  car_service.addCharacteristic(output_characteristic);

  BLE.addService(car_service);

  BLE.setAdvertisedService(car_service);
  BLE.advertise();

  Serial.print("Bluetooth running as:");
  Serial.println(BLE_CAR_NAME);
}

void BLE_Comm_update()
{
  BLEDevice central = BLE.central();
  static uint32_t last_output_refresh_time = millis();

  if (central)
  {
    if (central.connected())
    {
      if (op_data.has_new_telemetry)
      {
        telemetry_characteristic.writeValue(_json_buffer, JSON_BUFFER_LEN, true);
        telemetry_available_characteristic.writeValue(_telemetry_avail_flag);
        _telemetry_avail_flag = !_telemetry_avail_flag;
        op_data.has_new_telemetry = 0;
      }

      if ((op_data.time_now - last_output_refresh_time) > BLE_OUTPUT_REFRESH_INTERVAL)
      {
        if (!_output_queue.isEmpty())
        {
          helper_clear_output_buffer();
          _output_queue.pop(_output_buffer);
          output_characteristic.writeValue(_output_buffer, BLE_IO_SERVICE_BUFFER_LEN, true);
        }

      }
    }
    else
    {
    }
  }


}

void callback_characteristic_written(BLEDevice central, BLECharacteristic characteristic)
{
  helper_clear_input_buffer();
  sprintf(_input_buffer, "%s", input_characteristic.value());
}