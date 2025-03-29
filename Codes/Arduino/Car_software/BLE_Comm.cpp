
#include <Arduino.h>
#include <cppQueue.h> 
#include "definitions.hpp"

#include "BLE_Comm.hpp"
#include "helpers.hpp"

extern operation_data op_data;
extern char _input_buffer[BLE_IO_SERVICE_BUFFER_LEN];
extern char _output_buffer[BLE_IO_SERVICE_BUFFER_LEN];
extern char _json_buffer[JSON_BUFFER_LEN];
extern char _telemetry_notify_buffer[BLE_IO_SERVICE_BUFFER_LEN];
extern cppQueue _output_queue;
extern cppQueue tracker_queue;

BLEService car_service("19B10000-E8F2-537E-4F6C-D104768A1214");
BLECharacteristic telemetry_available_characteristic("19B10001-E8F2-537E-4F6C-D104768A1214", BLENotify, BLE_IO_SERVICE_BUFFER_LEN, true);
BLECharacteristic telemetry_characteristic("7b0db1df-67ed-46ef-b091-b4472119ef6d", BLERead, JSON_BUFFER_LEN, true);
BLECharacteristic input_characteristic("99924646-b9d6-4a51-bda9-ef084d793abf", BLEWrite, BLE_IO_SERVICE_BUFFER_LEN, true);
BLECharacteristic output_characteristic("8cf10e3b-0e9c-4809-b94a-5217ed9d6902", BLERead | BLENotify, BLE_IO_SERVICE_BUFFER_LEN, true);
BLEDescriptor telemetry_descriptor("5078b4d3-9eb6-43f6-b55b-861ac78f388b", "Telemetry Downlink");
BLEDescriptor input_descriptor("081db19d-d936-40c5-8cac-06d26f7e7a11", "Input commands");
BLEDescriptor output_descriptor("69a085e4-6ae1-4bb8-9e35-9f51fa664f92", "Outgoing messages");

void callback_characteristic_written(BLEDevice central, BLECharacteristic characteristic);

uint8_t _telemetry_avail_flag = 0;
int _hold_output_flag = 0;

void BLE_Comm_init()
{
  if (!BLE.begin()) {
    Serial.println("starting Bluetooth® Low Energy module failed!");

    while (1);
  }
  if (!helper_load_BLE_name())
  {
    Serial.println("Error loading car name from EEPROM, using default name.");
  }

  BLE.setLocalName(op_data.ble.car_name);
  BLE.setDeviceName(op_data.ble.car_name);
  

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
  Serial.println(op_data.ble.car_name);
}

void BLE_Comm_update()
{
  BLEDevice central = BLE.central();
  static uint32_t last_output_refresh_time = millis();

  op_data.ble.is_connected = 0;

  op_data.ble.rssi = BLE.rssi();

  if (central)
  {
    if (central.connected())
    {
      op_data.ble.is_connected = 1;

      if (op_data.has_new_telemetry)
      {

        ble_telemetry_avail_data _telemetry_avail_data;
        memset(&_telemetry_avail_data, 0, sizeof(ble_telemetry_avail_data));
        _telemetry_avail_data.telemetry_avail_flag = _telemetry_avail_flag;
        _telemetry_avail_data.sys_time = op_data.time_now;
        _telemetry_avail_data.left_speed = op_data.car.left_speed;
        _telemetry_avail_data.right_speed = op_data.car.right_speed;
        _telemetry_avail_data.heading = (int32_t)(op_data.imu.euler_heading * 10.0);

        uint8_t _tracker_count = 0;
        for (int i = 0; i < BLE_N_TRACKER_POINTS_PER_TELEMETRY; i++)
        {
          if (!tracker_queue.isEmpty())
          {
            tracker_queue.pop(&_telemetry_avail_data.tracker_data[_tracker_count]);
            _tracker_count++;
          }
          else
          {
            break;
          }
        }

        _telemetry_avail_data.n_tracker_points = _tracker_count;

        telemetry_characteristic.writeValue(_json_buffer, JSON_BUFFER_LEN, true);
        memset(_telemetry_notify_buffer, 0, BLE_IO_SERVICE_BUFFER_LEN);
        memcpy(_telemetry_notify_buffer, &_telemetry_avail_data, sizeof(ble_telemetry_avail_data));
        telemetry_available_characteristic.writeValue(_telemetry_notify_buffer, BLE_IO_SERVICE_BUFFER_LEN, true);
        _telemetry_avail_flag = !_telemetry_avail_flag;
        op_data.has_new_telemetry = 0;
        _hold_output_flag = 5; // Supress the automatic sending on the output channel for 5 cycles
      }

      if ((op_data.time_now - last_output_refresh_time) > BLE_OUTPUT_REFRESH_INTERVAL)
      {
        // Do not send output for 1 refresh interval if a telemetry has just been updated
        if (_hold_output_flag >= 0)
        {
          _hold_output_flag--;
        }
        else
        {
          if (!_output_queue.isEmpty())
          {
            helper_clear_output_buffer();
            _output_queue.pop(_output_buffer);
            output_characteristic.writeValue(_output_buffer, BLE_IO_SERVICE_BUFFER_LEN, true);
          }
        }
        last_output_refresh_time = op_data.time_now;
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