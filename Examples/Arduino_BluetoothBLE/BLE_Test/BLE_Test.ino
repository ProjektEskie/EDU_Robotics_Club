/*
  LED

  This example creates a Bluetooth® Low Energy peripheral with service that contains a
  characteristic to control an LED.

  The circuit:
  - Arduino MKR WiFi 1010, Arduino Uno WiFi Rev2 board, Arduino Nano 33 IoT,
    Arduino Nano 33 BLE, or Arduino Nano 33 BLE Sense board.

  You can use a generic Bluetooth® Low Energy central app, like LightBlue (iOS and Android) or
  nRF Connect (Android), to interact with the services and characteristics
  created in this sketch.

  This example code is in the public domain.
*/

#include <ArduinoBLE.h>

#define BLE_NAME "Arduino LED"
BLEService ledService("19B10000-E8F2-537E-4F6C-D104768A1214"); // Bluetooth® Low Energy LED Service

// Bluetooth® Low Energy LED Switch Characteristic - custom 128-bit UUID, read and writable by central
BLEByteCharacteristic switchCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);

char _ble_char_buffer[512];
BLECharacteristic stringCharacteristic("7b0db1df-67ed-46ef-b091-b4472119ef6d", BLERead | BLENotify, sizeof(_ble_char_buffer), true);
char _input_char_buffer[256];

BLECharacteristic input_characteristic("99924646-b9d6-4a51-bda9-ef084d793abf", BLEWrite, sizeof(_input_char_buffer), true);
char _output_char_buffer[256];

BLECharacteristic output_characteristic("8cf10e3b-0e9c-4809-b94a-5217ed9d6902", BLERead | BLENotify, sizeof(_output_char_buffer), true);

// Add a descriptor
BLEDescriptor millisLabelDescriptor("5078b4d3-9eb6-43f6-b55b-861ac78f388b", "millis placeholder");

BLEDescriptor input_Descriptor("081db19d-d936-40c5-8cac-06d26f7e7a11", "Input commands");
BLEDescriptor output_Descriptor("69a085e4-6ae1-4bb8-9e35-9f51fa664f92", "Outgoing messages");

const int ledPin = LED_BUILTIN; // pin to use for the LED

void setup() {
  Serial.begin(115200);
  while (!Serial);

  // set LED pin to output mode
  pinMode(ledPin, OUTPUT);

  // begin initialization
  if (!BLE.begin()) {
    Serial.println("starting Bluetooth® Low Energy module failed!");

    while (1);
  }

  memset(_input_char_buffer, 0, sizeof(_input_char_buffer));
  memset(_output_char_buffer, 0, sizeof(_output_char_buffer));

  // set advertised local name and service UUID:
  BLE.setLocalName(BLE_NAME);
  BLE.setAdvertisedService(ledService);

  // add descriptor
  stringCharacteristic.addDescriptor(millisLabelDescriptor);
  input_characteristic.addDescriptor(input_Descriptor);
  output_characteristic.addDescriptor(output_Descriptor);

  // Add event handlers
  input_characteristic.setEventHandler(BLEWritten, callback_characteristic_written);

  // add the characteristic to the service
  ledService.addCharacteristic(switchCharacteristic);

  ledService.addCharacteristic(stringCharacteristic);
  ledService.addCharacteristic(input_characteristic);
  ledService.addCharacteristic(output_characteristic);



  // add service
  BLE.addService(ledService);

  // set the initial value for the characeristic:
  switchCharacteristic.writeValue(0);
  _str_characteristic_update();
  output_characteristic.writeValue(_output_char_buffer, strlen(_output_char_buffer));

  // start advertising
  BLE.advertise();

  Serial.println("BLE LED Peripheral");
}

void loop() {
  // listen for Bluetooth® Low Energy peripherals to connect:
  BLEDevice central = BLE.central();

  // if a central is connected to peripheral:
  if (central) {
    Serial.print("Connected to central: ");
    // print the central's MAC address:
    Serial.println(central.address());

    // while the central is still connected to peripheral:
    while (central.connected()) {
      // if the remote device wrote to the characteristic,
      // use the value to control the LED:
      if (switchCharacteristic.written()) {
        if (switchCharacteristic.value()) {   // any value other than 0
          Serial.println("LED on");
          digitalWrite(ledPin, HIGH);         // will turn the LED on
        } else {                              // a 0 value
          Serial.println(F("LED off"));
          digitalWrite(ledPin, LOW);          // will turn the LED off
        }
      }

      // if (input_characteristic.written())
      // {
      //   Serial.print("Input Buffer Written:");
      //   sprintf(_input_char_buffer, "'%s'", input_characteristic.value());
      //   Serial.println(_input_char_buffer);
      //   Serial.println(input_characteristic.valueLength());
      //   memset(_input_char_buffer, 0, sizeof(_input_char_buffer));
      // }

      _str_characteristic_update();
    }

    // when the central disconnects, print it out:
    Serial.print(F("Disconnected from central: "));
    Serial.println(central.address());
  }
}

void callback_characteristic_written(BLEDevice central, BLECharacteristic characteristic)
{
  Serial.println(characteristic.uuid());
  sprintf(_input_char_buffer, "'%s'", input_characteristic.value());
  Serial.println(_input_char_buffer);
  Serial.println(input_characteristic.valueLength());
  memset(_input_char_buffer, 0, sizeof(_input_char_buffer));
}

void _str_characteristic_update()
{
  static uint32_t last_write_time = millis();
  if ((millis() - last_write_time) > 200)
  {
    memset(_ble_char_buffer, 0, sizeof(_ble_char_buffer));
    sprintf(_ble_char_buffer, "Arduino time: '%li'", millis());
    stringCharacteristic.writeValue(_ble_char_buffer, strlen(_ble_char_buffer));
    last_write_time = millis();
  }
}
