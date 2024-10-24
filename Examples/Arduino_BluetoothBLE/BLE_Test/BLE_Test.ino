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

// Add a descriptor
BLEDescriptor millisLabelDescriptor("5078b4d3-9eb6-43f6-b55b-861ac78f388b", "millis");

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

  // set advertised local name and service UUID:
  BLE.setLocalName(BLE_NAME);
  BLE.setAdvertisedService(ledService);

  // add descriptor
  stringCharacteristic.addDescriptor(millisLabelDescriptor);

  // add the characteristic to the service
  ledService.addCharacteristic(switchCharacteristic);

  ledService.addCharacteristic(stringCharacteristic);



  // add service
  BLE.addService(ledService);

  // set the initial value for the characeristic:
  switchCharacteristic.writeValue(0);
  _str_characteristic_update();

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

      _str_characteristic_update();
    }

    // when the central disconnects, print it out:
    Serial.print(F("Disconnected from central: "));
    Serial.println(central.address());
  }
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
