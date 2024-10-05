
#include "WiFiS3.h"
#include "arduino_secrets.h" 
#include <CmdParser.hpp>
#include <ArduinoJson.h>
#include "Arduino_NineAxesMotion.h"  

#define SERVER_PORT_NUMBER 8765
#define CMD_INPUT_BUFFER_LEN 256
#define CMD_OUTPUT_BUFFER_LEN 256
#define JSON_BUFFER_LEN 1024

#define ARDUINOJSON_SLOT_ID_SIZE 1
#define ARDUINOJSON_STRING_LENGTH_SIZE 1
#define ARDUINOJSON_USE_DOUBLE 0
#define ARDUINOJSON_USE_LONG_LONG 0

typedef struct _imu_data
{
  NineAxesMotion mySensor; 
  uint32_t last_update_time;
  uint32_t update_interval;
  uint16_t n_updates_counter;
  int system_calibration_status;
  float euler_heading;
  float euler_pitch;
  float euler_roll;
} imu_data;

typedef struct _car_data
{
  int left_speed;
  int right_speed;
} car_data;

typedef struct _operation_data
{
  int status;
  uint32_t time_now;
  uint32_t time_since_last_telemetry;

  imu_data imu;
  car_data car;

} operation_data;

operation_data op_data;


///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;                 // your network key index number (needed only for WEP)

int status = WL_IDLE_STATUS;

WiFiServer server(SERVER_PORT_NUMBER);

char _input_buffer[CMD_INPUT_BUFFER_LEN];
int _input_buffer_cur_idx = 0;
char _output_buffer[CMD_OUTPUT_BUFFER_LEN];
char _json_buffer[JSON_BUFFER_LEN];

CmdParser cmdParser;

void setup() {
  //Initialize serial and wait for port to open:
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  delay(1000);

  Wire.begin();
  Wire.setClock(400000);
  delay(1);

  // Start the IMU
  IMU_Init();

  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

  Serial.println("Attempting WIFI connection...");
  // attempt to connect to WiFi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    for (int i=0; i<10; i++)
    {
      if (WiFi.status() == WL_CONNECTED)
      {
        break;
      }

      delay(1000);
    }
  }


  // you're connected now, so print out the status:
  printWifiStatus();

  // start the server
  server.begin();

  // reset the buffers
  helper_clear_input_buffer();
  helper_clear_output_buffer();

  // Initalized some numbers
  op_data.time_since_last_telemetry = 0;
}


void loop() {

  op_data.time_now = millis();
  IMU_update();


  networking_tasks();

}

void networking_tasks()
{
  // listen for incoming clients
  WiFiClient client = server.available();
  static uint8_t _is_client_connected = 0;
  if (client) {

    if (client.connected()) {
      if (_is_client_connected == 0)
      {
        // Serial.println("new client");
      }
      
      _is_client_connected = 1;
      while (client.available()) {
        char c = client.read();
        _input_buffer[_input_buffer_cur_idx] = c;
        _input_buffer_cur_idx++;

        if (c == '\n') {
          // end of input command
          _input_buffer[_input_buffer_cur_idx] = 0; // cap off the string
          if (_input_buffer_cur_idx > 1)
          {
            _input_buffer[_input_buffer_cur_idx-1] = 0; // remove new-line character
          }
            
          cmd_parse();
          helper_clear_input_buffer();
          client.println(_json_buffer);
          client.flush();
          break;
        }
      }
    }
  }
  else
  {
    if (_is_client_connected)
    {
      // Serial.println("Client disconnected");
      helper_clear_input_buffer();
    }
    _is_client_connected = 0;
  }
}

void cmd_parse()
{
  // Serial.print("cmd_parse: parsing: ");
  // Serial.println(_input_buffer);

    if (cmdParser.parseCmd(_input_buffer) != CMDPARSER_ERROR) {
      // Serial.print("Command: ");
      // Serial.println(cmdParser.getCommand());

      // Serial.print("Size of parameter: ");
      // Serial.println(cmdParser.getParamCount());

      // Serial.print("Param 1: ");
      // Serial.println(cmdParser.getCmdParam(1));
    }
    else
    {
      Serial.println("Invalid command");
      helper_clear_output_buffer();
      return;
    }

    if (strcmp("Hello", cmdParser.getCommand()) == 0)
    {
      callback_funct_hello();
    }
    else if (strcmp("query", cmdParser.getCommand()) == 0)
    {
      helper_clear_output_buffer();
    }
    else
    {
      // Command not found
      helper_clear_output_buffer();
      sprintf(_output_buffer, "Error, command not found: %s", cmdParser.getCommand());
      Serial.println(_output_buffer);
    }

    telemetry_generate();

}

void callback_funct_hello()
{
  helper_clear_output_buffer();
  sprintf(_output_buffer, "Hello no. %s", cmdParser.getCmdParam(1));
  // Serial.println(_output_buffer);
}

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
}

void IMU_update()
{
  if ((op_data.time_now - op_data.imu.last_update_time) > op_data.imu.update_interval)
  {
    op_data.imu.last_update_time = op_data.time_now;
    op_data.imu.n_updates_counter++;

    op_data.imu.mySensor.updateEuler();        //Update the Euler data into the structure of the object
    op_data.imu.mySensor.updateCalibStatus();  //Update the Calibration Status

    op_data.imu.euler_heading = op_data.imu.mySensor.readEulerHeading();
    op_data.imu.euler_roll = op_data.imu.mySensor.readEulerRoll();
    op_data.imu.euler_pitch = op_data.imu.mySensor.readEulerPitch();

    op_data.imu.system_calibration_status = op_data.imu.mySensor.readSystemCalibStatus();
  }
}

void IMU_reset_n_updates_counter()
{
  op_data.imu.n_updates_counter = 0;
}

void telemetry_generate()
{
  static uint32_t _last_telemetry_time = 0;

  op_data.time_since_last_telemetry = op_data.time_now - _last_telemetry_time;
  _last_telemetry_time = op_data.time_now;

  JsonDocument doc;

  doc["status"] = op_data.status;
  doc["message"] = _output_buffer;
  doc["time_ms"] = op_data.time_now;
  doc["t_last"] = op_data.time_since_last_telemetry;

  JsonObject IMU = doc["IMU"].to<JsonObject>();
  IMU["last_updated"] = op_data.imu.last_update_time;
  IMU["n_updates"] = op_data.imu.n_updates_counter;
  IMU["system_cal"] = op_data.imu.system_calibration_status;

  JsonObject IMU_euler_angles = IMU["euler_angles"].to<JsonObject>();
  IMU_euler_angles["heading"] = op_data.imu.euler_heading;
  IMU_euler_angles["pitch"] = op_data.imu.euler_pitch;
  IMU_euler_angles["roll"] = op_data.imu.euler_roll;

  JsonObject CAR_speeds = doc["CAR"]["speeds"].to<JsonObject>();
  CAR_speeds["left"] = op_data.car.left_speed;
  CAR_speeds["right"] = op_data.car.right_speed;

  memset(_json_buffer, 0, JSON_BUFFER_LEN);

  doc.shrinkToFit();  // optional

  serializeJson(doc, _json_buffer);

  IMU_reset_n_updates_counter();

}

void helper_clear_input_buffer()
{
  memset(_input_buffer, 0, CMD_INPUT_BUFFER_LEN);
  _input_buffer_cur_idx = 0;
}

void helper_clear_output_buffer()
{
  memset(_output_buffer, 0, CMD_OUTPUT_BUFFER_LEN);
}

void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}
