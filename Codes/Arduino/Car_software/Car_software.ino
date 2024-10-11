
#include "definitions.hpp"
#include "helpers.hpp"
#include "car.hpp"
#include "IMU.hpp"

#include "WiFiS3.h"
#include <CmdParser.hpp>
#include <ArduinoJson.h>
#include "ArduinoGraphics.h"
#include "Arduino_NineAxesMotion.h"
#include "Arduino_LED_Matrix.h"
#include <cppQueue.h>  

#define SECRET_SSID "Robotics_Club"
#define SECRET_PASS "beep boop"

#define SERVER_PORT_NUMBER 8765

#define JSON_BUFFER_LEN 1024

#define OUTPUT_MESSAGE_QUEUE_CAPACITY 10

#define ARDUINOJSON_SLOT_ID_SIZE 1
#define ARDUINOJSON_STRING_LENGTH_SIZE 1
#define ARDUINOJSON_USE_DOUBLE 0
#define ARDUINOJSON_USE_LONG_LONG 0

operation_data op_data;


///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;                 // your network key index number (needed only for WEP)

int status = WL_IDLE_STATUS;

WiFiServer server(SERVER_PORT_NUMBER);

char _json_buffer[JSON_BUFFER_LEN];

char _input_buffer[CMD_INPUT_BUFFER_LEN];
int _input_buffer_cur_idx = 0;
char _output_buffer[CMD_OUTPUT_BUFFER_LEN];

char _queue_buffer[OUTPUT_MESSAGE_QUEUE_CAPACITY][CMD_OUTPUT_BUFFER_LEN];
cppQueue _output_queue(CMD_OUTPUT_BUFFER_LEN, OUTPUT_MESSAGE_QUEUE_CAPACITY, FIFO, false, _queue_buffer, sizeof(_queue_buffer));

CmdParser cmdParser;

ArduinoLEDMatrix matrix;

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

  // Start the LED matrix
  matrix.begin();

  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

  Serial.println("Attempting WIFI connection...");
  // attempt to connect to WiFi network:
  if (status != WL_CONNECTED) {
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

  // Setup the car
  CAR_init(&op_data.car);


  // you're connected now, so print out the status:
  printWifiStatus();

  // display the last 3 digits of the IP address

  if (WiFi.status() == WL_CONNECTED)
  {
    matrix.beginDraw();
    matrix.stroke(0xFFFFFFFF);
    matrix.textFont(Font_5x7);
    matrix.beginText(-50, 2, 0xFFFFFF);
    matrix.println(WiFi.localIP());
    matrix.endText();

    matrix.endDraw();
  }

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

  if (WiFi.status() != WL_CONNECTED)
  {
    static bool matrix_frame_loaded = false;
    if (!matrix_frame_loaded)
    {
      matrix.loadFrame(LEDMATRIX_DANGER);
      matrix_frame_loaded = true;
    }
  }

  IMU_update();

  CAR_update(&op_data.car, op_data.time_now);

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
      if (!_output_queue.isEmpty())
      {
        helper_clear_output_buffer();
        _output_queue.pop(_output_buffer);
      }
    }
    else if (strcmp("car_m_move", cmdParser.getCommand()) == 0)
    {
      if (cmdParser.getParamCount() != 3)
      {
        helper_clear_output_buffer();
        sprintf(_output_buffer, "Error, '%s' command accepts exactly 3 arguements",
                cmdParser.getCommand());
        helper_queue_messages("Info: car_m_move (car, manual move) example usage:");
        helper_queue_messages("Info: car_m_move [left_speed] [right_speed] [duration_ms]");
        helper_queue_messages("Info: car_m_move 200 200 1500");
        helper_queue_messages("Info: car_m_move left_speed and right_speed should be between -255 to 255");
      }
      else
      {
        int left_speed = atoi(cmdParser.getCmdParam(1));
        int right_speed = atoi(cmdParser.getCmdParam(2));
        uint32_t duration = atol(cmdParser.getCmdParam(3));
        callback_car_m_move(left_speed,
                            right_speed,
                            duration);
        helper_clear_output_buffer();
        sprintf(_output_buffer, "Success, '%s'. Moving.",
                cmdParser.getCommand());
      }
    }
    else if (strcmp("help", cmdParser.getCommand()) == 0)
    {
      callback_func_help();
    }
    else
    {
      // Command not found
      helper_clear_output_buffer();
      sprintf(_output_buffer, "Error, command not found: '%s'.Type 'help' for a list of known commands.",
              cmdParser.getCommand());
      Serial.println(_output_buffer);

    }

    telemetry_generate();
    // output buffer content has been copied to telemetry, safe to clear.
    helper_clear_output_buffer();

}

void callback_funct_hello()
{
  helper_clear_output_buffer();
  sprintf(_output_buffer, "Hello no. %s", cmdParser.getCmdParam(1));
  // Serial.println(_output_buffer);
}

void callback_func_help()
{
  helper_clear_output_buffer();
  sprintf(_output_buffer, "Help messages to follow.");
  helper_queue_messages("Valid commands:");
  helper_queue_messages("query, returns the telemetry, triggers the sending of the next output message in queue, if any.");
  helper_queue_messages("help, displays the help message.");
}

void callback_car_m_move(int left_speed, int right_speed, uint32_t duration)
{

  op_data.car.mode = CAR_MODE_MANUAL;
  op_data.car.manual_mode_left_speed = left_speed;
  op_data.car.mnaual_mode_right_speed = right_speed;
  op_data.car.manual_move_duration = duration;
  op_data.car._manual_move_start_time = op_data.time_now;
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
