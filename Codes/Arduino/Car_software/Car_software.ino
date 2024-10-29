
#include "definitions.hpp"
#include "helpers.hpp"
#include "car.hpp"
#include "IMU.hpp"
#include "BLE_Comm.hpp"


#include <CmdParser.hpp>
#include <ArduinoJson.h>
#include "Arduino_NineAxesMotion.h"
#include <cppQueue.h>  

#define OUTPUT_MESSAGE_QUEUE_CAPACITY 10

#define ARDUINOJSON_SLOT_ID_SIZE 1
#define ARDUINOJSON_STRING_LENGTH_SIZE 1
#define ARDUINOJSON_USE_DOUBLE 0
#define ARDUINOJSON_USE_LONG_LONG 0


operation_data op_data;

char _json_buffer[JSON_BUFFER_LEN];

char _input_buffer[BLE_IO_SERVICE_BUFFER_LEN];
char _output_buffer[BLE_IO_SERVICE_BUFFER_LEN];

char _queue_buffer[OUTPUT_MESSAGE_QUEUE_CAPACITY][BLE_IO_SERVICE_BUFFER_LEN];
cppQueue _output_queue(BLE_IO_SERVICE_BUFFER_LEN, OUTPUT_MESSAGE_QUEUE_CAPACITY, FIFO, false, _queue_buffer, sizeof(_queue_buffer));

CmdParser cmdParser;

void setup() {
  //Initialize serial and wait for port to open:
  Serial.begin(115200);
  // while (!Serial) {
  //   ; // wait for serial port to connect. Needed for native USB port only
  // }
  delay(1);

  Wire.begin();
  Wire.setClock(400000);
  delay(1);

  // Start the IMU
  // IMU_Init();

  // Setup the car
  CAR_init();

  BLE_Comm_init();

  // reset the buffers
  helper_clear_input_buffer();
  helper_clear_output_buffer();

  // Initalized some numbers
  op_data.time_since_last_telemetry = 0;

  Serial.println("Ready!");
}


void loop() {

  op_data.time_now = millis();

  // IMU_update();

  CAR_update();

  if ((op_data.time_now - op_data.last_telemetry_time) > TELEMETRY_UPDATE_INTERNVAL)
  {
    telemetry_generate();
  }
  BLE_Comm_update();

  cmd_parse();
}

void cmd_parse()
{
  // Serial.print("cmd_parse: parsing: ");
  // Serial.println(_input_buffer);

  // Exit immediately if the input buffer is empty
  if (_input_buffer[0] == 0)
  {
    return;
  }

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
    helper_queue_messages("Invalid command");
    helper_clear_input_buffer();
    return;
  }

  if (strcmp("Hello", cmdParser.getCommand()) == 0)
  {
    callback_funct_hello();
  }
  else if (strcmp("car_m_move", cmdParser.getCommand()) == 0)
  {
    if (cmdParser.getParamCount() != 3)
    {
      helper_clear_output_buffer();
      sprintf(_output_buffer, "Error, '%s' command accepts exactly 3 arguements",
              cmdParser.getCommand());
      helper_queue_messages(_output_buffer);
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
      sprintf(_output_buffer, "Success, '%s'. Moving. Speeds: L%i R%i",
              cmdParser.getCommand(), left_speed, right_speed);
      helper_queue_messages(_output_buffer);
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
    helper_queue_messages(_output_buffer);
    Serial.println(_output_buffer);

  }

  // Command has been parsed, clear the buffer for next time
  helper_clear_input_buffer();

}

void callback_funct_hello()
{
  helper_clear_output_buffer();
  sprintf(_output_buffer, "Hello no. %s", cmdParser.getCmdParam(1));
  helper_queue_messages(_output_buffer);
  // Serial.println(_output_buffer);
}

void callback_func_help()
{
  helper_clear_output_buffer();
  helper_queue_messages("Valid commands:");
  helper_queue_messages("help, displays the help message.");
}

void callback_car_m_move(int left_speed, int right_speed, uint32_t duration)
{
  op_data.car.mode = CAR_MODE_MANUAL;
  op_data.car.mm_data.mm_left_speed = left_speed;
  op_data.car.mm_data.mm_right_speed = right_speed;
  op_data.car.mm_data.mm_duration = duration;
  op_data.car.mm_data._mm_start_time = op_data.time_now;
}

void telemetry_generate()
{
  JsonDocument doc;

  doc["status"] = op_data.status;
  doc["time_ms"] = op_data.time_now;
  doc["t_last"] = op_data.time_since_last_telemetry;
  doc["ble_rssi"] = op_data.ble.rssi;

  JsonObject IMU = doc["IMU"].to<JsonObject>();
  IMU["last_updated"] = op_data.imu.last_update_time;
  IMU["n_updates"] = op_data.imu.n_updates_counter;
  
  JsonObject IMU_calibration = IMU["calibration"].to<JsonObject>();
  IMU_calibration["system_cal"] = op_data.imu.system_calibration_status;
  IMU_calibration["gryo_cal"] = op_data.imu.gryo_calibration_status;
  IMU_calibration["accel_cal"] = op_data.imu.accel_calibration_status;
  IMU_calibration["mag_cal"] = op_data.imu.mag_calibration_status;

  JsonObject IMU_euler_angles = IMU["euler_angles"].to<JsonObject>();
  IMU_euler_angles["heading"] = op_data.imu.euler_heading;
  IMU_euler_angles["pitch"] = op_data.imu.euler_pitch;
  IMU_euler_angles["roll"] = op_data.imu.euler_roll;

  JsonObject IMU_lin_accel = IMU["lin_accel"].to<JsonObject>();
  IMU_lin_accel["lin_accel_x"] = op_data.imu.linaccel_x;
  IMU_lin_accel["lin_accel_y"] = op_data.imu.linaccel_y;
  IMU_lin_accel["lin_accel_z"] = op_data.imu.linaccel_z;

  JsonObject CAR_speeds = doc["CAR"]["speeds"].to<JsonObject>();
  CAR_speeds["left"] = op_data.car.left_speed;
  CAR_speeds["right"] = op_data.car.right_speed;

  memset(_json_buffer, 0, JSON_BUFFER_LEN);

  doc.shrinkToFit();  // optional

  serializeJson(doc, _json_buffer);

  IMU_reset_n_updates_counter();

  op_data.time_since_last_telemetry = op_data.time_now - op_data.last_telemetry_time;
  op_data.last_telemetry_time = op_data.time_now;
  op_data.has_new_telemetry = 1;

}
