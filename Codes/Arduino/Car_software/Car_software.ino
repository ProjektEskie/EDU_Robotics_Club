
#include "definitions.hpp"
#include "helpers.hpp"
#include "car.hpp"
#include "IMU.hpp"
#include "BLE_Comm.hpp"
#include "tracker.hpp"


#include <CmdParser.hpp>
#include <ArduinoJson.h>
#include "Arduino_NineAxesMotion.h"
#include <cppQueue.h>  

#define OUTPUT_MESSAGE_QUEUE_CAPACITY 20

#undef ARDUINOJSON_SLOT_ID_SIZE
#define ARDUINOJSON_SLOT_ID_SIZE 1
#undef ARDUINOJSON_STRING_LENGTH_SIZE
#define ARDUINOJSON_STRING_LENGTH_SIZE 1
#undef ARDUINOJSON_USE_DOUBLE
#define ARDUINOJSON_USE_DOUBLE 0
#undef ARDUINOJSON_USE_LONG_LONG
#define ARDUINOJSON_USE_LONG_LONG 0


operation_data op_data;

char _json_buffer[JSON_BUFFER_LEN];

char _input_buffer[BLE_IO_SERVICE_BUFFER_LEN];
char _output_buffer[BLE_IO_SERVICE_BUFFER_LEN];
char _telemetry_notify_buffer[BLE_IO_SERVICE_BUFFER_LEN];

char _queue_buffer[OUTPUT_MESSAGE_QUEUE_CAPACITY][BLE_IO_SERVICE_BUFFER_LEN];
cppQueue _output_queue(BLE_IO_SERVICE_BUFFER_LEN, OUTPUT_MESSAGE_QUEUE_CAPACITY, FIFO, false, _queue_buffer, sizeof(_queue_buffer));

track_point _tracker_buffer[TRACKER_QUEUE_CAPACITY];
cppQueue tracker_queue(sizeof(track_point), TRACKER_QUEUE_CAPACITY, FIFO, true, _tracker_buffer, sizeof(_tracker_buffer));


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
  if (ENABLE_IMU)
  {
    IMU_Init();
  }

  // Setup the car
  CAR_init();

  BLE_Comm_init();

  // reset the buffers
  helper_clear_input_buffer();
  helper_clear_output_buffer();

  // Initalized some numbers
  op_data.time_since_last_telemetry = 0;
  op_data.n_cycles_since_last_telemetry = 0;

  sync_pulse_reset();

  tracker_init();

  Serial.println("Ready!");

}


void loop() {

  static uint32_t _last_loop_time = 0;
  _last_loop_time = op_data.time_now;
  op_data.time_now = millis();

  // Serial.print("Main - Loop time: \t");
  // Serial.println(op_data.time_now - _last_loop_time);

  sync_pulse_update();

  if (ENABLE_IMU)
  {
    IMU_update();
  }

  CAR_update();

  if (op_data.sync.pulse_500ms)
  {
    telemetry_generate();
  }
  BLE_Comm_update();

  cmd_parse();

  tracker_update();

  op_data.n_cycles_since_last_telemetry++;
  sync_pulse_reset();
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

  if (strcmp("car_m_move", cmdParser.getCommand()) == 0)
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
      CAR_API_car_m_move(left_speed,
                          right_speed,
                          duration);
      helper_clear_output_buffer();
      sprintf(_output_buffer, "Success, '%s'. Moving. Speeds: L%i R%i",
              cmdParser.getCommand(), left_speed, right_speed);
      helper_queue_messages(_output_buffer);
    }
  }
  else if (strcmp("car_set_mode", cmdParser.getCommand()) == 0)
  {
    if (cmdParser.getParamCount() == 1)
    {
      uint8_t requested_mode = (uint8_t)atoi(cmdParser.getCmdParam(1));
      CAR_API_set_mode(requested_mode);
    }
    else
    {
      helper_clear_output_buffer();
      sprintf(_output_buffer, "Error, '%s' command accepts exactly 1 arguement",
              cmdParser.getCommand());
      helper_queue_messages(_output_buffer);
      helper_queue_messages("Info: car_set_mode (car, set mode) example usage:");
      helper_queue_messages("Info: car_set_mode [mode], where mode is defined in the car_mode enum");
      helper_queue_messages("Info: car_set_mode 0");
      helper_queue_messages("Info: car_set_mode known modes: 0 - Idle, 1 - manual, 2 - heading keep, 3 - point and go");
    }
  }
  else if (strcmp("car_set_heading", cmdParser.getCommand()) == 0)
  {
    if (cmdParser.getParamCount() == 1)
    {
      float requested_heading = atof(cmdParser.getCmdParam(1));
      CAR_API_set_heading(requested_heading);
    }
    else
    {
      helper_clear_output_buffer();
      sprintf(_output_buffer, "Error, '%s' command accepts exactly 1 arguement",
              cmdParser.getCommand());
      helper_queue_messages(_output_buffer);
      helper_queue_messages("Info: car_set_heading (car, set heading) example usage:");
      helper_queue_messages("Info: car_set_heading [heading], where heading is the direction the car is to face");
      helper_queue_messages("Info: car_set_heading 90");
    }
  }
  else if (strcmp("car_set_png_param", cmdParser.getCommand()) == 0)
  {
    if (cmdParser.getParamCount() == 3)
    {

      float target_heading;
      int speed;
      uint32_t duration;

      helper_clear_output_buffer();
      sprintf(_output_buffer, "car_set_png_param, you've entered parameters of '%s' '%s' '%s'",
              cmdParser.getCmdParam(1),
              cmdParser.getCmdParam(2),
              cmdParser.getCmdParam(3));
      helper_queue_messages(_output_buffer);

      target_heading = atof(cmdParser.getCmdParam(1));
      speed = atoi(cmdParser.getCmdParam(2));
      duration = atol(cmdParser.getCmdParam(3));

      CAR_API_set_PNG_settings( target_heading, speed, duration);
    }
    else
    {
      helper_clear_output_buffer();
      sprintf(_output_buffer, "Error, '%s' command accepts exactly 3 arguements, heading, straight line speed and duration",
              cmdParser.getCommand());
      helper_queue_messages(_output_buffer);
    }

  }
  else if (strcmp("help", cmdParser.getCommand()) == 0)
  {
    callback_func_help();
  }
  else if (strcmp("car_set_servo", cmdParser.getCommand()) == 0)
  {
    if (cmdParser.getParamCount() == 1)
    {
      int angle = atoi(cmdParser.getCmdParam(1));
      CAR_API_set_Servo_angle(angle);
    }
    else
    {
      helper_clear_output_buffer();
      sprintf(_output_buffer, "Error, '%s' command accepts exactly 1 arguement",
              cmdParser.getCommand());
      helper_queue_messages(_output_buffer);
      helper_queue_messages("Info: car_set_servo (car, set servo angle) example usage:");
      helper_queue_messages("Info: car_set_servo [angle], where angle is the angle to set the servo to");
      helper_queue_messages("Info: car_set_servo 0");
    }
  }
  else if (strcmp("car_do_ranging", cmdParser.getCommand()) == 0)
  {
    CAR_API_start_ranging_scan();
  }
  else if (strcmp("car_ping", cmdParser.getCommand()) == 0)
  {
    int range = CAR_echo_range_cm();
    helper_queue_formatted_message("Info: car_ping, heading: %i deg, range: %i cm",
                                   op_data.car.servo_angle,
                                   range);
  }
  else if (strcmp("ble_set_name", cmdParser.getCommand()) == 0)
  {
    if (cmdParser.getParamCount() == 1)
    {
      if (strlen(cmdParser.getCmdParam(1)) > 19)
      {
        helper_clear_output_buffer();
        sprintf(_output_buffer, "Error, '%s' command accepts a name of 19 characters or less",
                cmdParser.getCommand());
        helper_queue_messages(_output_buffer);
      }
      else
      {
        strcpy(op_data.ble.car_name, cmdParser.getCmdParam(1));
        sprintf(_output_buffer, "Success, '%s'. Name set to: %s.",
                cmdParser.getCommand(), op_data.ble.car_name);
        helper_save_BLE_name();
        helper_queue_messages(_output_buffer);
        helper_queue_messages("Info: Reboot the car for the name to take effect.");
      }
    }
    else
    {
      helper_clear_output_buffer();
      sprintf(_output_buffer, "Error, '%s' command accepts exactly 1 arguement",
              cmdParser.getCommand());
      helper_queue_messages(_output_buffer);
      helper_queue_messages("Info: ble_set_name (ble, set name) example usage:");
      helper_queue_messages("Info: ble_set_name [name], where name is the name to set the BLE device to");
      helper_queue_messages("Info: ble_set_name RClub_Car");
      helper_queue_messages("Info: name needs to be 19 characters or less");
    }
  }
  else if (strcmp("car_m_auto", cmdParser.getCommand()) == 0)
  {
    if (cmdParser.getParamCount() == 3)
    {
      float target_heading;
      int forward_speed;
      uint32_t forward_duration;

      helper_clear_output_buffer();
      sprintf(_output_buffer, "car_m_auto, you've entered parameters of '%s' '%s' '%s'",
              cmdParser.getCmdParam(1),
              cmdParser.getCmdParam(2),
              cmdParser.getCmdParam(3));
      helper_queue_messages(_output_buffer);

      target_heading = atof(cmdParser.getCmdParam(1));
      forward_speed = atoi(cmdParser.getCmdParam(2));
      forward_duration = atol(cmdParser.getCmdParam(3));

      CAR_API_set_auto_settings(forward_speed, target_heading, forward_duration);
    }
    else
    {
      helper_clear_output_buffer();
      sprintf(_output_buffer, "Error, '%s' command accepts exactly 3 arguements, target_heading, forward_speed, forward_duration",
              cmdParser.getCommand());
      helper_queue_messages(_output_buffer);
    }
  }
  else if (strcmp("car_turn_rate", cmdParser.getCommand()) == 0)
  {
    if (cmdParser.getParamCount() == 1)
    {
      float turn_rate;
      turn_rate = atof(cmdParser.getCmdParam(1));
      op_data.car.turning_rate = turn_rate;
      CAR_API_set_mode(CAR_MODE_TURN_RATE);
    }
    else
    {
      helper_clear_output_buffer();
      sprintf(_output_buffer, "Error, '%s' command accepts exactly 1 arguements, turn rate",
              cmdParser.getCommand());
      helper_queue_messages(_output_buffer);
    }
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


void callback_func_help()
{
  helper_clear_output_buffer();
  helper_queue_messages("Valid commands:");
  helper_queue_messages("help, displays the help message.");
  helper_queue_messages("car_m_move, move the car by manualcommand.");
  helper_queue_messages("car_set_mode, select which automatic mode of the car to use.");
  helper_queue_messages("car_set_heading, Set the heading for heading_keeping mode.");
  helper_queue_messages("car_set_png_param, Set the parameters for the point-and-go mode.");
  helper_queue_messages("car_set_servo, Set the angle of the servo.");
  helper_queue_messages("car_do_ranging, Perform a ranging scan across the front arc of the car.");
  helper_queue_messages("car_ping, WIP. Triggers a test function that performs one echo ranging test.");
  helper_queue_messages("ble_set_name, Set the name of the BLE device.");
  helper_queue_messages("car_m_auto, Set the parameters for the automatic mode.");
  helper_queue_messages("car_turn_rate, Set the turn rate for the car.");
}

// Look at the time stored in sync, and if it has been long enough, update the pulse
void sync_pulse_update()
{
  if ((op_data.time_now - op_data.sync._10ms_start_time) >= 10)
  {
    op_data.sync._10ms_start_time = op_data.time_now;
    op_data.sync.pulse_10ms = 1;
  }

  if ((op_data.time_now - op_data.sync._50ms_start_time) >= 50)
  {
    op_data.sync._50ms_start_time = op_data.time_now;
    op_data.sync.pulse_50ms = 1;
  }

  if ((op_data.time_now - op_data.sync._100ms_start_time) >= 100)
  {
    op_data.sync._100ms_start_time = op_data.time_now;
    op_data.sync.pulse_100ms = 1;
  }

  if ((op_data.time_now - op_data.sync._500ms_start_time) >= 500)
  {
    op_data.sync._500ms_start_time = op_data.time_now;
    op_data.sync.pulse_500ms = 1;
  }

  if ((op_data.time_now - op_data.sync._1000ms_start_time) >= 1000)
  {
    op_data.sync._1000ms_start_time = op_data.time_now;
    op_data.sync.pulse_1000ms = 1;
  }
}

void sync_pulse_reset()
{
  op_data.sync.pulse_10ms = 0;
  op_data.sync.pulse_50ms = 0;
  op_data.sync.pulse_100ms = 0;
  op_data.sync.pulse_500ms = 0;
  op_data.sync.pulse_1000ms = 0;
}

void telemetry_generate()
{
  JsonDocument doc;

  doc["status"] = op_data.status;
  // doc["time_ms"] = op_data.time_now;
  doc["t_last"] = op_data.time_since_last_telemetry;
  doc["n_cycles"] = op_data.n_cycles_since_last_telemetry;
  doc["ble_rssi"] = op_data.ble.rssi;
  // doc["n_out"] = _output_queue.getCount();

  JsonObject IMU = doc["IMU"].to<JsonObject>();
  IMU["cal"].add(op_data.imu.system_calibration_status);
  IMU["cal"].add(op_data.imu.gryo_calibration_status);
  IMU["cal"].add(op_data.imu.accel_calibration_status);
  IMU["cal"].add(op_data.imu.mag_calibration_status);

  IMU["euler"].add(op_data.imu.euler_heading);
  // IMU["euler"].add(op_data.imu.euler_pitch);
  // IMU["euler"].add(op_data.imu.euler_roll);

  IMU["gyro_z"] = op_data.imu.gyro_z;

  IMU["lin_accel"].add(op_data.imu.linaccel_x);
  // IMU["lin_accel"].add(op_data.imu.linaccel_y);
  // IMU["lin_accel"].add(op_data.imu.linaccel_z);

  JsonObject CAR = doc["CAR"].to<JsonObject>();
  CAR["servo_angle"] = op_data.car.servo_angle;
  CAR["mode"] = op_data.car.mode;

  memset(_json_buffer, 0, JSON_BUFFER_LEN);

  doc.shrinkToFit();  // optional
  size_t n_bytes = measureJson(doc);
  if (n_bytes >= JSON_BUFFER_LEN)
  {
    // Serial.println("Error: JSON buffer is too small for the telemetry data.");
    helper_queue_messages("Error: JSON buffer is too small for the telemetry data.");
  }
  else if (n_bytes == 0)
  {
    // Serial.println("Error: JSON serialization failed.");
    helper_queue_messages("Error: JSON serialization failed.");
  }
  else
  {
    serializeJson(doc, _json_buffer);
    Serial.print("Telemetry JSON: ");
    Serial.println(_json_buffer);
    helper_queue_formatted_message("TELE:%s", _json_buffer);
  }  

  IMU_reset_n_updates_counter();

  op_data.time_since_last_telemetry = op_data.time_now - op_data.last_telemetry_time;
  op_data.last_telemetry_time = op_data.time_now;
  op_data.has_new_telemetry = 1;
  op_data.n_cycles_since_last_telemetry = 0;

}