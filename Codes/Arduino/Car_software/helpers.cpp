#include <cstdarg>
#include "helpers.hpp"
#include "definitions.hpp"
#include <cppQueue.h>  

// Calcualte the angle between two points on the compass
// Returns: difference in angles in degrees, positve is in the clockwise direction
// Rounded to 2 decimal places
float helper_angle_diff(float current_angle, float target_angle)
{
  int angle_diff = 0;

  angle_diff = int((current_angle - target_angle) * 100);
  angle_diff = (angle_diff + 18000 + 36000) % 36000 - 18000;

  return float(angle_diff/100.0);
}

void helper_clear_input_buffer()
{
  extern char _input_buffer[];
  memset(_input_buffer, 0, CMD_INPUT_BUFFER_LEN);
}

void helper_clear_output_buffer()
{
  extern char _output_buffer[];
  memset(_output_buffer, 0, CMD_OUTPUT_BUFFER_LEN);
}

void helper_queue_messages(const char* message)
{
  extern cppQueue _output_queue;
  extern char _output_buffer[];
  if (strlen(message) < (CMD_OUTPUT_BUFFER_LEN - 1))
  {
    sprintf(_output_buffer, "%s", message);
    _output_queue.push(_output_buffer);
  }
  else
  {
    Serial.println("Error, message too long for output buffer.");
    Serial.println(message);
    Serial.println(strlen(message));
  }
}
void helper_queue_messages(char* message)
{
  extern cppQueue _output_queue;

  if (strlen(message) < (CMD_OUTPUT_BUFFER_LEN - 1))
  {

    _output_queue.push(message);
  }
  else
  {
    Serial.println("Error, message too long for output buffer.");
    Serial.println(message);
    Serial.println(strlen(message));
  }
}

void helper_queue_formatted_message(const char *format, ...)
{
  extern char _output_buffer[];
  extern cppQueue _output_queue;
  va_list args;
  helper_clear_output_buffer();
  va_start(args, format);
  vsnprintf(_output_buffer, CMD_OUTPUT_BUFFER_LEN, format, args);
  va_end(args);

  _output_buffer[CMD_OUTPUT_BUFFER_LEN - 1] = '\0';
  _output_queue.push(_output_buffer);
}

void helper_queue_data(float* data, uint8_t n_data)
{
  String data_str = "DATA: ";
  for (int i = 0; i < n_data; i++)
  {
    data_str += String(data[i], 2);
    if (i < (n_data - 1))
    {
      data_str += ", ";
    }
  }
  helper_queue_messages(data_str.c_str());
}

void helper_queue_ranging_data(ranging_data* data)
{
  String data_str = "RANGE: ";
  for (int i = 0; i < RANGING_DATA_SIZE; i++)
  {
    data_str += String(data->distance[i], 3);
    if (i < (RANGING_DATA_SIZE-1))
    {
      data_str += ", ";
    }
  }
  helper_queue_messages(data_str.c_str());
}
