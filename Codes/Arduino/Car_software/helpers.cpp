#include "helpers.hpp"
#include "definitions.hpp"
#include <cppQueue.h>  

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
  if (strlen(message) < (CMD_OUTPUT_BUFFER_LEN - 1))
  {
    _output_queue.push(message);
  }
  else
  {
    Serial.println("Error, message too long for output buffer.");
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
  }
}
