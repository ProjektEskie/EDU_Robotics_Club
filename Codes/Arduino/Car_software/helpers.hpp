#ifndef HELPERS_H
#define HELPERS_H

#include <arduino.h>
#include "definitions.hpp"

float helper_angle_diff(float current_angle, float target_angle);
void helper_clear_input_buffer();
void helper_clear_output_buffer();
void helper_queue_messages(const char* message);
void helper_queue_messages(char* message);
void helper_queue_formatted_message(const char *format_str, ...);
void helper_queue_data(float* data, uint8_t n_data); // Transmits an array of float. Keep the number of datapoints to less than 10 please.

#endif