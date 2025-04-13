#ifndef HELPERS_H
#define HELPERS_H

#include <arduino.h>
#include <EEPROM.h>
#include "definitions.hpp"

float helper_angle_diff(float current_angle, float target_angle);
float helper_angle_add(float angle, float delta_angle);
void helper_clear_input_buffer();
void helper_clear_output_buffer();
void helper_queue_messages(const char* message);
void helper_queue_messages(char* message);
void helper_queue_formatted_message(const char *format_str, ...);
void helper_queue_data(float* data, uint8_t n_data); // Transmits an array of float. Keep the number of datapoints to less than 10 please.
void helper_queue_ranging_data(ranging_data* data);

bool helper_load_BLE_name();
void helper_save_BLE_name();

#endif