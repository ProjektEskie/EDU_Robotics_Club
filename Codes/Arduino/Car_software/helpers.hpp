#ifndef HELPERS_H
#define HELPERS_H

#include <arduino.h>
#include "definitions.hpp"


void helper_clear_input_buffer();
void helper_clear_output_buffer();
void helper_queue_messages(const char* message);
void helper_queue_messages(char* message);
#endif