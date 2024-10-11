#ifndef RCLUB_CAR_H
#define RCLUB_CAR_H

#include <arduino.h>
#include "CAR_definitions.hpp"
#include "helpers.hpp"

void CAR_init(car_data * cd);
void CAR_update(car_data * cd, uint32_t time_now);

#endif