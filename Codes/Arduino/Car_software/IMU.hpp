#ifndef RCLUB_IMU_H
#define RCLUB_IMU_H

#include <arduino.h>
#include "definitions.hpp"

void IMU_Init();
void IMU_update();
void IMU_reset_n_updates_counter();

#endif