#ifndef CAR_DEFINITIONS_H
#define CAR_DEFINITIONS_H

#define IS_EGLOO_PLATFORM 1

#define PIN_BATTERY_SENSE A0


#if IS_EGLOO_PLATFORM
  const int left_direction_pin = A3;
  const int left_direction_pin_in2 = 8;
  const int right_direction_pin = 9;
  const int right_direction_pin_in4 = 11;
  const int left_speed_pin = 5;
  const int right_speed_pin = 6;
#else
  const int left_direction_pin = 4;
  const int right_direction_pin = 3;
  const int left_speed_pin = 6;
  const int right_speed_pin = 5;
#endif

#endif