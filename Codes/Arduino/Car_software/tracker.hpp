#ifndef TRACKER_HPP
#define TRACKER_HPP

#include <Arduino.h>

#define TRACKER_QUEUE_CAPACITY 20

// This is a very inefficient packing of the data, but bit fields is too advnaced topic for the club at the moment
typedef struct _track_point
{
    int32_t heading;  // in 0.1 degrees
    int32_t distance_mm; 
    uint8_t echo_range_cm;
    uint8_t spare_bytes[3];
} track_point;

void tracker_init();
void tracker_update();



#endif // TRACKER_HPP