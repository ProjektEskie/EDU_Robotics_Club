#ifndef TRACKER_HPP
#define TRACKER_HPP

#include <Arduino.h>

#define TRACKER_QUEUE_CAPACITY 20

// This is a very inefficient packing of the data, but bit fields is too advnaced topic for the club at the moment
typedef struct _track_point
{
    int32_t heading_and_distance;  // in 0.1 degrees and mm
    int32_t lin_accel_x;  // in cm/s/s,
    uint8_t echo_range_cm;
    uint8_t status_flags;  // bit 0: motor on/off 
                           // bit 1: Ranging sensor valid
                           // bit 2: Is in auto mode
                           // bit 3-7: auto mode state number
    uint8_t spare_bytes[2];
} track_point;

void tracker_init();
void tracker_update();



#endif // TRACKER_HPP