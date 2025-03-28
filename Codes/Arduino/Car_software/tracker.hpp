#ifndef TRACKER_HPP
#define TRACKER_HPP

#include <Arduino.h>

#define TRACKER_QUEUE_CAPACITY 100

typedef struct _track_point
{
    uint16_t heading;  // in 0.1 degrees
    uint16_t distance_mm; 
} track_point;

void tracker_init();
void tracker_update();



#endif // TRACKER_HPP