#ifndef TRACKER_HPP
#define TRACKER_HPP

#include <Arduino.h>

#define TRACKER_QUEUE_CAPACITY 20

// This is a very inefficient packing of the data, but bit fields is too advnaced topic for the club at the moment
typedef struct _track_point
{
    int32_t heading_and_distance;  // in 0.1 degrees and mm
    int lin_accel;  // accel in cm/s/s, high 2 bytes 
    int gyro;       // gyro z in 0.1 degrees/s
    uint8_t echo_range_cm;
    uint8_t status_flags;  // bit 0: motor on/off 
                           // bit 1: Ranging sensor valid
                           // bit 2: Is in auto mode
                           // bit 3-7: auto mode state number
    uint8_t spare_bytes[2]; // for alignement
    uint8_t spare_bytes2[8];

} track_point;

typedef struct _tracker_xy
{
    int32_t x;  // in mm
    int32_t y;  // in mm
} tracker_xy;

void tracker_init();
void tracker_update();

void tracker_set_reference_heading(float heading);
void tracker_set_reference_xy(int32_t x, int32_t y);
void tracker_api_add_xy_delta(int32_t delta_x, int32_t delta_y);
tracker_xy tracker_api_get_current_xy();



#endif // TRACKER_HPP