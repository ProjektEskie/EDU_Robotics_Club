
#include <Arduino.h>
#include <cppQueue.h> 
#include "definitions.hpp"

#include "tracker.hpp"
#include "helpers.hpp"

extern operation_data op_data;
extern cppQueue tracker_queue;

// Estimate the distance traveled based on the power
// level of the car and the time interval, returns
// distance as integer in mm. Currently assumes
// the car travels at speed of 1m/s when the power
// level is 255, and 0m/s when the power level is 90.
int _tracker_distance_estimate(int average_car_speed, uint32_t interval_ms);

void tracker_init()
{
    tracker_queue.flush();
}

void tracker_update()
{
    static int average_car_speed, _prev_average_speed;

    if (op_data.sync.pulse_100ms)
    {
        average_car_speed = (op_data.car.left_speed + op_data.car.right_speed) / 2;

        // Skip adding any tracking points if the car has not moved
        if ((average_car_speed == 0) and (_prev_average_speed == 0))
        {
            return;
        }

        _prev_average_speed = average_car_speed;
        // Estimate a distance based on power for now, update later when more sensors
        // are available
        int distance_mm = _tracker_distance_estimate(average_car_speed, 100);
        track_point tp;
        tp.heading = (uint16_t)(op_data.imu.euler_heading*10);
        tp.distance_mm = distance_mm;
        tracker_queue.push(&tp);
    }
}

int _tracker_distance_estimate(int average_car_speed, uint32_t interval_ms)
{
    int distance_mm;
    float speed_mps;
    speed_mps = (average_car_speed / 255.0) * 1.0;
    distance_mm = (int)(speed_mps * interval_ms);
    return distance_mm;
}