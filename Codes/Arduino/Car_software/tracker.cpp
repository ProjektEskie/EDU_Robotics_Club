
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
    // If the car is not moving, continue to transmit for the following counts and then stop
    // transmitting until the car moves again
    // This is to avoid flooding the tracker with data when the car is not moving
    // The car is considered to be moving if the average speed is greater than 0
    static uint8_t cont_transmit_counter = 5;

    if (op_data.sync.pulse_100ms)
    {
        average_car_speed = (op_data.car.left_speed + op_data.car.right_speed) / 2;


        // Skip adding any tracking points if the car has not moved
        if ((average_car_speed == 0))
        {
            if (cont_transmit_counter > 0)
            {
                cont_transmit_counter--;
            }
        }
        else
        {
            cont_transmit_counter = 5;
        }

        if (cont_transmit_counter == 0)
        {
            return;
        }

        _prev_average_speed = average_car_speed;
        // Estimate a distance based on power for now, update later when more sensors
        // are available
        int distance_mm = _tracker_distance_estimate(average_car_speed, 100);
        track_point tp;
        int heading = (int)(op_data.imu.euler_heading * 10);
        // Store the heading and distance in a single variable
        tp.heading_and_distance = (int32_t)((heading << 16) | (distance_mm & 0xFFFF));
        int lin_accel_x = (int)(op_data.imu.linaccel_x * 100);
        // Store the linear acceleration in cm/s/s
        tp.lin_accel_x = lin_accel_x;
        tp.echo_range_cm = op_data.car.am_data.range_infront;

        tp.status_flags = 0;
        if (op_data.car.left_speed != 0 || op_data.car.right_speed != 0)
        {
            tp.status_flags |= 0x01; // motor on
        }
        tracker_queue.push(&tp);
    }
}

int _tracker_distance_estimate(int average_car_speed, uint32_t interval_ms)
{
    int distance_mm;
    float speed_mps;
    speed_mps = ((float)average_car_speed / 255.0) * 0.6;
    distance_mm = (int)(speed_mps * interval_ms);
    return distance_mm;
}