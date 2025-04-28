
#include <Arduino.h>
#include <cppQueue.h> 
#include "definitions.hpp"

#include "tracker.hpp"
#include "helpers.hpp"

#define _CONT_TRANSMIT_COUNTER 10

extern operation_data op_data;
extern cppQueue tracker_queue;

float _tracker_reference_heading = 0.0f; // in degrees
tracker_xy _current_xy = {0, 0}; // in mm

// Estimate the distance traveled based on the power
// level of the car and the time interval, returns
// distance as integer in mm. Currently assumes
// the car travels at speed of 1m/s when the power
// level is 255, and 0m/s when the power level is 90.
int _tracker_distance_estimate(int average_car_speed, uint32_t interval_ms);
bool _should_transmit(int average_car_speed);


void tracker_init()
{
    tracker_set_reference_heading(op_data.imu.euler_heading);
    tracker_set_reference_xy(0, 0);
    tracker_queue.flush();
}

void tracker_update()
{
    static int average_car_speed, _prev_average_speed;
    // If the car is not moving, continue to transmit for the following counts and then stop
    // transmitting until the car moves again
    // This is to avoid flooding the tracker with data when the car is not moving
    // The car is considered to be moving if the average speed is greater than 0
    static uint8_t cont_transmit_counter = _CONT_TRANSMIT_COUNTER;

    if (op_data.sync.pulse_100ms)
    {
        average_car_speed = (op_data.car.left_speed + op_data.car.right_speed) / 2;


        // Skip adding any tracking points if the car has not moved
        if (_should_transmit(average_car_speed))
        {

            cont_transmit_counter = _CONT_TRANSMIT_COUNTER;
        }
        else
        {
            // If the car is not moving, continue to transmit for the following counts and then stop
            // transmitting until the car moves again
            // This is to avoid flooding the tracker with data when the car is not moving
            // The car is considered to be moving if the average speed is greater than 0
            if (cont_transmit_counter > 0)
            {
                cont_transmit_counter--;
            }
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
        // Ranging data is only valid if the car is in auto mode, data is stale otherwise
        if (op_data.car.mode == CAR_MODE_AUTO)
        {
            tp.status_flags |= 0x02; // ranging sensor valid
        }

        // The auto mode state is stored in the last 5 bits of the status flags
        uint8_t auto_mode_state = (uint8_t)op_data.car.am_data.step;
        if (auto_mode_state > 31)
        {
            auto_mode_state = 31;
        }
        tp.status_flags |= (auto_mode_state << 3); // auto mode state

        // Set bit 2 to 1 if the car is in auto mode
        if (op_data.car.mode == CAR_MODE_AUTO)
        {
            tp.status_flags |= 0x04; // auto mode
        }
        tracker_queue.push(&tp);
    }
}

void tracker_set_reference_heading(float heading)
{
    _tracker_reference_heading = heading;
}

void tracker_set_reference_xy(int32_t x, int32_t y)
{
    _current_xy.x = x;
    _current_xy.y = y;
}

int _tracker_distance_estimate(int average_car_speed, uint32_t interval_ms)
{
    int distance_mm;
    float speed_mps;
    speed_mps = ((float)average_car_speed / 255.0) * 0.6;
    distance_mm = (int)(speed_mps * interval_ms);
    return distance_mm;
}

bool _should_transmit(int average_car_speed)
{
    // If the car is not moving, continue to transmit for the following counts and then stop
    // transmitting until the car moves again
    // This is to avoid flooding the tracker with data when the car is not moving
    // The car is considered to be moving if the average speed is greater than 0
    if ((average_car_speed != 0) || (op_data.car.mode == CAR_MODE_AUTO))
    // If the car is in auto mode, always transmit
    {
        return true;
    }
    else
    {
        return false;
    }
}