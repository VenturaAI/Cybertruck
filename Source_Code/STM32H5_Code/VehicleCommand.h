#ifndef VEHICLE_COMMAND_H
#define VEHICLE_COMMAND_H

#include <Arduino.h>

/*****************************************************
 *
 * VehicleCommand
 *
 * This structure is intentionally kept identical to
 * the ROS2 VehicleCommand.msg.
 *
 * Speed:
 *   -1.0  = Full Reverse
 *    0.0  = Stop
 *   +1.0  = Full Forward
 *
 * Steering:
 *   -1.0  = Full Left
 *    0.0  = Straight
 *   +1.0  = Full Right
 *
 * Brake:
 *   false = Normal Driving
 *   true  = Stop Vehicle
 *
 *****************************************************/

struct VehicleCommand
{

    float speed;

    float steering;

    bool brake;

    VehicleCommand()
    {
        speed = 0.0f;
        steering = 0.0f;
        brake = false;
    }

};

#endif