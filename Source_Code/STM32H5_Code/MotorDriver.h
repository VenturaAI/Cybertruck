#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

/******************************************************************************
 * MotorDriver
 *
 * Controls the traction motor and steering motor.
 * Receives VehicleCommand objects from the communication layer.
 ******************************************************************************/

#include <Arduino.h>

#include "VehicleCommand.h"

class MotorDriver
{
public:

    //--------------------------------------------------
    // Constructor
    //--------------------------------------------------

    MotorDriver();

    //--------------------------------------------------
    // Initialize Hardware
    //--------------------------------------------------

    void begin();

    //--------------------------------------------------
    // Execute Command
    //--------------------------------------------------

    void executeCommand(const VehicleCommand &cmd);

    //--------------------------------------------------
    // Periodic Update
    //--------------------------------------------------

    void update();

    //--------------------------------------------------
    // Emergency Stop
    //--------------------------------------------------

    void emergencyStop();

private:

    //--------------------------------------------------
    // Watchdog
    //--------------------------------------------------

    unsigned long lastCommandTime;

    static constexpr unsigned long COMMAND_TIMEOUT_MS = 500;

    //--------------------------------------------------
    // Steering Timer
    //--------------------------------------------------

    bool steeringActive;

    unsigned long steeringStartTime;

    unsigned long steeringDuration;

    //--------------------------------------------------
    // Internal Functions
    //--------------------------------------------------

    void drive(float speed);

    void steer(float steering);

    void stopDrive();

    void stopSteering();

    void brakeDrive();

    //--------------------------------------------------
    // Helper Functions
    //--------------------------------------------------

    uint8_t speedToPWM(float speed);

    uint16_t steeringToPulse(float steering);
};

#endif