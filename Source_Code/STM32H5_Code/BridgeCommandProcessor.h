#ifndef BRIDGE_COMMAND_PROCESSOR_H
#define BRIDGE_COMMAND_PROCESSOR_H

/******************************************************************************
 * BridgeCommandProcessor
 *
 * Registers RPC methods exposed by the STM32 firmware and forwards them
 * to the MotorDriver.
 *
 * Exposed RPC Methods:
 *   cybertruck.drive(speed, steering, brake)
 *   cybertruck.estop()
 *   cybertruck.heartbeat()
 *
 ******************************************************************************/

#include <Arduino.h>
#include <Arduino_RouterBridge.h>

#include "MotorDriver.h"
#include "UltrasonicManager.h"
#include "VehicleCommand.h"
#include <array>

class BridgeCommandProcessor
{
public:

    BridgeCommandProcessor();

    // Register all RPC callbacks
    bool begin(MotorDriver *motor,
           UltrasonicManager *ultrasonic);

private:

    //--------------------------------------------------
    // Shared Instance
    //--------------------------------------------------

    static MotorDriver *driver;
    static UltrasonicManager *ultrasonic;

    //--------------------------------------------------
    // RPC Callbacks
    //--------------------------------------------------

    static bool drive(float speed,
                      float steering,
                      bool brake);

    static bool estop();

    static bool heartbeat();
    //static std::vector<float> getUltrasonic();
    static std::array<float,3> getUltrasonic();
};

#endif // BRIDGE_COMMAND_PROCESSOR_H