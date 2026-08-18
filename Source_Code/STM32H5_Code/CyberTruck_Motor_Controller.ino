/******************************************************************************
 * CyberTruck Motor Controller
 * STM32 Firmware - Arduino UNO Q
 ******************************************************************************/

#include <Arduino.h>
#include <Arduino_RouterBridge.h>

#include "Config.h"
#include "VehicleCommand.h"
#include "MotorDriver.h"
#include "CommandProcessor.h"
#include "BridgeCommandProcessor.h"
#include "UltrasonicManager.h"

//====================================================
// Configuration
//====================================================

// Enable only for debugging.
// Leave as 0 for normal ROS2/RPC operation.
#define ENABLE_SERIAL_COMMANDS 0

//====================================================
// Global Objects
//====================================================

MotorDriver motor;
UltrasonicManager ultrasonic;
BridgeCommandProcessor bridgeProcessor;

#if ENABLE_SERIAL_COMMANDS
CommandProcessor commandProcessor;
#endif

//====================================================
// Setup
//====================================================
void setup()
{
    Bridge.begin();
    Monitor.begin(115200);

    motor.begin();
    ultrasonic.begin();

    if (!bridgeProcessor.begin(&motor, &ultrasonic))
    {
        while (true)
        {
            delay(100);
        }
    }
}

void loop()
{
    motor.update();
    ultrasonic.update();
    delay(MAIN_LOOP_DELAY);
}



