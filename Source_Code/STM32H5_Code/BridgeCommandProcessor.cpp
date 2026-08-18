/******************************************************************************
 * BridgeCommandProcessor.cpp
 *
 * Registers RPC methods exposed by the STM32 firmware and forwards them
 * to the MotorDriver.
 ******************************************************************************/

#include "BridgeCommandProcessor.h"
#include <array>

//--------------------------------------------------
// Static Members
//--------------------------------------------------

MotorDriver* BridgeCommandProcessor::driver = nullptr;
UltrasonicManager* BridgeCommandProcessor::ultrasonic = nullptr;

//--------------------------------------------------
// Constructor
//--------------------------------------------------

BridgeCommandProcessor::BridgeCommandProcessor()
{
}

//--------------------------------------------------
// Register RPC Methods
//--------------------------------------------------

bool BridgeCommandProcessor::begin(
    MotorDriver *motor,
    UltrasonicManager *ultra)
{
    driver = motor;
    ultrasonic = ultra;

    //--------------------------------------------------
    // Validate pointers
    //--------------------------------------------------

    if (driver == nullptr || ultrasonic == nullptr)
    {
        Monitor.println("[ERROR] Invalid driver pointer.");
        return false;
    }

    bool success = true;

    //--------------------------------------------------
    // Register Drive RPC
    //--------------------------------------------------

    if (!Bridge.provide("cybertruck.drive", drive))
    {
        Monitor.println("[ERROR] Failed to register cybertruck.drive");
        success = false;
    }
    else
    {
        Monitor.println("[OK] Registered: cybertruck.drive");
    }

    //--------------------------------------------------
    // Register Emergency Stop RPC
    //--------------------------------------------------

    if (!Bridge.provide("cybertruck.estop", estop))
    {
        Monitor.println("[ERROR] Failed to register cybertruck.estop");
        success = false;
    }
    else
    {
        Monitor.println("[OK] Registered: cybertruck.estop");
    }

    //--------------------------------------------------
    // Register Heartbeat RPC
    //--------------------------------------------------

    if (!Bridge.provide("cybertruck.heartbeat", heartbeat))
    {
        Monitor.println("[ERROR] Failed to register cybertruck.heartbeat");
        success = false;
    }
    else
    {
        Monitor.println("[OK] Registered: cybertruck.heartbeat");
    }

    //--------------------------------------------------
    // Register Ultrasonic RPC
    //--------------------------------------------------

    if (!Bridge.provide("cybertruck.ultrasonic", getUltrasonic))
    {
        Monitor.println("[ERROR] Failed to register cybertruck.ultrasonic");
        success = false;
    }
    else
    {
        Monitor.println("[OK] Registered: cybertruck.ultrasonic");
    }

    //--------------------------------------------------
    // Final Status
    //--------------------------------------------------

    if (success)
    {
        Monitor.println("[OK] RPC Bridge Ready.");
    }

    return success;
}

//--------------------------------------------------
// RPC : Drive Vehicle
//--------------------------------------------------

bool BridgeCommandProcessor::drive(
    float speed,
    float steering,
    bool brake)
{
    if (driver == nullptr)
    {
        return false;
    }

    VehicleCommand cmd;

    cmd.speed = speed;
    cmd.steering = steering;
    cmd.brake = brake;

    driver->executeCommand(cmd);

    return true;
}

//--------------------------------------------------
// RPC : Emergency Stop
//--------------------------------------------------

bool BridgeCommandProcessor::estop()
{
    if (driver == nullptr)
    {
        return false;
    }

    driver->emergencyStop();

    Monitor.println("[INFO] Emergency Stop Executed.");

    return true;
}

//--------------------------------------------------
// RPC : Heartbeat
//--------------------------------------------------

bool BridgeCommandProcessor::heartbeat()
{
    return true;
}

//--------------------------------------------------
// RPC : Ultrasonic
//--------------------------------------------------

std::array<float, 3> BridgeCommandProcessor::getUltrasonic()
{
    if (ultrasonic == nullptr)
    {
        return { -1.0f, -1.0f, -1.0f };
    }

    return {
        ultrasonic->frontDistance(),
        ultrasonic->leftDistance(),
        ultrasonic->rightDistance()
    };
}