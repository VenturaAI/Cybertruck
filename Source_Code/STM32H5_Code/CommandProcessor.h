#ifndef COMMAND_PROCESSOR_H
#define COMMAND_PROCESSOR_H

#include <Arduino.h>

#include "MotorDriver.h"
#include "VehicleCommand.h"

class CommandProcessor
{

public:

    CommandProcessor();

    void begin(MotorDriver *motor);

    void update();

private:

    MotorDriver *driver;

    String rxBuffer;

    void processPacket(String packet);

};

#endif