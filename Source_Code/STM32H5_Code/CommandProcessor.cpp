#include <Arduino.h>

#include "CommandProcessor.h"

CommandProcessor::CommandProcessor()
{

    driver = nullptr;

    rxBuffer = "";

}

void CommandProcessor::begin(MotorDriver *motor)
{

    driver = motor;

    rxBuffer = "";

}

void CommandProcessor::update()
{

    while (Serial.available())
    {

        char c = Serial.read();

        if (c == '\r')
            continue;

        if (c == '\n')
        {

            processPacket(rxBuffer);

            rxBuffer = "";

        }
        else
        {

            rxBuffer += c;

        }

    }

}

void CommandProcessor::processPacket(String packet)
{

    packet.trim();

    if (!packet.startsWith("CMD"))
    {
        //Serial.println("Invalid Command");
        return;
    }

    int p1 = packet.indexOf(',');

    int p2 = packet.indexOf(',', p1 + 1);

    int p3 = packet.indexOf(',', p2 + 1);

    if (p1 < 0 || p2 < 0 || p3 < 0)
    {

        //Serial.println("Packet Error");

        return;

    }

    VehicleCommand cmd;

    cmd.speed =
        packet.substring(p1 + 1, p2).toFloat();

    cmd.steering =
        packet.substring(p2 + 1, p3).toFloat();

    cmd.brake =
        packet.substring(p3 + 1).toInt();

    driver->executeCommand(cmd);

}