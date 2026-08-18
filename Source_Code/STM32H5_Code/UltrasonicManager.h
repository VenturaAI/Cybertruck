#ifndef ULTRASONIC_MANAGER_H
#define ULTRASONIC_MANAGER_H

#include <Arduino.h>

class UltrasonicManager
{
public:

    UltrasonicManager();

    void begin();
    void update();

    float frontDistance() const;
    float leftDistance() const;
    float rightDistance() const;

private:

    float front_;
    float left_;
    float right_;

    uint8_t sensorIndex_;
    uint32_t lastUpdate_;

    float measure(uint8_t trigPin, uint8_t echoPin);
};

#endif