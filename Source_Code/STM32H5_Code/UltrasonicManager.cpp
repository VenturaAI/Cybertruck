/******************************************************************************
 * UltrasonicManager.cpp
 *
 * HC-SR04 Driver for Arduino UNO Q STM32
 *
 * Current Implementation:
 *   - Front sensor fully implemented
 *   - Left and Right placeholders
 ******************************************************************************/

#include "UltrasonicManager.h"
#include "Config.h"

UltrasonicManager::UltrasonicManager()
{
    front_ = -1.0f;
    left_  = -1.0f;
    right_ = -1.0f;

    sensorIndex_ = 0;
    lastUpdate_ = 0;
}

void UltrasonicManager::begin()
{
    pinMode(FRONT_TRIG, OUTPUT);
    pinMode(FRONT_ECHO, INPUT);

    pinMode(LEFT_TRIG, OUTPUT);
    pinMode(LEFT_ECHO, INPUT);

    pinMode(RIGHT_TRIG, OUTPUT);
    pinMode(RIGHT_ECHO, INPUT);

    digitalWrite(FRONT_TRIG, LOW);
    digitalWrite(LEFT_TRIG, LOW);
    digitalWrite(RIGHT_TRIG, LOW);
}

void UltrasonicManager::update()
{
    const uint32_t now = millis();

    // Update one sensor every 50 ms
    if (now - lastUpdate_ < 50)
        return;

    lastUpdate_ = now;

    switch(sensorIndex_)
    {
        case 0:
           front_ = measure(FRONT_TRIG, FRONT_ECHO);
            break;

        case 1:
            // Enable after wiring left sensor
            // left_ = measure(LEFT_TRIG, LEFT_ECHO);
            break;

        case 2:
            // Enable after wiring right sensor
            // right_ = measure(RIGHT_TRIG, RIGHT_ECHO);
            break;
    }

    sensorIndex_++;

    if(sensorIndex_ > 2)
        sensorIndex_ = 0;
}

float UltrasonicManager::measure(uint8_t trigPin,
                                 uint8_t echoPin)
{
    digitalWrite(trigPin, LOW);
    delayMicroseconds(3);

    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    unsigned long duration =
        pulseIn(echoPin, HIGH, 30000);

    if(duration == 0)
    {
        //return -2.0f;
        return ULTRASONIC_MAX_CM;
    }

    float distance = duration * 0.0343f / 2.0f;

    if(distance > ULTRASONIC_MAX_CM)
        distance = ULTRASONIC_MAX_CM;

    return distance;
}

float UltrasonicManager::frontDistance() const
{
    return front_;
}

float UltrasonicManager::leftDistance() const
{
    return left_;
}

float UltrasonicManager::rightDistance() const
{
    return right_;
}