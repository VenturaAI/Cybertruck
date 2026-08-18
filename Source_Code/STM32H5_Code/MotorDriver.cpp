#include <Arduino.h>
#include <math.h>
#include "MotorDriver.h"
#include "Config.h"


//====================================================
// Constructor
//====================================================

MotorDriver::MotorDriver()
{
    steeringActive = false;
    steeringStartTime = 0;
    steeringDuration = 0;
    // Watchdog
    lastCommandTime = millis();
}


//====================================================
// Initialize GPIO
//====================================================

void MotorDriver::begin()
{

    pinMode(BTS_RPWM, OUTPUT);
    pinMode(BTS_LPWM, OUTPUT);

    pinMode(BTS_REN, OUTPUT);
    pinMode(BTS_LEN, OUTPUT);

    pinMode(STEER_PWM, OUTPUT);

    pinMode(STEER_IN1, OUTPUT);
    pinMode(STEER_IN2, OUTPUT);

    pinMode(STEER_STBY, OUTPUT);


    digitalWrite(BTS_REN, HIGH);
    digitalWrite(BTS_LEN, HIGH);

    digitalWrite(STEER_STBY, HIGH);


    stopDrive();

    stopSteering();
    lastCommandTime = millis();

}


//====================================================
// Update
//====================================================

void MotorDriver::update()
{
    //------------------------------------------------
    // Communication watchdog
    //------------------------------------------------

    if (millis() - lastCommandTime > COMMAND_TIMEOUT_MS)
    {
        emergencyStop();
        return;
    }

    //------------------------------------------------
    // Steering timeout
    //------------------------------------------------

    if (!steeringActive)
        return;

    if (millis() - steeringStartTime >= steeringDuration)
    {
        stopSteering();
    }
}


//====================================================
// Execute Vehicle Command
//====================================================

void MotorDriver::executeCommand(const VehicleCommand &cmd)
{
    // Refresh watchdog
    lastCommandTime = millis();
    if(cmd.brake)
    {

        brakeDrive();

        stopSteering();

        return;

    }
    // Debug Print
    Monitor.print(  "Speed: ");
    Monitor.print(cmd.speed);

    Monitor.print("  Steering: ");
    Monitor.print(cmd.steering);

    Monitor.print("  Brake: ");
    Monitor.println(cmd.brake);
    
    drive(cmd.speed);

    steer(cmd.steering);

}


//====================================================
// Drive Motors
//====================================================

void MotorDriver::drive(float speed)
{

    uint8_t pwm = speedToPWM(speed);

    if(speed > 0.05f)
    {

        analogWrite(BTS_RPWM,pwm);

        analogWrite(BTS_LPWM,0);

    }

    else if(speed < -0.05f)
    {

        analogWrite(BTS_RPWM,0);

        analogWrite(BTS_LPWM,pwm);

    }

    else
    {

        stopDrive();

    }

}


//====================================================
// Steering
//====================================================

void MotorDriver::steer(float steering)
{

    if(steeringActive)
        return;

    steeringDuration = steeringToPulse(steering);

    if(steeringDuration == 0)
        return;

    steeringStartTime = millis();

    steeringActive = true;


    if(steering < 0)
    {

        digitalWrite(STEER_IN1,HIGH);

        digitalWrite(STEER_IN2,LOW);

    }

    else
    {

        digitalWrite(STEER_IN1,LOW);

        digitalWrite(STEER_IN2,HIGH);

    }

    analogWrite(STEER_PWM,STEER_PWM_VALUE);

}


//====================================================
// Stop Drive
//====================================================

void MotorDriver::stopDrive()
{

    analogWrite(BTS_RPWM,0);

    analogWrite(BTS_LPWM,0);

}

// ===================================

// Brake Drive
//====================================
void MotorDriver::brakeDrive()
{

    analogWrite(BTS_RPWM, 255);

    analogWrite(BTS_LPWM, 255);

}



//====================================================
// Stop Steering
//====================================================

void MotorDriver::stopSteering()
{

    analogWrite(STEER_PWM,0);

    digitalWrite(STEER_IN1,LOW);

    digitalWrite(STEER_IN2,LOW);

    steeringActive = false;

}


//====================================================
// Emergency Stop
//====================================================

void MotorDriver::emergencyStop()
{

     // Active braking on BTS7960
    analogWrite(BTS_RPWM, 255);
    analogWrite(BTS_LPWM, 255);

    // Stop steering motor
    stopSteering();

}


//====================================================
// Speed → PWM
//====================================================

uint8_t MotorDriver::speedToPWM(float speed)
{

    speed = fabs(speed);

    if(speed < 0.05f)
        return PWM_STOP;

    if(speed > 1.0f)
        speed = 1.0f;

    return PWM_MIN +
           (uint8_t)(speed * (PWM_MAX - PWM_MIN));

}


//====================================================
// Steering → Pulse Time
//====================================================

uint16_t MotorDriver::steeringToPulse(float steering)
{

    steering = fabs(steering);

    if(steering < 0.10f)
        return 0;

    if(steering > 1.0f)
        steering = 1.0f;

    return STEER_SMALL +
           (uint16_t)(steering * (STEER_LARGE - STEER_SMALL));

}