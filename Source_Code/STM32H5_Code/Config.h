#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

/****************************************************
 * CyberTruck Autonomous Robot
 * Arduino UNO Q (STM32 Firmware)
 ****************************************************/


//====================================================
// SERIAL
//====================================================

constexpr uint32_t SERIAL_BAUD = 115200;


//====================================================
// BTS7960
// Drive Motor Driver
//====================================================

constexpr uint8_t BTS_RPWM = 5;
constexpr uint8_t BTS_LPWM = 6;

constexpr uint8_t BTS_REN  = 7;
constexpr uint8_t BTS_LEN  = 8;


//====================================================
// TB6612FNG
// Steering Motor Driver
//====================================================

constexpr uint8_t STEER_PWM  = 9;

constexpr uint8_t STEER_IN1  = 2;
constexpr uint8_t STEER_IN2  = 4;

constexpr uint8_t STEER_STBY = 3;


//====================================================
// DRIVE MOTOR PWM
// (Initial values - to be calibrated later)
//====================================================

constexpr uint8_t PWM_STOP   = 0;

constexpr uint8_t PWM_MIN    = 30;

constexpr uint8_t PWM_MAX    = 255;


//====================================================
// STEERING MOTOR PWM
//====================================================

constexpr uint8_t STEER_PWM_VALUE = 200;


//====================================================
// STEERING PULSE DURATIONS
// (milliseconds)
//====================================================

constexpr uint16_t STEER_SMALL = 100;

constexpr uint16_t STEER_MEDIUM = 180;

constexpr uint16_t STEER_LARGE = 300;


//====================================================
// COMMAND LIMITS
//====================================================

constexpr float MAX_FORWARD_SPEED = 1.0f;

constexpr float MAX_REVERSE_SPEED = -1.0f;

constexpr float MAX_LEFT_STEERING = -1.0f;

constexpr float MAX_RIGHT_STEERING = 1.0f;


//====================================================
// LOOP DELAY
//====================================================

constexpr uint16_t MAIN_LOOP_DELAY = 5;

//====================================================
// HC-SR04 Ultrasonic Sensors
//====================================================

// Front
constexpr uint8_t FRONT_TRIG = 10;
constexpr uint8_t FRONT_ECHO = 11;

// Left
constexpr uint8_t LEFT_TRIG = 12;
constexpr uint8_t LEFT_ECHO = 13;

// Right
constexpr uint8_t RIGHT_TRIG = A0;
constexpr uint8_t RIGHT_ECHO = A1;

// Maximum valid distance (cm)
constexpr float ULTRASONIC_MAX_CM = 400.0f;

#endif