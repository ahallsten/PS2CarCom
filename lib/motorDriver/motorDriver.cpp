#include "MotorDriver.h"

// Define the static constants for the motor and servo pins.
const uint8_t MotorDriver::MOTORA_RPWM = 9;
const uint8_t MotorDriver::MOTORA_LPWM = 6;
const uint8_t MotorDriver::MOTORA_REN  = A4;
const uint8_t MotorDriver::MOTORA_LEN  = A5;
const uint8_t MotorDriver::MOTORA_RIS  = A0;
const uint8_t MotorDriver::MOTORA_LIS  = A1;

const uint8_t MotorDriver::MOTORB_RPWM = 11;
const uint8_t MotorDriver::MOTORB_LPWM = 10;
const uint8_t MotorDriver::MOTORB_REN  = 13;
const uint8_t MotorDriver::MOTORB_LEN  = 12;
const uint8_t MotorDriver::MOTORB_RIS  = A2;
const uint8_t MotorDriver::MOTORB_LIS  = A3;

const uint8_t MotorDriver::STEER_PWM   = 3;

MotorDriver::MotorDriver() {}

void MotorDriver::begin() {
    // Configure and initialize motor A pins
    pinMode(MOTORA_REN, OUTPUT);
    pinMode(MOTORA_LEN, OUTPUT);
    pinMode(MOTORA_RPWM, OUTPUT);
    pinMode(MOTORA_LPWM, OUTPUT);
    pinMode(MOTORA_RIS, INPUT);
    pinMode(MOTORA_LIS, INPUT);
    digitalWrite(MOTORA_REN, LOW);
    digitalWrite(MOTORA_LEN, LOW);
    digitalWrite(MOTORA_RPWM, LOW);
    digitalWrite(MOTORA_LPWM, LOW);

    // Configure and initialize motor B pins
    pinMode(MOTORB_REN, OUTPUT);
    pinMode(MOTORB_LEN, OUTPUT);
    pinMode(MOTORB_RPWM, OUTPUT);
    pinMode(MOTORB_LPWM, OUTPUT);
    pinMode(MOTORB_RIS, INPUT);
    pinMode(MOTORB_LIS, INPUT);
    digitalWrite(MOTORB_REN, LOW);
    digitalWrite(MOTORB_LEN, LOW);
    digitalWrite(MOTORB_RPWM, LOW);
    digitalWrite(MOTORB_LPWM, LOW);
}

void MotorDriver::apply(bool tankMode,
                        byte pwmLeft,
                        byte pwmRight,
                        byte steerPwm,
                        bool enableAMotorFwd,
                        bool enableAMotorRev,
                        bool enableBMotorFwd,
                        bool enableBMotorRev) {
    // Set the steering servo PWM value
    analogWrite(STEER_PWM, steerPwm);
    // Apply motor PWM values
    if (tankMode) {
        // Left motor (A) uses pwmLeft, Right motor (B) uses pwmRight
        analogWrite(MOTORA_RPWM, pwmLeft);
        analogWrite(MOTORA_LPWM, pwmLeft);
        analogWrite(MOTORB_RPWM, pwmRight);
        analogWrite(MOTORB_LPWM, pwmRight);
    } else {
        // Both motors driven by the same PWM (pwmLeft)
        analogWrite(MOTORA_RPWM, pwmLeft);
        analogWrite(MOTORB_RPWM, pwmLeft);
        analogWrite(MOTORA_LPWM, pwmLeft);
        analogWrite(MOTORB_LPWM, pwmLeft);
    }
    // Apply motor direction and enable flags
    digitalWrite(MOTORA_REN, enableAMotorFwd ? HIGH : LOW);
    digitalWrite(MOTORA_LEN, enableAMotorRev ? HIGH : LOW);
    digitalWrite(MOTORB_REN, enableBMotorFwd ? HIGH : LOW);
    digitalWrite(MOTORB_LEN, enableBMotorRev ? HIGH : LOW);
}