#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <Arduino.h>

// MotorDriver encapsulates the BTS7960 motor drivers and servo control pins.
// It provides an interface to configure the pins and to apply PWM and direction
// signals to the motors and steering servo.
class MotorDriver {
public:
    MotorDriver();

    // Configure all the motor and servo pins as inputs or outputs and set initial states.
    void begin();

    // Apply the given PWM and enable values to the motors and servo.
    // If tankMode is true, the left and right motors are driven independently by pwmLeft and pwmRight.
    // Otherwise both motors are driven by pwmLeft.
    void apply(bool tankMode,
               byte pwmLeft,
               byte pwmRight,
               byte steerPwm,
               bool enableAMotorFwd,
               bool enableAMotorRev,
               bool enableBMotorFwd,
               bool enableBMotorRev);

private:
    // Pin constants for the motor driver and steering servo.
    static const uint8_t MOTORA_RPWM;
    static const uint8_t MOTORA_LPWM;
    static const uint8_t MOTORA_REN;
    static const uint8_t MOTORA_LEN;
    static const uint8_t MOTORA_RIS;
    static const uint8_t MOTORA_LIS;

    static const uint8_t MOTORB_RPWM;
    static const uint8_t MOTORB_LPWM;
    static const uint8_t MOTORB_REN;
    static const uint8_t MOTORB_LEN;
    static const uint8_t MOTORB_RIS;
    static const uint8_t MOTORB_LIS;

    static const uint8_t STEER_PWM;
};

#endif // MOTOR_DRIVER_H