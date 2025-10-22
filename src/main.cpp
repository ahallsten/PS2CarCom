
/*----------------------------------------------------------------------*/
// Uncomment one of these to set the role of the device
// #define DEBUG
// #define TRANSMITTER
#define RECEIVER

#include <Adafruit_SleepyDog.h>
#include <LibPrintf.h>
#include <RH_RF95.h>
#include <Wire.h>
#include <avr/pgmspace.h>
#include "BTS7960.h"
#include "Joystick.h"
#include "MotorDriver.h"
#include "AxisMap.h"
typedef const __FlashStringHelper* FlashStr;
typedef const byte* PGM_BYTES_P;
#define PSTR_TO_F(s) reinterpret_cast<const __FlashStringHelper *>(s)

/*TRANSMITTER DEFINITIONS*/
/*----------------------------------------------------------------------*/
#ifdef TRANSMITTER
#include <DigitalIO.h>
#include <PsxControllerBitBang.h>
#include <SPI.h>
#endif

/*RECIEVER DEFINITIONS*/
/*----------------------------------------------------------------------*/
#ifdef RECEIVER
// Motor FL (Front Left) Pin Assignments
#define MOTOR_FL_RPWM 10 // 
#define MOTOR_FL_LPWM 11 // 
#define MOTOR_FL_REN A4
#define MOTOR_FL_LEN A5
#define MOTOR_FL_RIS A0
#define MOTOR_FL_LIS A1
// Motor FR (Front Right) Pin Assignments
#define MOTOR_FR_RPWM 6 //
#define MOTOR_FR_LPWM 9 // 
#define MOTOR_FR_REN 13
#define MOTOR_FR_LEN 12
#define MOTOR_FR_RIS A2
#define MOTOR_FR_LIS A3
// Motor FR (Front Right) Pin Assignments
#define MOTOR_RL_RPWM 6 //
#define MOTOR_RL_LPWM 9 // 
#define MOTOR_RL_REN 13
#define MOTOR_RL_LEN 12
#define MOTOR_RL_RIS A2
#define MOTOR_RL_LIS A3
// Motor FR (Front Right) Pin Assignments
#define MOTOR_RR_RPWM 6 //
#define MOTOR_RR_LPWM 9 // 
#define MOTOR_RR_REN 13
#define MOTOR_RR_LEN 12
#define MOTOR_RR_RIS A2
#define MOTOR_RR_LIS A3

// Other Pin Assignments
#define STEER_PWM 3

// BTS7960 (uint8_t RPWM,uint8_t LPWM,uint8_t L_EN,uint8_t R_EN)
BTS7960 mFL(MOTOR_FL_RPWM, MOTOR_FL_LPWM, MOTOR_FL_LEN, MOTOR_FL_REN, MOTOR_FL_RIS, MOTOR_FL_LIS);
BTS7960 mFR(MOTOR_FR_RPWM, MOTOR_FR_LPWM, MOTOR_FR_LEN, MOTOR_FR_REN, MOTOR_FR_RIS, MOTOR_FR_LIS);
BTS7960 mRL(MOTOR_RL_RPWM, MOTOR_RL_LPWM, MOTOR_RL_LEN, MOTOR_RL_REN, MOTOR_RL_RIS, MOTOR_RL_LIS);
BTS7960 mRR(MOTOR_RR_RPWM, MOTOR_RR_LPWM, MOTOR_RR_LEN, MOTOR_RR_REN, MOTOR_RR_RIS, MOTOR_RR_LIS);
// BTS7960 mRL.();
// BTS7960 mRR.();
#endif

/*TRANSMITTER & RECIEVER DEFINITIONS*/
/*----------------------------------------------------------------------*/
// Radio Pin Assignments
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 7

// Radio configuration parameters and statements
#define RF95_FREQ 915.0
RH_RF95 rfm(RFM95_CS, RFM95_INT);

struct ControllerState
{
    uint16_t buttonWord; // Stores the pressed buttons as a bitmask
    uint8_t leftX;       // Left joystick X-axis
    uint8_t leftY;       // Left joystick Y-axis
    uint8_t rightX;      // Right joystick X-axis
    uint8_t rightY;      // Right joystick Y-axis
};

ControllerState currentState = { 0, 0, 0, 0, 0 };
ControllerState previousState = { 0, 0, 0, 0, 0 };

enum Button
{
    SQUARE = 15,
    CROSS = 14,
    CIRCLE = 13,
    TRIANGLE = 12,
    R1 = 11,
    L1 = 10,
    R2 = 9,
    L2 = 8,
    LEFT = 7,
    RIGHT = 6,
    DOWN = 5,
    UP = 4,
    START = 3,
    R3 = 2,
    L3 = 1,
    SELECT = 0,
    UNKNOWN
};

/*TRANSMITTER & RECIEVER OBJECTS AND VARIABLES*/
/*----------------------------------------------------------------------*/

/*TRANSMITTER OBJECTS AND VARIABLES*/
/*----------------------------------------------------------------------*/
#ifdef TRANSMITTER
// These can be changed freely when using the bitbanged protocol
const byte PIN_PS2_ATT = 10;
const byte PIN_PS2_CMD = 11;
const byte PIN_PS2_DAT = 12;
const byte PIN_PS2_CLK = 13;

const byte PIN_BUTTONPRESS = A0;
const byte PIN_HAVECONTROLLER = A1;

// PS2 controller configuration
// PsxControllerHwSpi<10> psxCtrl;
PsxControllerBitBang<PIN_PS2_ATT, PIN_PS2_CMD, PIN_PS2_DAT, PIN_PS2_CLK> psx;
boolean haveController = false;

const char ctrlTypeUnknown[] PROGMEM = "Unknown";
const char ctrlTypeDualShock[] PROGMEM = "Dual Shock";
const char ctrlTypeDsWireless[] PROGMEM = "Dual Shock Wireless";
const char ctrlTypeGuitHero[] PROGMEM = "Guitar Hero";
const char ctrlTypeOutOfBounds[] PROGMEM = "(Out of bounds)";

const char* const controllerTypeStrings[PSCTRL_MAX + 1] PROGMEM = {
    ctrlTypeUnknown, ctrlTypeDualShock, ctrlTypeDsWireless, ctrlTypeGuitHero,
    ctrlTypeOutOfBounds };
#endif

/*RECIEVER OBJECTS AND VARIABLES*/
/*----------------------------------------------------------------------*/
// #ifdef RECEIVER
// Global variables for the controlling the Motors and steering
bool parking_Brake = 0;
bool enableToggle = 0;    // Toggle for enabling/disabling the car
bool enableAMotorFwd = 0; // Enable right motor
bool enableAMotorRev = 0; // Enable left motor
bool enableBMotorFwd = 0; // Enable right motor
bool enableBMotorRev = 0; // Enable left motor
bool tankMode = 0;        // Tank mode allows for independent control of left and right motors
bool leftYForward = 0;    // bit to represent direction of left Y analog stick
bool rightYForward = 0;   // bit to represent direction of right Y analog stick
byte pwmValueLY = 0;      // PWM value for the motors
byte pwmValueLX = 0;      // PWM value for the motors
byte pwmValueRY = 0;      // PWM value for the motors
byte pwmValueRX = 0;      // PWM value for the motors
byte AMotorPWMValue = 0;  // PWM value for the left motor
byte BMotorPWMValue = 0;  // PWM value for the right motor
byte steerPwmValue = 0;   // PWM value for the steering servo
int maximumSpeed = 0;     // Maximum speed of the car
int minimumSpeed = 25;    // Minimum speed of the car
int zeroCalibration = 0;  // Zero calibration for the steering servo
// #endif

/*TRANSMITTER & RECEIVER FUNCTIONS*/
/*----------------------------------------------------------------------*/
// byte psxButtonToIndex(PsxButtons psxButtons) {
//   byte i;
//   for (i = 0; i < PSX_BUTTONS_NO; ++i) {
//     if (psxButtons & 0x01) {
//       break;
//     }
//     psxButtons >>= 1U;
//   }
//   return i;
// }
// FlashStr getbit(PsxButtons psxButton) {
//   FlashStr ret = F("");
//   byte b = psxButtonToIndex(psxButton);
//   if (b < PSX_BUTTONS_NO) {
//     PGM_BYTES_P bName =
//         reinterpret_cast<PGM_BYTES_P>(pgm_read_ptr(&(psxbits[b])));
//     ret = PSTR_TO_F(bName);
//     Serial.print("btn name: ");
//     Serial.println(ret);
//   }
//   return ret;
// }

// Function to check if the state has changed
bool stateChanged(const ControllerState& current,
    const ControllerState& previous)
{
    return current.buttonWord != previous.buttonWord ||
        current.leftX != previous.leftX || current.leftY != previous.leftY ||
        current.rightX != previous.rightX || current.rightY != previous.rightY;
}
// Function to print the current state of the controller
void printControllerStruct()
{
    // Serial.print(F("ButtonWord: "));
    // printf("%016b", currentState.buttonWord);
    // Serial.print(F("| LX: "));
    // Serial.print(currentState.leftX);
    // Serial.print(F("| LY: "));
    // Serial.print(currentState.leftY);
    // Serial.print(F("| RX: "));
    // Serial.print(currentState.rightX);
    // Serial.print(F("| RY: "));
    // Serial.print(currentState.rightY);
    Serial.print(F("| pwmLY "));
    Serial.print(pwmValueLY);
    Serial.print(F("| pwmRY "));
    Serial.print(pwmValueRY);
    Serial.print(F("| pwmSTR "));
    Serial.print(steerPwmValue);
    Serial.print(F("| ENAF "));
    Serial.print(enableAMotorFwd);
    Serial.print(F("| ENAR "));
    Serial.print(enableAMotorRev);
    Serial.print(F("| ENBF "));
    Serial.print(enableBMotorFwd);
    Serial.print(F("| ENBR "));
    Serial.print(enableBMotorRev);
    Serial.print(F("| EN: "));
    Serial.print(enableToggle ? F("ON") : F("OFF"));
    Serial.print(F("| T_MD: "));
    Serial.print(tankMode ? F("ON") : F("OFF"));
    Serial.print(F("| p_Brk: "));
    Serial.println(parking_Brake ? F("ON") : F("OFF"));
}

/*TRANSMITTER FUNCTIONS*/
/*----------------------------------------------------------------------*/
#ifdef TRANSMITTER
// void dumpButtons(PsxButtons psxButtons) {
//   static PsxButtons lastB = 0;
//   if (psxButtons != lastB) {
//     lastB = psxButtons;  // Save it before we alter it
//     Serial.print(F("Pressed: "));
//     for (byte i = 0; i < PSX_BUTTONS_NO; ++i) {
//       byte b = psxButtonToIndex(psxButtons);
//       if (b < PSX_BUTTONS_NO) {
//         PGM_BYTES_P bName =
//             reinterpret_cast<PGM_BYTES_P>(pgm_read_ptr(&(psxbits[b])));
//         Serial.print(PSTR_TO_F(bName));
//       }
//       psxButtons &= ~(1 << b);
//       if (psxButtons != 0) {
//         Serial.print(F(", "));
//       }
//     }
//     Serial.println();
//   }
// }
// Function to update the current state from the PSX controller
void updateCurrentState()
{
    currentState.buttonWord = psx.getButtonWord();
    psx.getLeftAnalog(currentState.leftX, currentState.leftY);
    psx.getRightAnalog(currentState.rightX, currentState.rightY);
}

void dumpAnalog(const char* str, const byte x, const byte y)
{
    Serial.print(str);
    Serial.print(F(" analog: x = "));
    Serial.print(x);
    Serial.print(F(", y = "));
    Serial.println(y);
}

void sendStateIfChanged()
{
    if (stateChanged(currentState, previousState))
    {
        // Serialize the struct into a byte array
        uint8_t packet[sizeof(ControllerState)];
        memcpy(packet, &currentState, sizeof(ControllerState));
        // Send the packet via RFM9x
        rfm.send(packet, sizeof(packet));
        // Output the button word and joystick values
        printControllerStruct();
        // Update the previous state
        previousState = currentState;
    }
}

void PS2ControllerCheck()
{
    if (!haveController)
    {
        if (psx.begin())
        {
            Serial.println(F("Controller found!"));
            delay(300);
            if (!psx.enterConfigMode())
            {
                Serial.println(F("Cannot enter config mode"));
            }
            else
            {
                PsxControllerType ctype = psx.getControllerType();
                PGM_BYTES_P cname = reinterpret_cast<PGM_BYTES_P>(pgm_read_ptr(&(
                    controllerTypeStrings[ctype < PSCTRL_MAX ? static_cast<byte>(ctype)
                    : PSCTRL_MAX])));
                Serial.print(F("Controller Type is: "));
                Serial.println(PSTR_TO_F(cname));

                if (!psx.enableAnalogSticks())
                {
                    Serial.println(F("Cannot enable analog sticks"));
                }

                if (!psx.enableAnalogButtons())
                {
                    Serial.println(F("Cannot enable analog buttons"));
                }

                if (!psx.exitConfigMode())
                {
                    Serial.println(F("Cannot exit config mode"));
                }
            }

            haveController = true;
        }
    }
    else
    {
        if (!psx.read())
        {
            Serial.println(F("Controller lost :("));
            haveController = false;
        }
        else
        {
            updateCurrentState();
            sendStateIfChanged();

            // // Optional: Debugging outputs
            // fastDigitalWrite(PIN_BUTTONPRESS, !!psx.getButtonWord());
            // dumpButtons(psx.getButtonWord());
        }
    }
}
#endif

/*RECEIVER FUNCTIONS*/
/*----------------------------------------------------------------------*/
#ifdef RECEIVER
Button getButtonEnum(uint8_t bit)
{
    if (bit == 0)
        return SELECT;
    if (bit == 1)
        return L3;
    if (bit == 2)
        return R3;
    if (bit == 3)
        return START;
    if (bit == 4)
        return UP;
    if (bit == 5)
        return RIGHT;
    if (bit == 6)
        return DOWN;
    if (bit == 7)
        return LEFT;
    if (bit == 8)
        return L2;
    if (bit == 9)
        return R2;
    if (bit == 10)
        return L1;
    if (bit == 11)
        return R1;
    if (bit == 12)
        return TRIANGLE;
    if (bit == 13)
        return CIRCLE;
    if (bit == 14)
        return CROSS;
    if (bit == 15)
        return SQUARE;
    return UNKNOWN;
}
void handleButtonPress(uint8_t bit)
{
    switch (getButtonEnum(bit))
    {
    case SELECT:
        parking_Brake = !parking_Brake; // Toggle parking brake
        Serial.print(F("SELECT->Parking Brake: "));
        Serial.println(parking_Brake ? F("ON") : F("OFF"));
        break;
    case START:
        tankMode = !tankMode; // Toggle tank mode
        Serial.print(F("START->Tank mode: "));
        Serial.println(tankMode ? F("ON") : F("OFF"));
        break;
    case UP:
        if (maximumSpeed < 250)
        {
            maximumSpeed += 25;
        }
        else
        {
            maximumSpeed = 250; // Prevent negative speed
        }
        Serial.print(F("UP->Maximum Speed: "));
        Serial.println(maximumSpeed);
        break;
    case DOWN:
        if (maximumSpeed > 25)
        {
            maximumSpeed -= 25;
        }
        else
        {
            maximumSpeed = minimumSpeed; // Prevent negative speed
        }
        Serial.print(F("DOWN->Maximum Speed: "));
        Serial.println(maximumSpeed);
        break;
    case LEFT:
        maximumSpeed = minimumSpeed; // Quick set to 0 speed
        Serial.print(F("LEFT->MaximumSpeed: "));
        Serial.println(maximumSpeed);
        break;
    case RIGHT:
        maximumSpeed = 250; // Quick set to 0 speed
        Serial.print(F("RIGHT->MaximumSpeed: "));
        Serial.println(maximumSpeed);
        break;
    case TRIANGLE:
        Serial.println(F("Triangle button pressed!"));
        break;
    case CIRCLE:
        Serial.println(F("Circle button pressed!"));
        break;
    case CROSS:
        Serial.println(F("Cross button pressed!"));
        break;
    case SQUARE:
        Serial.println(F("Square button pressed!"));
        break;
    case L1:
        enableAMotorFwd = 1;
        enableAMotorRev = 1;
        Serial.println(F("L1 button pressed!"));
        break;
    case R1:
        enableBMotorFwd = 1;
        enableBMotorRev = 1;
        Serial.println(F("R1 button pressed!"));
        break;
    case L2:
        Serial.println(F("L2 button pressed!"));
        break;
    case R2:
        enableToggle = !enableToggle;
        Serial.print(F("R2->Enable Toggle: "));
        Serial.println(enableToggle ? F("ON") : F("OFF"));
        break;
    case L3:
        Serial.println(F("L3 button pressed!"));
        break;
    case R3:
        Serial.println(F("R3 button pressed!"));
        break;
    default:
        Serial.println(F("FUCK!"));
        break;
    }
}
void onRisingEdge(uint8_t bit)
{
    // Serial.print("Rising edge on bit ");
    // Serial.println(bit);
    // Example: special case for bit 3
    handleButtonPress(bit);
}
void onFallingEdge(uint8_t bit)
{
    // Serial.print("Falling edge on bit ");
    // Serial.println(bit);
    // Example: special case for bit 7
    handleButtonPress(bit);
}
void onBitState(uint8_t bit, bool state)
{
    // char buffer[20];
    // sprintf(buffer, "Bit %d is %s", bit, state ? "HIGH" : "LOW");
    // Serial.println(buffer);

    // Optional: act on steady state (high or low)
    if (bit == 12 && state)
    {
        // do something when bit 12 is high
    }
}
void detectButtonWordStateChange(uint16_t buttonWord)
{
    uint16_t previousButtonWord; // Initialize previous state for buttonWord
    const int NUM_BITS = 16;     // Number of bits in the word
    for (int bit = 0; bit < NUM_BITS; bit++)
    {
        bool currentBit = (buttonWord >> bit) & 0x01;
        bool previousBit = (previousButtonWord >> bit) & 0x01;

        if (!previousBit && currentBit)
        {
            onRisingEdge(bit);
        }
        else if (previousBit && !currentBit)
        {
            onFallingEdge(bit);
        }
        onBitState(bit, currentBit); // always called
    }
    previousButtonWord = buttonWord; // Update the previous state
}
void receivePacket()
{
    if (rfm.available())
    {
        uint8_t buffer[sizeof(ControllerState)];
        uint8_t len = sizeof(buffer);

        // Receive the packet
        rfm.recv(buffer, &len);

        // Deserialize into a struct
        memcpy(&currentState, buffer, sizeof(ControllerState));

        // Output the button word and joystick values
        printControllerStruct();
    }
}
// void writePins()
// {
//     // Write the PWM value to the desired pin
//     analogWrite(STEER_PWM, steerPwmValue);
//     // if (tankMode)
//     // { // Control A Motor: Left Joystick Y-axis, B Motor: Right
//     // }
//     // else
//     // { // Control both motors with the left joystick Y-axis
//     // }
// }
// byte pwmOutput(byte stickValue, byte deadHigh, byte deadLow) {
//   if (stickValue < deadLow) {
//     return map(stickValue, deadLow, 0, 0, 255);
//   } else if (stickValue > deadHigh) {
//     return map(stickValue, deadHigh, 255, 0, 255);
//   } else {
//     return pwmValue;  // Dead zone: set PWM to 0
//   }
// }
void controlsDecision()
{
    if (stateChanged(currentState, previousState))
    {
        previousState = currentState; // Update the previous state
        detectButtonWordStateChange(currentState.buttonWord); // Detect button word state change

        // Mapping profiles
        const AxisMap leftYMap = { 110, 110, 0, 145, 145, 255, 0, 255 };
        // const AxisMap leftXMap = { 110, 110, 0, 145, 145, 255, 0, 255 };
        // const AxisMap rightYMap = { 106, 106, 0, 133, 133, 255, 0, 255 };
        const AxisMap rightXMap = { 106, 106, 0, 133, 133, 255, 0, 255 };

        int16_t leftPWM = mapAxisSigned(currentState.leftY, leftYMap);
        // int16_t leftPWM = mapAxisSigned(currentState.leftX, leftXMap);
        // int16_t rightPWM = mapAxisSigned(currentState.rightY, rightYMap);
        int16_t rightPWM = mapAxisSigned(currentState.rightX, rightXMap);

        if (parking_Brake)
        {
            pwmValueLY = 0;          // Set PWM to 0 when parking brake is on
            pwmValueRY = 0;          // Set PWM to 0 when parking brake is on
            enableAMotorFwd = false; // Disable left motor forward
            enableBMotorFwd = false; // Disable right motor forward
            enableAMotorRev = false; // Disable left motor reverse
            enableBMotorRev = false; // Disable right motor reverse
        }
        else
        {
            // restrict the max speed set to keep the kids safe
            map(pwmValueLY, 0, 255, 0, maximumSpeed);
            map(pwmValueRY, 0, 255, 0, maximumSpeed);
        }
        if (enableToggle)
        {
            mFL.enable();
            mFR.enable();
            mRL.enable();
            mRR.enable();
        }
        else
        {
            mFL.brake();
            mRL.brake();
            mRR.brake();
            mFR.brake();
        }

        // writePins(); // Write the motor control pins based on the joystick values
        mFL.drive(leftPWM);
        mFR.drive(leftPWM);
        mRL.drive(leftPWM);
        mRR.drive(leftPWM);
    }
}
#endif

/*DEBUGGIN FUNCTIONS*/
/*----------------------------------------------------------------------*/
#ifdef DEBUG
// Debugging functions for testing purposes
#endif

/*TRANSMITTER SETUP AND LOOP*/
/*----------------------------------------------------------------------*/
#ifdef TRANSMITTER

void setup()
{
    // Initialize serial monitor
    Serial.begin(115200);

    // Initialize RFM95
    pinMode(RFM95_RST, OUTPUT);
    digitalWrite(RFM95_RST, HIGH);
    if (!rfm.init())
    {
        Serial.println("RFM95 initialization failed");
        while (1)
            ;
    }
    rfm.setFrequency(915.0);
    Serial.println("RFM95 initialized: TRANSMITTER");

    // Debug pins for having controller and button presses
    fastPinMode(PIN_BUTTONPRESS, OUTPUT);
    fastPinMode(PIN_HAVECONTROLLER, OUTPUT);

    delay(300);
}

void loop()
{
    // Check if the controller is connected
    PS2ControllerCheck();
    delay(1000 / 140); // 140 Hz update rate
}
#endif

/*RECEIVER SETUP AND LOOP*/
/*----------------------------------------------------------------------*/
#ifdef RECEIVER
void setup()
{
    // Initialize serial monitor
    Serial.begin(115200);

    // Initialize RFM95
    pinMode(RFM95_RST, OUTPUT);
    digitalWrite(RFM95_RST, HIGH);

    if (!rfm.init())
    {
        Serial.println("RFM95 initialization failed");
        while (1)
            ;
    }
    rfm.setFrequency(915.0);
    Serial.println("RFM95 initialized: RECEIVER");

    // enable the BTS7960 Motordriver
    mFL.begin();
    mFR.begin();
    mRL.begin();
    mRR.begin();
}

void loop()
{
    receivePacket();
    controlsDecision();
}
#endif