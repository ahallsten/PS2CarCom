
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
typedef const __FlashStringHelper *FlashStr;
typedef const byte *PGM_BYTES_P;
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
// Motor A Pin Assignments
#define MOTORA_RPWM 9
#define MOTORA_LPWM 6
#define MOTORA_REN A4
#define MOTORA_LEN A5
#define MOTORA_RIS A0
#define MOTORA_LIS A1
// Motor B Pin Assignments
#define MOTORB_RPWM 11
#define MOTORB_LPWM 10
#define MOTORB_REN 13
#define MOTORB_LEN 12
#define MOTORB_RIS A2
#define MOTORB_LIS A3
// Other Pin Assignments
#define STEER_PWM 3
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

struct ControllerState {
  uint16_t buttonWord;  // Stores the pressed buttons as a bitmask
  uint8_t leftX;        // Left joystick X-axis
  uint8_t leftY;        // Left joystick Y-axis
  uint8_t rightX;       // Right joystick X-axis
  uint8_t rightY;       // Right joystick Y-axis
};

ControllerState currentState = {0, 0, 0, 0, 0};
ControllerState previousState = {0, 0, 0, 0, 0};

enum Button {
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

const char *const controllerTypeStrings[PSCTRL_MAX + 1] PROGMEM = {
    ctrlTypeUnknown, ctrlTypeDualShock, ctrlTypeDsWireless, ctrlTypeGuitHero,
    ctrlTypeOutOfBounds};
#endif

/*RECIEVER OBJECTS AND VARIABLES*/
/*----------------------------------------------------------------------*/
#ifdef RECEIVER
// Global variables for the controlling the Motors and steering
bool parking_Brake = 0;
bool enableToggle = 0;     // Toggle for enabling/disabling the car
bool enableAMotorFwd = 0;  // Enable right motor
bool enableAMotorRev = 0;  // Enable left motor
bool enableBMotorFwd = 0;  // Enable right motor
bool enableBMotorRev = 0;  // Enable left motor
bool tankMode = 0;         // Tank mode allows for independent control of left and right motors
bool leftYForward = 0;     // bit to represent direction of left Y analog stick
bool rightYForward = 0;    // bit to represent direction of right Y analog stick
byte pwmValueLY = 0;       // PWM value for the motors
byte pwmValueLX = 0;       // PWM value for the motors
byte pwmValueRY = 0;       // PWM value for the motors
byte pwmValueRX = 0;       // PWM value for the motors
byte AMotorPWMValue = 0;   // PWM value for the left motor
byte BMotorPWMValue = 0;   // PWM value for the right motor
byte steerPwmValue = 0;    // PWM value for the steering servo
int maximumSpeed = 0;      // Maximum speed of the car
int minimumSpeed = 25;     // Minimum speed of the car
int zeroCalibration = 0;   // Zero calibration for the steering servo
#endif

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
bool stateChanged(const ControllerState &current,
                  const ControllerState &previous) {
  return current.buttonWord != previous.buttonWord ||
         current.leftX != previous.leftX || current.leftY != previous.leftY ||
         current.rightX != previous.rightX || current.rightY != previous.rightY;
}
// Function to print the current state of the controller
void printControllerStruct() {
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
void updateCurrentState() {
  currentState.buttonWord = psx.getButtonWord();
  psx.getLeftAnalog(currentState.leftX, currentState.leftY);
  psx.getRightAnalog(currentState.rightX, currentState.rightY);
}

void dumpAnalog(const char *str, const byte x, const byte y) {
  Serial.print(str);
  Serial.print(F(" analog: x = "));
  Serial.print(x);
  Serial.print(F(", y = "));
  Serial.println(y);
}

void sendStateIfChanged() {
  if (stateChanged(currentState, previousState)) {
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

void PS2ControllerCheck() {
  if (!haveController) {
    if (psx.begin()) {
      Serial.println(F("Controller found!"));
      delay(300);
      if (!psx.enterConfigMode()) {
        Serial.println(F("Cannot enter config mode"));
      } else {
        PsxControllerType ctype = psx.getControllerType();
        PGM_BYTES_P cname = reinterpret_cast<PGM_BYTES_P>(pgm_read_ptr(&(
            controllerTypeStrings[ctype < PSCTRL_MAX ? static_cast<byte>(ctype)
                                                     : PSCTRL_MAX])));
        Serial.print(F("Controller Type is: "));
        Serial.println(PSTR_TO_F(cname));

        if (!psx.enableAnalogSticks()) {
          Serial.println(F("Cannot enable analog sticks"));
        }

        if (!psx.enableAnalogButtons()) {
          Serial.println(F("Cannot enable analog buttons"));
        }

        if (!psx.exitConfigMode()) {
          Serial.println(F("Cannot exit config mode"));
        }
      }

      haveController = true;
    }
  } else {
    if (!psx.read()) {
      Serial.println(F("Controller lost :("));
      haveController = false;
    } else {
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
Button getButtonEnum(uint8_t bit) {
  if (bit == 0) return SELECT;
  if (bit == 1) return L3;
  if (bit == 2) return R3;
  if (bit == 3) return START;
  if (bit == 4) return UP;
  if (bit == 5) return RIGHT;
  if (bit == 6) return DOWN;
  if (bit == 7) return LEFT;
  if (bit == 8) return L2;
  if (bit == 9) return R2;
  if (bit == 10) return L1;
  if (bit == 11) return R1;
  if (bit == 12) return TRIANGLE;
  if (bit == 13) return CIRCLE;
  if (bit == 14) return CROSS;
  if (bit == 15) return SQUARE;
  return UNKNOWN;
}
void handleButtonPress(uint8_t bit) {
  switch (getButtonEnum(bit)) {
    case SELECT:
      parking_Brake = !parking_Brake;  // Toggle parking brake
      Serial.print(F("SELECT->Parking Brake: "));
      Serial.println(parking_Brake ? F("ON") : F("OFF"));
      break;
    case START:
      tankMode = !tankMode;  // Toggle tank mode
      Serial.print(F("START->Tank mode: "));
      Serial.println(tankMode ? F("ON") : F("OFF"));
      break;
    case UP:
      if (maximumSpeed < 250) {
        maximumSpeed += 25;
      } else {
        maximumSpeed = 250;  // Prevent negative speed
      }
      Serial.print(F("UP->Maximum Speed: "));
      Serial.println(maximumSpeed);
      break;
    case DOWN:
      if (maximumSpeed > 25) {
        maximumSpeed -= 25;
      } else {
        maximumSpeed = minimumSpeed;  // Prevent negative speed
      }
      Serial.print(F("DOWN->Maximum Speed: "));
      Serial.println(maximumSpeed);
      break;
    case LEFT:
      maximumSpeed = minimumSpeed;  // Quick set to 0 speed
      Serial.print(F("LEFT->MaximumSpeed: "));
      Serial.println(maximumSpeed);
      break;
    case RIGHT:
      maximumSpeed = 250;  // Quick set to 0 speed
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
void onRisingEdge(uint8_t bit) {
  // Serial.print("Rising edge on bit ");
  // Serial.println(bit);
  // Example: special case for bit 3
  handleButtonPress(bit);
}
void onFallingEdge(uint8_t bit) {
  // Serial.print("Falling edge on bit ");
  // Serial.println(bit);
  // Example: special case for bit 7
  handleButtonPress(bit);
}
void onBitState(uint8_t bit, bool state) {
  // char buffer[20];
  // sprintf(buffer, "Bit %d is %s", bit, state ? "HIGH" : "LOW");
  // Serial.println(buffer);

  // Optional: act on steady state (high or low)
  if (bit == 12 && state) {
    // do something when bit 12 is high
  }
}
void detectButtonWordStateChange(uint16_t buttonWord) {
  uint16_t previousButtonWord;  // Initialize previous state for buttonWord
  const int NUM_BITS = 16;      // Number of bits in the word
  for (int bit = 0; bit < NUM_BITS; bit++) {
    bool currentBit = (buttonWord >> bit) & 0x01;
    bool previousBit = (previousButtonWord >> bit) & 0x01;

    if (!previousBit && currentBit) {
      onRisingEdge(bit);
    } else if (previousBit && !currentBit) {
      onFallingEdge(bit);
    }
    onBitState(bit, currentBit);  // always called
  }
  previousButtonWord = buttonWord;  // Update the previous state
}
void receivePacket() {
  if (rfm.available()) {
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
void writePins() {
  // Write the PWM value to the desired pin
  analogWrite(STEER_PWM, steerPwmValue);
  if (tankMode) {  // Control A Motor: Left Joystick Y-axis, B Motor: Right
                   // Joystick Y-axis
    analogWrite(MOTORA_RPWM, pwmValueLY);
    analogWrite(MOTORB_RPWM, pwmValueRY);
    analogWrite(MOTORA_LPWM, pwmValueLY);
    analogWrite(MOTORB_LPWM, pwmValueRY);
  } else {  // Control both motors with the left joystick Y-axis
    analogWrite(MOTORA_RPWM, pwmValueLY);
    analogWrite(MOTORB_RPWM, pwmValueLY);
    analogWrite(MOTORA_LPWM, pwmValueLY);
    analogWrite(MOTORB_LPWM, pwmValueLY);
  }
  digitalWrite(MOTORA_REN, enableAMotorFwd ? HIGH : LOW);
  digitalWrite(MOTORA_LEN, enableAMotorRev ? HIGH : LOW);
  digitalWrite(MOTORB_REN, enableBMotorFwd ? HIGH : LOW);
  digitalWrite(MOTORB_LEN, enableBMotorRev ? HIGH : LOW);
}
// byte pwmOutput(byte stickValue, byte deadHigh, byte deadLow) {
//   if (stickValue < deadLow) {
//     return map(stickValue, deadLow, 0, 0, 255);
//   } else if (stickValue > deadHigh) {
//     return map(stickValue, deadHigh, 255, 0, 255);
//   } else {
//     return pwmValue;  // Dead zone: set PWM to 0
//   }
// }
void controlsDecision() {
  if (stateChanged(currentState, previousState)) {
    previousState = currentState;  // Update the previous state
    detectButtonWordStateChange(
        currentState.buttonWord);  // Detect button word state change

    // Left Joystick Y-> Up:0 Down:255 control both motor speeds or just the
    if (currentState.leftY < 110) {
      pwmValueLY = map(currentState.leftY, 110, 0, 0, 255);
      leftYForward = true;
    } else if (currentState.leftY > 145) {
      pwmValueLY = map(currentState.leftY, 145, 255, 0, 255);
      leftYForward = false;
    } else {
      pwmValueLY = 0;  // Dead zone: set PWM to 0
    }

    // Left Joystick X-> Left:0 Right:255 Reserved for future use
    // if (currentState.leftX < 90) {
    //   pwmValueLX = map(currentState.leftX, 90, 0, 0, 255);
    //   forward = true;
    // } else if (currentState.leftX > 145) {
    //   pwmValueLX = map(currentState.leftX, 145, 255, 0, 255);
    //   forward = false;
    // } else {
    //   pwmValueLX = 0;  // Dead zone: set PWM to 0
    // }

    // Right Joystick Y-> Up:0 Down:255 control for right motor speeds in tank
    // mode
    if (currentState.rightY < 106) {
      pwmValueRY = map(currentState.rightY, 106, 0, 0, 255);
      rightYForward = true;
    } else if (currentState.rightY > 133) {
      pwmValueRY = map(currentState.rightY, 133, 255, 0, 255);
      rightYForward = false;
    } else {
      pwmValueRY = 0;  // Dead zone: set PWM to 0
    }

    // Right Joystick X-> Left:0 Right:255 to control the steering Servo
    if (currentState.rightX > 140) {
      steerPwmValue = map(currentState.rightX, 135, 255, 0, 255);
    } else if (currentState.rightX < 110) {
      steerPwmValue = map(currentState.rightX, 105, 0, 0, 255);
    } else {
      steerPwmValue = 0;  // Dead zone: set PWM to 0
    }

    if (parking_Brake) {
      pwmValueLY = 0;           // Set PWM to 0 when parking brake is on
      pwmValueRY = 0;           // Set PWM to 0 when parking brake is on
      enableAMotorFwd = false;  // Disable left motor forward
      enableBMotorFwd = false;  // Disable right motor forward
      enableAMotorRev = false;  // Disable left motor reverse
      enableBMotorRev = false;  // Disable right motor reverse
    } else {
      // pwmValueLY = constrain(pwmValueLY, minimumSpeed, maximumSpeed);
      // pwmValueRY = constrain(pwmValueRY, minimumSpeed, maximumSpeed);
      map(pwmValueLY, 0, 255, 0, maximumSpeed);  // Map the PWM value to the maximum speed
      map(pwmValueRY, 0, 255, 0, maximumSpeed);  // Map the PWM value to the maximum speed
    }
    if (enableToggle) {
      if (leftYForward) {
        enableAMotorFwd = true;   // Enable A motor forward
        enableBMotorFwd = true;   // Enable B motor forward
        enableAMotorRev = false;  // Disable A motor reverse
        enableBMotorRev = false;  // Disable B motor reverse
      } else {
        enableAMotorFwd = false;  // Disable A motor forward
        enableBMotorFwd = false;  // Disable B motor forward
        enableAMotorRev = true;   // Enable A motor reverse
        enableBMotorRev = true;   // Enable B motor reverse
      }
    } else {
      enableAMotorFwd = false;  // Disable A motor forward
      enableBMotorFwd = false;  // Disable B motor forward
      enableAMotorRev = false;  // Disable A motor reverse
      enableBMotorRev = false;  // Disable B motor reverse
    }
    if (tankMode) {
      if (currentState.leftY < 110) {
        enableAMotorFwd = true;   // Enable LEFT motor forward
        enableAMotorRev = false;  // Disable LEFT motor reverse
      } else if (currentState.leftY > 145) {
        enableAMotorFwd = false;  // Disable LEFT motor forward
        enableAMotorRev = true;   // Enable LEFT motor reverse
      }
      if (currentState.rightY < 106) {
        enableBMotorFwd = true;  // Disable RIGHT motor forward
        enableBMotorRev = false;   // Enable RIGHT motor reverse
      } else if (currentState.rightY > 133) {
        enableBMotorFwd = false;   // Enable RIGHT motor forward
        enableBMotorRev = true;  // Disable RIGHT motor reverse
      }
    }
    writePins();  // Write the motor control pins based on the joystick values
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

void setup() {
  // Initialize serial monitor
  Serial.begin(115200);

  // Initialize RFM95
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  if (!rfm.init()) {
    Serial.println("RFM95 initialization failed");
    while (1);
  }
  rfm.setFrequency(915.0);
  Serial.println("RFM95 initialized: TRANSMITTER");

  // Debug pins for having controller and button presses
  fastPinMode(PIN_BUTTONPRESS, OUTPUT);
  fastPinMode(PIN_HAVECONTROLLER, OUTPUT);

  delay(300);
}

void loop() {
  // Check if the controller is connected
  PS2ControllerCheck();
  delay(1000 / 140);  // 140 Hz update rate
}
#endif

/*RECEIVER SETUP AND LOOP*/
/*----------------------------------------------------------------------*/
#ifdef RECEIVER
void setup() {
  // Initialize serial monitor
  Serial.begin(115200);

  // Initialize RFM95
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  if (!rfm.init()) {
    Serial.println("RFM95 initialization failed");
    while (1);
  }
  rfm.setFrequency(915.0);
  Serial.println("RFM95 initialized: RECEIVER");

  // Setup pins for BTS7960 Motor Driver
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

void loop() {
  receivePacket();
  controlsDecision();
}
#endif