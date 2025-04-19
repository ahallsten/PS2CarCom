
/*----------------------------------------------------------------------*/
// Uncomment one of these to set the role of the device
// #define DEBUG
// #define TRANSMITTER
#define RECEIVER

#include <LibPrintf.h>
#include <Adafruit_SleepyDog.h>
#include <RH_RF95.h>
#include <Wire.h>
#include <avr/pgmspace.h>
typedef const __FlashStringHelper *FlashStr;
typedef const byte *PGM_BYTES_P;
#define PSTR_TO_F(s) reinterpret_cast<const __FlashStringHelper *>(s)

/*TRANSMITTER DEFINITIONS*/
/*----------------------------------------------------------------------*/
// #ifdef TRANSMITTER
#include <DigitalIO.h>
#include <PsxControllerBitBang.h>
#include <SPI.h>
// #endif

/*RECIEVER DEFINITIONS*/
/*----------------------------------------------------------------------*/
#ifdef RECEIVER
#define MOTORA_REN A4
#define MOTORA_LEN A5
#define MOTORA_RPWM 6
#define MOTORA_LPWM 5
#define MOTORA_RIS A0
#define MOTORA_LIS A1

#define MOTORB_REN 12
#define MOTORB_LEN 11
#define MOTORB_RPWM 9
#define MOTORB_LPWM 10
#define MOTORB_RIS A2
#define MOTORB_LIS A3

#define STEER_PWM 3
#endif

// Pin definitions
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 7

// Radio configuration
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

uint16_t buttonWordToDecipher = 0;

/*TRANSMITTER OBJECTS AND VARIABLES*/
/*----------------------------------------------------------------------*/
// #ifdef TRANSMITTER
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

const char buttonSelectName[] PROGMEM = "Select";
const char buttonL3Name[] PROGMEM = "L3";
const char buttonR3Name[] PROGMEM = "R3";
const char buttonStartName[] PROGMEM = "Start";
const char buttonUpName[] PROGMEM = "Up";
const char buttonRightName[] PROGMEM = "Right";
const char buttonDownName[] PROGMEM = "Down";
const char buttonLeftName[] PROGMEM = "Left";
const char buttonL2Name[] PROGMEM = "L2";
const char buttonR2Name[] PROGMEM = "R2";
const char buttonL1Name[] PROGMEM = "L1";
const char buttonR1Name[] PROGMEM = "R1";
const char buttonTriangleName[] PROGMEM = "Triangle";
const char buttonCircleName[] PROGMEM = "Circle";
const char buttonCrossName[] PROGMEM = "Cross";
const char buttonSquareName[] PROGMEM = "Square";

const char *const psxButtonNames[PSX_BUTTONS_NO] PROGMEM = {
    buttonSelectName,   buttonL3Name,     buttonR3Name,    buttonStartName,
    buttonUpName,       buttonRightName,  buttonDownName,  buttonLeftName,
    buttonL2Name,       buttonR2Name,     buttonL1Name,    buttonR1Name,
    buttonTriangleName, buttonCircleName, buttonCrossName, buttonSquareName};

enum Button {
  SELECT,
  START,
  UP,
  DOWN,
  LEFT,
  RIGHT,
  TRIANGLE,
  CIRCLE,
  CROSS,
  SQUARE,
  L1,
  R1,
  L2,
  R2,
  L3,
  R3,
  UNKNOWN
};

bool parking_Brake = false;
int maximumSpeed = 0;       // Maximum speed of the car
bool enableToggle = false;  // Toggle for enabling/disabling the car
bool enableRightMotor = 0;  // Enable right motor
bool enableLeftMotor = 0;   // Enable left motor
bool tankMode = false;  // Tank mode allows for independent control of left and
                        // right motors with the left and right joysticks

/*TRANSMITTER & RECEIVER FUNCTIONS*/
/*----------------------------------------------------------------------*/
byte psxButtonToIndex(PsxButtons psxButtons) {
  byte i;
  for (i = 0; i < PSX_BUTTONS_NO; ++i) {
    if (psxButtons & 0x01) {
      break;
    }
    psxButtons >>= 1U;
  }
  return i;
}

FlashStr getButtonName(PsxButtons psxButton) {
  FlashStr ret = F("");
  byte b = psxButtonToIndex(psxButton);
  if (b < PSX_BUTTONS_NO) {
    PGM_BYTES_P bName =
        reinterpret_cast<PGM_BYTES_P>(pgm_read_ptr(&(psxButtonNames[b])));
    ret = PSTR_TO_F(bName);
    Serial.print("btn name: ");
    Serial.println(ret);
  }
  return ret;
}

bool stateChanged(const ControllerState &current,
                  const ControllerState &previous) {
  return current.buttonWord != previous.buttonWord ||
         current.leftX != previous.leftX || current.leftY != previous.leftY ||
         current.rightX != previous.rightX || current.rightY != previous.rightY;
}

void updateCurrentState() {
  currentState.buttonWord = psx.getButtonWord();
  psx.getLeftAnalog(currentState.leftX, currentState.leftY);
  psx.getRightAnalog(currentState.rightX, currentState.rightY);
}

#ifdef TRANSMITTER
/*TRANSMITTER FUNCTIONS*/
/*----------------------------------------------------------------------*/
void dumpButtons(PsxButtons psxButtons) {
  static PsxButtons lastB = 0;
  if (psxButtons != lastB) {
    lastB = psxButtons;  // Save it before we alter it
    Serial.print(F("Pressed: "));
    for (byte i = 0; i < PSX_BUTTONS_NO; ++i) {
      byte b = psxButtonToIndex(psxButtons);
      if (b < PSX_BUTTONS_NO) {
        PGM_BYTES_P bName =
            reinterpret_cast<PGM_BYTES_P>(pgm_read_ptr(&(psxButtonNames[b])));
        Serial.print(PSTR_TO_F(bName));
      }
      psxButtons &= ~(1 << b);
      if (psxButtons != 0) {
        Serial.print(F(", "));
      }
    }
    Serial.println();
  }
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

    // Print for debugging
    Serial.println(F("Packet sent!"));
    Serial.print(currentState.buttonWord);
    Serial.print(" ");
    Serial.print(currentState.leftX);
    Serial.print(" ");
    Serial.print(currentState.leftY);
    Serial.print(" ");
    Serial.print(currentState.rightX);
    Serial.print(" ");
    Serial.println(currentState.rightY);

    // Update the previous state
    previousState = currentState;
  }
}

const char ctrlTypeUnknown[] PROGMEM = "Unknown";
const char ctrlTypeDualShock[] PROGMEM = "Dual Shock";
const char ctrlTypeDsWireless[] PROGMEM = "Dual Shock Wireless";
const char ctrlTypeGuitHero[] PROGMEM = "Guitar Hero";
const char ctrlTypeOutOfBounds[] PROGMEM = "(Out of bounds)";

const char *const controllerTypeStrings[PSCTRL_MAX + 1] PROGMEM = {
    ctrlTypeUnknown, ctrlTypeDualShock, ctrlTypeDsWireless, ctrlTypeGuitHero,
    ctrlTypeOutOfBounds};
#endif

#ifdef RECEIVER
/*RECEIVER FUNCTIONS*/
/*----------------------------------------------------------------------*/
void receivePacket() {
  if (rfm.available()) {
    uint8_t buffer[sizeof(ControllerState)];
    uint8_t len = sizeof(buffer);

    // Receive the packet
    rfm.recv(buffer, &len);

    // Deserialize into a struct
    memcpy(&currentState, buffer, sizeof(ControllerState));

    // Process the received state

    Serial.print(F("ButtonWord: "));
    printf("%016b", currentState.buttonWord);
    Serial.print(F(" (HEX: "));
    Serial.print(currentState.buttonWord, HEX);
    Serial.print(F(" "));
    Serial.print(currentState.leftX);
    Serial.print(F(" "));
    Serial.print(currentState.leftY);
    Serial.print(F(" "));
    Serial.print(currentState.rightX);
    Serial.print(F(" "));
    Serial.println(currentState.rightY);
  }
}
Button getButtonEnum(const String &buttonName) {
  if (buttonName == "Select") return SELECT;
  if (buttonName == "Start") return START;
  if (buttonName == "Up") return UP;
  if (buttonName == "Down") return DOWN;
  if (buttonName == "Left") return LEFT;
  if (buttonName == "Right") return RIGHT;
  if (buttonName == "Triangle") return TRIANGLE;
  if (buttonName == "Circle") return CIRCLE;
  if (buttonName == "Cross") return CROSS;
  if (buttonName == "Square") return SQUARE;
  if (buttonName == "L1") return L1;
  if (buttonName == "R1") return R1;
  if (buttonName == "L2") return L2;
  if (buttonName == "R2") return R2;
  if (buttonName == "L3") return L3;
  if (buttonName == "R3") return R3;
  return UNKNOWN;
}
void handleButtonPress(const String &buttonName) {
  switch (getButtonEnum(buttonName)) {
    case SELECT:
      Serial.println(F("Start button pressed!"));
      parking_Brake = true;
      break;

    case START:
      tankMode = !tankMode;  // Toggle tank mode
      Serial.print(buttonName);
      Serial.print(F("Tank mode: "));
      Serial.println(tankMode ? F("ON") : F("OFF"));
      break;

    case UP:
      Serial.println(F("Up button pressed!"));
      maximumSpeed += 25;
      Serial.print(F("Maximum Speed: "));
      Serial.println(maximumSpeed);
      break;

    case DOWN:
      Serial.println(F("Down button pressed!"));
      maximumSpeed -= 25;
      Serial.print(F("Maximum Speed: "));
      Serial.println(maximumSpeed);
      break;

    case LEFT:
      Serial.println(F("Left button pressed!"));
      break;

    case RIGHT:
      Serial.println(F("Right button pressed!"));
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
      enableLeftMotor = 1;
      Serial.println(F("L1 button pressed!"));
      break;

    case R1:
      enableRightMotor = 1;
      Serial.println(F("R1 button pressed!"));
      break;

    case L2:
      Serial.println(F("L2 button pressed!"));
      break;

    case R2:
      Serial.println(F("R2 button pressed!"));
      enableToggle = !enableToggle;
      break;

    case L3:
      Serial.println(F("L3 button pressed!"));
      break;

    case R3:
      Serial.println(F("R3 button pressed!"));
      break;

    case UNKNOWN:
    default:
      Serial.println(F("FUCK!"));
      break;
  }
  // if (buttonName == "Select") {                   // toggle tank mode
  // } else if (buttonName == "Start") {
  //   Serial.println(F("Start button pressed!"));
  //   parking_Brake = true;
  // } else if (buttonName == "Up") {                // Increase maximum speed
  //   Serial.println(F("Up button pressed!"));
  //   maximumSpeed += 25;
  //   Serial.print(F("Maximum Speed: "));
  //   Serial.println(maximumSpeed);
  // } else if (buttonName == "Down") {              // Decrease maximum speed
  //   Serial.println(F("Down button pressed!"));
  //   maximumSpeed -= 25;
  //   Serial.print(F("Maximum Speed: "));
  //   Serial.println(maximumSpeed);
  // } else if (buttonName == "Left") {
  //   Serial.println(F("Left button pressed!"));
  // } else if (buttonName == "Right") {
  //   Serial.println(F("Right button pressed!"));
  // } else if (buttonName == "Triangle") {
  //   Serial.println(F("Triangle button pressed!"));
  // } else if (buttonName == "Circle") {            // Parking brake
  //   Serial.println(F("Circle button pressed!"));
  // } else if (buttonName == "Cross") {
  //   Serial.println(F("Cross button pressed!"));
  // } else if (buttonName == "Square") {            // Realtime brake
  //   Serial.println(F("Square button pressed!"));
  // } else if (buttonName == "L1") {                // Enable left motor
  //   enableLeftMotor = 1;
  //   Serial.println(F("L1 button pressed!"));
  // } else if (buttonName == "R1") {                // Enable right motor
  //   enableRightMotor = 1;
  //   Serial.println(F("R1 button pressed!"));
  // } else if (buttonName == "L2") {
  //   Serial.println(F("L2 button pressed!"));
  // } else if (buttonName == "R2") {                // Realtime Toggle Moter
  // enable state
  //   Serial.println(F("R2 button pressed!"));
  //   enableToggle = !enableToggle;
  // } else if (buttonName == "L3") {
  //   Serial.println(F("L3 button pressed!"));
  // } else if (buttonName == "R3") {
  //   Serial.println(F("R3 button pressed!"));
  // } else {
  //   Serial.println(F("Unknown button pressed!"));
  // }
}

void controlsDecision() {
  if (stateChanged(currentState, previousState)) {
    buttonWordToDecipher = currentState.buttonWord;
    previousState = currentState;
    getButtonName(buttonWordToDecipher);
    FlashStr buttonName = F("");
    Serial.print(buttonName);
    Serial.print(F(" "));
    handleButtonPress(buttonName);

    // Set the motor direction and speed based on the joystick values
    int pwmValue = 0;
    bool forward = false;
    bool backward = false;
    if (currentState.leftY < 140) {
      // Map values from 140 to 255 to 0 to 255
      pwmValue = map(currentState.leftY, 110, 0, 0, 255);
      Serial.print(F(" -> PWM: "));
      Serial.print(pwmValue);
      Serial.print(F(" "));
      forward = true;
      backward = false;
    } else if (currentState.leftY > 110) {
      // Map values from 110 to 0 to 0 to 255
      pwmValue = map(currentState.leftY, 140, 255, 0, 255);
      Serial.print(F(" <- PWM: "));
      Serial.print(pwmValue);
      Serial.print(F(" "));
      forward = false;
      backward = true;
    } else {
      // Dead zone: set PWM to 0
      pwmValue = 0;
      forward = false;
      backward = false;
    }

    // Use the Right Joystick to control the steering Servo
    int steerPwmValue = 0;
    if (currentState.rightX > 140) {
      // Map values from 140 to 255 to 0 to 255
      steerPwmValue = map(currentState.rightX, 135, 255, 0, 255);
      Serial.print(F("STR RT:  "));
      Serial.print(steerPwmValue);
    } else if (currentState.rightX < 110) {
      // Map values from 110 to 0 to 0 to 255
      steerPwmValue = map(currentState.rightX, 105, 0, 0, 255);
      Serial.print(F("STR LT: "));
      Serial.println(steerPwmValue);
    } else {
      // Dead zone: set PWM to 0
      steerPwmValue = 0;
    }

    // Write the PWM value to the desired pin
    Serial.print(F("fwd: "));
    Serial.print(forward ? "true" : "false");
    Serial.print(F(" bwd: "));
    Serial.println(backward ? "true" : "false");
    digitalWrite(MOTORA_REN, forward ? HIGH : LOW);
    digitalWrite(MOTORA_LEN, backward ? HIGH : LOW);
    analogWrite(STEER_PWM, steerPwmValue);
    analogWrite(MOTORA_RPWM, pwmValue);
    analogWrite(MOTORA_LPWM, pwmValue);
  }
}
#endif

#ifdef DEBUG
void sendDebug() {
  // Serialize the struct into a byte array
  uint8_t packet[sizeof(ControllerState)];
  memcpy(packet, &currentState, sizeof(ControllerState));

  // Send the packet via RFM9x
  rfm.send(packet, sizeof(packet));

  // Print for debugging
  Serial.println(F("Packet sent!"));
  Serial.print(currentState.buttonWord);
  Serial.print(" ");
  Serial.print(currentState.leftX);
  Serial.print(" ");
  Serial.print(currentState.leftY);
  Serial.print(" ");
  Serial.print(currentState.rightX);
  Serial.print(" ");
  Serial.println(currentState.rightY);
  Serial.println("finished");

  // // Update the previous state
  // previousState = currentState;
}
#endif
/*TRANSMITTER SETUP AND LOOP*/
/*----------------------------------------------------------------------*/
#ifdef TRANSMITTER

void setup() {
  // Initialize serial monitor
  Serial.begin(115200);
  while (!Serial);

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
  // int delayTime = 0;
  // if (Serial.available()) {
  //   String input =
  //       Serial.readStringUntil('\n');  // Read input until a newline
  //       character
  //   input.trim();  // Remove any leading or trailing whitespace

  //   delayTime = input.toInt();  // Convert input to an integer
  //   if (delayTime > 0) {            // Ensure it's a valid positive number
  //     Serial.print(F("Delay time set to: "));
  //     Serial.println(delayTime);
  //   } else {
  //     Serial.println(F("Invalid input for delay time."));
  //   }
  // }

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

  delay(1000 / 60);  // 60 Hz update rate
}
#endif

/*RECEIVER SETUP AND LOOP*/
/*----------------------------------------------------------------------*/
#ifdef RECEIVER
void setup() {
  // Initialize serial monitor
  Serial.begin(115200);
  while (!Serial);

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