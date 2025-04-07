
/*----------------------------------------------------------------------*/
// Uncomment one of these to set the role of the device
// #define TRANSMITTER
#define RECEIVER

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

/*TRANSMITTER FUNCTIONS AND VARIABLES*/
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
  Serial.println("getting btn name");
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

void controlsDecision() {
  if (stateChanged(currentState, previousState)) {
    buttonWordToDecipher = currentState.buttonWord;
    previousState = currentState;
    getButtonName(buttonWordToDecipher);
  }

  
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
  Serial.println("RFM95 initialized");

  // Debug pins for having controller and button presses
  fastPinMode(PIN_BUTTONPRESS, OUTPUT);
  fastPinMode(PIN_HAVECONTROLLER, OUTPUT);

  delay(300);
}

void loop() {
  // int delayTime = 0; 
  // if (Serial.available()) {
  //   String input =
  //       Serial.readStringUntil('\n');  // Read input until a newline character
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
      PGM_BYTES_P cname = reinterpret_cast<PGM_BYTES_P>(pgm_read_ptr(
          &(controllerTypeStrings[ctype < PSCTRL_MAX ? static_cast<byte>(ctype)
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
  Serial.println("RFM95 initialized");

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
  // controlsDecision();
}
#endif