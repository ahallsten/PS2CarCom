#include "RoleConfig.h"

#ifdef TRANSMITTER

#include <Arduino.h>
#include <DigitalIO.h>
#include <PsxControllerBitBang.h>
#include <RH_RF95.h>
#include <SPI.h>
#include <avr/pgmspace.h>

#include "ControllerState.h"
#include "RadioConfig.h"

typedef const __FlashStringHelper* FlashStr;
typedef const byte* PGM_BYTES_P;
#define PSTR_TO_F(s) reinterpret_cast<const __FlashStringHelper *>(s)

static const byte PIN_PS2_ATT = 10;
static const byte PIN_PS2_CMD = 11;
static const byte PIN_PS2_DAT = 12;
static const byte PIN_PS2_CLK = 13;

static const byte PIN_BUTTONPRESS = A0;
static const byte PIN_HAVECONTROLLER = A1;

PsxControllerBitBang<PIN_PS2_ATT, PIN_PS2_CMD, PIN_PS2_DAT, PIN_PS2_CLK> psx;
static bool haveController = false;

static const char ctrlTypeUnknown[] PROGMEM = "Unknown";
static const char ctrlTypeDualShock[] PROGMEM = "Dual Shock";
static const char ctrlTypeDsWireless[] PROGMEM = "Dual Shock Wireless";
static const char ctrlTypeGuitHero[] PROGMEM = "Guitar Hero";
static const char ctrlTypeOutOfBounds[] PROGMEM = "(Out of bounds)";

static const char* const controllerTypeStrings[PSCTRL_MAX + 1] PROGMEM = {
  ctrlTypeUnknown, ctrlTypeDualShock, ctrlTypeDsWireless, ctrlTypeGuitHero,
  ctrlTypeOutOfBounds
};

static RH_RF95 rfm(RFM95_CS, RFM95_INT);
static ControllerState currentState = { 0, 0, 0, 0, 0 };
static ControllerState previousState = { 0, 0, 0, 0, 0 };

static void updateCurrentState() {
  currentState.buttonWord = psx.getButtonWord();
  psx.getLeftAnalog(currentState.leftX, currentState.leftY);
  psx.getRightAnalog(currentState.rightX, currentState.rightY);
}

static void sendStateIfChanged() {
  if (!stateChanged(currentState, previousState)) return;

  uint8_t packet[sizeof(ControllerState)];
  memcpy(packet, &currentState, sizeof(ControllerState));
  rfm.send(packet, sizeof(packet));
  printControllerState(currentState, Serial);
  previousState = currentState;
}

static void PS2ControllerCheck() {
  if (!haveController) {
    if (psx.begin()) {
      Serial.println(F("Controller found!"));
      delay(300);
      if (!psx.enterConfigMode()) {
        Serial.println(F("Cannot enter config mode"));
      } else {
        PsxControllerType ctype = psx.getControllerType();
        PGM_BYTES_P cname = reinterpret_cast<PGM_BYTES_P>(pgm_read_ptr(&(
          controllerTypeStrings[ctype < PSCTRL_MAX ? static_cast<byte>(ctype) : PSCTRL_MAX])));
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
    }
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  if (!rfm.init()) {
    Serial.println("RFM95 initialization failed");
    while (1) {
      ;
    }
  }
  rfm.setFrequency(RF95_FREQ);
  Serial.println("RFM95 initialized: TRANSMITTER");

  fastPinMode(PIN_BUTTONPRESS, OUTPUT);
  fastPinMode(PIN_HAVECONTROLLER, OUTPUT);

  delay(300);
}

void loop() {
  PS2ControllerCheck();
  delay(1000 / 140);
}

#endif
