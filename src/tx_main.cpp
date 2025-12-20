#include "RoleConfig.h"

#ifdef TRANSMITTER

#include <Arduino.h>
#include <DigitalIO.h>
#include <PsxControllerBitBang.h>
#include <RH_RF95.h>
#include <SPI.h>
#include <avr/pgmspace.h>

#include "ControllerState.h"
#include "Protocol.h"
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
static bool rxLinkOk = false;
static unsigned long lastRxHeartbeatMs = 0;
static unsigned long lastStatusSendMs = 0;
static const unsigned long HEARTBEAT_PERIOD_MS = 100;
static const unsigned long LINK_TIMEOUT_MS = 500;
static int16_t rssiRawTx = 0;
static int16_t rssiSmoothTx = 0;
static float transmitterBatteryVoltage = 0.0f;
static float receiverBatteryVoltage = 0.0f;
static uint8_t motorPercents[4] = {0, 0, 0, 0};

static float readTransmitterBatteryVoltage() {
  // TODO: hook up actual analog measurement
  return 0.0f;
}

static void updateCurrentState() {
  currentState.buttonWord = psx.getButtonWord();
  psx.getLeftAnalog(currentState.leftX, currentState.leftY);
  psx.getRightAnalog(currentState.rightX, currentState.rightY);
}

static void sendControlIfChanged() {
  if (!stateChanged(currentState, previousState)) return;

  ControlPacket pkt;
  pkt.state = currentState;
  rfm.send(reinterpret_cast<uint8_t*>(&pkt), sizeof(pkt));
  printControllerState(currentState, Serial);
  previousState = currentState;
}

static void sendStatusHeartbeat() {
  unsigned long now = millis();
  if (now - lastStatusSendMs < HEARTBEAT_PERIOD_MS) return;
  lastStatusSendMs = now;

  transmitterBatteryVoltage = readTransmitterBatteryVoltage();

  StatusPacket pkt;
  pkt.linkOk = rxLinkOk ? 1 : 0;
  pkt.rssiRaw = rssiRawTx;
  pkt.rssiSmooth = rssiSmoothTx;
  pkt.batteryV = transmitterBatteryVoltage;
  // Motor pct not applicable on TX; leave zeros

  rfm.send(reinterpret_cast<uint8_t*>(&pkt), sizeof(pkt));
}

static void handleIncomingPackets() {
  while (rfm.available()) {
    uint8_t buffer[sizeof(StatusPacket)] = {0};
    uint8_t len = sizeof(buffer);
    if (!rfm.recv(buffer, &len)) continue;
    if (len < 1) continue;

    uint8_t type = buffer[0];
    if (type == PACKET_STATUS && len == sizeof(StatusPacket)) {
      StatusPacket pkt;
      memcpy(&pkt, buffer, sizeof(StatusPacket));
      lastRxHeartbeatMs = millis();
      rxLinkOk = pkt.linkOk != 0;
      rssiRawTx = rfm.lastRssi();
      rssiSmoothTx = smoothRssi(rssiRawTx, rssiSmoothTx);
      receiverBatteryVoltage = pkt.batteryV;
      memcpy(motorPercents, pkt.motorPct, sizeof(motorPercents));
    } else if (type == PACKET_CONTROL && len == sizeof(ControlPacket)) {
      lastRxHeartbeatMs = millis();
      rxLinkOk = true;
      rssiRawTx = rfm.lastRssi();
      rssiSmoothTx = smoothRssi(rssiRawTx, rssiSmoothTx);
    }
  }
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
      sendControlIfChanged();
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
  handleIncomingPackets();

  if (millis() - lastRxHeartbeatMs > LINK_TIMEOUT_MS) {
    rxLinkOk = false;
  }

  sendStatusHeartbeat();
  delay(5);
}

#endif
