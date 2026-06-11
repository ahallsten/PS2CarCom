#include "RoleConfig.h"

#ifdef TRANSMITTER

#include <Arduino.h>
#include <DigitalIO.h>
#include <PsxControllerBitBang.h>
#include <RH_RF95.h>
#include <SPI.h>
#include <avr/pgmspace.h>

#include "ControllerState.h"
#include "FirmwareInfo.h"
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
static ControllerState currentState = kNeutralControllerState;
static bool rxLinkOk = false;
static unsigned long lastRxHeartbeatMs = 0;
static unsigned long lastControlSendMs = 0;
static unsigned long lastStatusSendMs = 0;
static const unsigned long CONTROL_PERIOD_MS = 50;
static const unsigned long HEARTBEAT_PERIOD_MS = 100;
static const unsigned long LINK_TIMEOUT_MS = 500;
static int16_t rssiRawTx = 0;
static int16_t rssiSmoothTx = 0;
static uint16_t transmitterBatteryMilliVolts = 0;
static uint16_t receiverBatteryTotalMilliVolts = 0;
static uint16_t receiverBatteryMidpointMilliVolts = 0;
static int16_t motorPwm[VEHICLE_MOTOR_COUNT] = {0, 0, 0, 0, 0};
static uint16_t currentSense[VEHICLE_CURRENT_SENSE_COUNT] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static ControllerState lastSentState = kNeutralControllerState;
static bool lastSentControllerPresent = false;
static uint8_t controlSequence = 0;
static uint8_t lastSentSeq = 0;
static uint8_t lastAckedSeq = 0;
static bool firmwareBannerPrinted = false;

struct AxisConditioning {
  uint8_t lowDead;
  uint8_t highDead;
  uint8_t center;
  uint8_t changeThreshold;
};

static const uint8_t kAxisCenter = 128;
static const uint8_t kAxisChangeThreshold = 4;
static const AxisConditioning kLeftXAxis = { 90, 145, kAxisCenter, kAxisChangeThreshold };
static const AxisConditioning kLeftYAxis = { 110, 145, kAxisCenter, kAxisChangeThreshold };
static const AxisConditioning kRightXAxis = { 105, 135, kAxisCenter, kAxisChangeThreshold };
static const AxisConditioning kRightYAxis = { 105, 133, kAxisCenter, kAxisChangeThreshold };

static uint16_t readTransmitterBatteryMilliVolts() {
  // TODO: hook up actual analog measurement
  return 0;
}

static void maybePrintFirmwareBanner() {
  if (firmwareBannerPrinted || !Serial) return;
  printFirmwareBanner(Serial, F("TRANSMITTER"));
  firmwareBannerPrinted = true;
}

static uint8_t conditionAxisValue(uint8_t raw, uint8_t previous,
                                  const AxisConditioning &conditioning) {
  if (raw >= conditioning.lowDead && raw <= conditioning.highDead) {
    return conditioning.center;
  }

  uint8_t delta = (raw > previous) ? (raw - previous) : (previous - raw);
  if (delta < conditioning.changeThreshold) {
    return previous;
  }

  return raw;
}

static void updateCurrentState() {
  uint8_t leftX = 0;
  uint8_t leftY = 0;
  uint8_t rightX = 0;
  uint8_t rightY = 0;

  currentState.buttonWord = psx.getButtonWord();
  psx.getLeftAnalog(leftX, leftY);
  psx.getRightAnalog(rightX, rightY);

  currentState.leftX = conditionAxisValue(leftX, currentState.leftX, kLeftXAxis);
  currentState.leftY = conditionAxisValue(leftY, currentState.leftY, kLeftYAxis);
  currentState.rightX = conditionAxisValue(rightX, currentState.rightX, kRightXAxis);
  currentState.rightY = conditionAxisValue(rightY, currentState.rightY, kRightYAxis);
}

static void sendControlHeartbeat() {
  ControlMessage msg;
  msg.seq = controlSequence;
  msg.controllerPresent = haveController;
  msg.state = haveController ? currentState : kNeutralControllerState;

  unsigned long now = millis();
  bool changed = (msg.controllerPresent != lastSentControllerPresent) ||
                 stateChanged(msg.state, lastSentState);
  if (!changed && (now - lastControlSendMs < CONTROL_PERIOD_MS)) return;

  uint8_t buffer[CONTROL_MESSAGE_SIZE] = {0};
  encodeControlMessage(msg, buffer);
  if (!rfm.send(buffer, sizeof(buffer))) return;
  rfm.waitPacketSent();

  lastControlSendMs = now;
  lastSentState = msg.state;
  lastSentControllerPresent = msg.controllerPresent;
  lastSentSeq = msg.seq;
  ++controlSequence;

  if (changed && msg.controllerPresent) {
    printLogPrefix(Serial, F("TX"), F("CTRL"), msg.seq);
    Serial.print(F("present=Y "));
    printControllerState(msg.state, Serial);
  }
}

static void sendStatusHeartbeat() {
  unsigned long now = millis();
  if (now - lastStatusSendMs < HEARTBEAT_PERIOD_MS) return;
  lastStatusSendMs = now;

  transmitterBatteryMilliVolts = readTransmitterBatteryMilliVolts();

  StatusMessage msg;
  msg.linkOk = rxLinkOk;
  msg.controllerPresent = haveController;
  msg.rssiRaw = rssiRawTx;
  msg.rssiSmooth = rssiSmoothTx;
  msg.batteryTotalMv = transmitterBatteryMilliVolts;

  uint8_t buffer[STATUS_MESSAGE_SIZE] = {0};
  encodeStatusMessage(msg, buffer);
  if (rfm.send(buffer, sizeof(buffer))) {
    rfm.waitPacketSent();
  }
}

static void handleIncomingPackets() {
  while (rfm.available()) {
    uint8_t buffer[MAX_WIRE_PACKET_SIZE] = {0};
    uint8_t len = sizeof(buffer);
    if (!rfm.recv(buffer, &len)) continue;
    if (len < 2) continue;

    StatusMessage msg;
    if (decodeStatusMessage(buffer, len, msg)) {
      lastRxHeartbeatMs = millis();
      rxLinkOk = msg.linkOk;
      rssiRawTx = rfm.lastRssi();
      rssiSmoothTx = smoothRssi(rssiRawTx, rssiSmoothTx);
      receiverBatteryTotalMilliVolts = msg.batteryTotalMv;
      receiverBatteryMidpointMilliVolts = msg.batteryMidpointMv;
      for (uint8_t i = 0; i < VEHICLE_MOTOR_COUNT; ++i) motorPwm[i] = msg.motorPwm[i];
      for (uint8_t i = 0; i < VEHICLE_CURRENT_SENSE_COUNT; ++i) currentSense[i] = msg.currentSense[i];
      lastAckedSeq = msg.ackSeq;

      // ACK line: seq= is the last control packet the receiver decoded, so
      // lastsent - ackedSeq shows how far behind the link is running.
      printLogPrefix(Serial, F("TX"), F("ACK"), msg.ackSeq);
      Serial.print(F("lastsent="));
      Serial.print(lastSentSeq);
      Serial.print(F(" link="));
      Serial.print(msg.linkOk ? F("OK") : F("NO"));
      Serial.print(F(" rssi="));
      Serial.print(rssiRawTx);
      Serial.print(F(" rxbatt_total="));
      Serial.print(receiverBatteryTotalMilliVolts);
      Serial.print(F(" rxbatt_mid="));
      Serial.print(receiverBatteryMidpointMilliVolts);
      Serial.print(F(" rxbatt_1="));
      Serial.print(receiverBatteryMidpointMilliVolts);
      Serial.print(F(" rxbatt_2="));
      Serial.print(receiverBatteryTotalMilliVolts > receiverBatteryMidpointMilliVolts
                     ? receiverBatteryTotalMilliVolts - receiverBatteryMidpointMilliVolts
                     : 0);
      Serial.print(F(" steer="));
      Serial.println(motorPwm[MOTOR_INDEX_STEER]);
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
      currentState = kNeutralControllerState;
    } else {
      updateCurrentState();
    }
  }
}

void setup() {
  Serial.begin(115200);
  maybePrintFirmwareBanner();

  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  if (!rfm.init()) {
    Serial.println("RFM95 initialization failed");
    while (1) {
      ;
    }
  }
  rfm.setFrequency(RF95_FREQ);
  // Fast+short-range LoRa: ~11ms time-on-air vs ~46ms at the default
  // Bw125Cr45Sf128. Must match the receiver's modem config.
  rfm.setModemConfig(RH_RF95::Bw500Cr45Sf128);
  Serial.println("RFM95 initialized: TRANSMITTER");

  fastPinMode(PIN_BUTTONPRESS, OUTPUT);
  fastPinMode(PIN_HAVECONTROLLER, OUTPUT);

  delay(300);
}

void loop() {
  maybePrintFirmwareBanner();
  PS2ControllerCheck();
  sendControlHeartbeat();
  handleIncomingPackets();

  if (millis() - lastRxHeartbeatMs > LINK_TIMEOUT_MS) {
    rxLinkOk = false;
  }

  sendStatusHeartbeat();
  delay(5);
}

#endif
