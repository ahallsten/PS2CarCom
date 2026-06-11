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
#include "NextionDisplay.h"
#include "Protocol.h"
#include "RadioConfig.h"
#include "SystemLog.h"

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

// Nextion HMI backend on Serial1 (D0/RX, D1/TX) — a hardware UART that is fully
// independent of the native-USB Serial used by the computer Serial Monitor.
static NextionDisplay nextion(Serial1);
static SystemLog systemLog;

static ControllerState currentState = kNeutralControllerState;
static bool rxLinkOk = false;
static unsigned long lastRxHeartbeatMs = 0;
static unsigned long lastControlSendMs = 0;
static unsigned long lastStatusSendMs = 0;
static const unsigned long CONTROL_PERIOD_MS = 50;
static const unsigned long HEARTBEAT_PERIOD_MS = 100;
static const unsigned long LINK_TIMEOUT_MS = 500;   // healthy link upper bound
static const unsigned long LINK_LOST_MS = 1500;     // beyond stale -> lost
static int16_t rssiRawTx = 0;
static int16_t rssiSmoothTx = 0;
static uint16_t transmitterBatteryMilliVolts = 0;
static ControllerState lastSentState = kNeutralControllerState;
static bool lastSentControllerPresent = false;
static uint8_t controlSequence = 0;
static uint8_t lastSentSeq = 0;
static uint8_t lastAckedSeq = 0;
static bool firmwareBannerPrinted = false;

// ---- Transmitter battery sense ----
// A9 (= D9) is the Feather 32u4 on-board LiPo monitor pin; its divider halves
// VBAT, so multiply the pin millivolts back by 2. Adjust the divider constants
// if you wire battery sense elsewhere.
#ifndef TX_BATTERY_PIN
#define TX_BATTERY_PIN A9
#endif
static const unsigned long TX_ADC_REF_MV = 3300UL;
static const unsigned long TX_BATT_DIV_NUM = 2;
static const unsigned long TX_BATT_DIV_DEN = 1;

// Latest telemetry mirrored from the receiver status packet plus local values,
// rendered onto the Nextion dashboard at NEXTION_DASH_HZ.
struct LatestTelemetry {
  int16_t rssiSmooth = 0;
  uint16_t txBatteryMv = 0;
  uint16_t rxBatteryMv = 0;
  uint16_t rxBatteryRaw = 0;
  int16_t motorPwm[4] = {0, 0, 0, 0};
  uint16_t currentSense[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  bool parkingBrake = false;
  bool driveEnabled = false;
  bool tankMode = false;
  bool linkOk = false;
  bool controllerPresent = false;
};
static LatestTelemetry telemetry;

static const unsigned long DASH_PERIOD_MS = 1000UL / (NEXTION_DASH_HZ);
static unsigned long lastDashMs = 0;

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
  long raw = analogRead(TX_BATTERY_PIN);
  if (raw < 0) raw = 0;
  if (raw > 1023) raw = 1023;
  unsigned long pinMv = (static_cast<unsigned long>(raw) * TX_ADC_REF_MV) / 1023UL;
  return static_cast<uint16_t>((pinMv * TX_BATT_DIV_NUM) / TX_BATT_DIV_DEN);
}

static const char *linkHealthStr() {
  unsigned long age = millis() - lastRxHeartbeatMs;
  if (age <= LINK_TIMEOUT_MS) return "OK";
  if (age <= LINK_LOST_MS) return "STALE";
  return "LOST";
}

// Push dashboard widgets at NEXTION_DASH_HZ, and only while the dashboard page
// is showing, so the UART and control timing are never flooded.
static void updateDashboard() {
#if NEXTION_ENABLED
  if (nextion.currentPage() != NextionDisplay::Page::Dash) return;
  unsigned long now = millis();
  if (now - lastDashMs < DASH_PERIOD_MS) return;
  lastDashMs = now;

  nextion.setNumber(NEX_DASH_RSSI, telemetry.rssiSmooth);
  nextion.setText(NEX_DASH_PARK, telemetry.parkingBrake ? "ON" : "OFF");
  nextion.setText(NEX_DASH_ENABLE, telemetry.driveEnabled ? "ON" : "OFF");
  nextion.setText(NEX_DASH_TANK, telemetry.tankMode ? "ON" : "OFF");
  nextion.setText(NEX_DASH_LINK, linkHealthStr());
  nextion.setNumber(NEX_DASH_TXBATT, telemetry.txBatteryMv);
  nextion.setNumber(NEX_DASH_RXBATT, telemetry.rxBatteryMv);

  nextion.setNumber(NEX_DASH_PWM_FL, telemetry.motorPwm[0]);
  nextion.setNumber(NEX_DASH_PWM_FR, telemetry.motorPwm[1]);
  nextion.setNumber(NEX_DASH_PWM_RL, telemetry.motorPwm[2]);
  nextion.setNumber(NEX_DASH_PWM_RR, telemetry.motorPwm[3]);

  nextion.setNumber(NEX_DASH_IS_FL_L, telemetry.currentSense[0]);
  nextion.setNumber(NEX_DASH_IS_FL_R, telemetry.currentSense[1]);
  nextion.setNumber(NEX_DASH_IS_FR_L, telemetry.currentSense[2]);
  nextion.setNumber(NEX_DASH_IS_FR_R, telemetry.currentSense[3]);
  nextion.setNumber(NEX_DASH_IS_RL_L, telemetry.currentSense[4]);
  nextion.setNumber(NEX_DASH_IS_RL_R, telemetry.currentSense[5]);
  nextion.setNumber(NEX_DASH_IS_RR_L, telemetry.currentSense[6]);
  nextion.setNumber(NEX_DASH_IS_RR_R, telemetry.currentSense[7]);
#endif
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
  telemetry.txBatteryMv = transmitterBatteryMilliVolts;

  StatusMessage msg;
  msg.linkOk = rxLinkOk;
  msg.controllerPresent = haveController;
  msg.rssiRaw = rssiRawTx;
  msg.rssiSmooth = rssiSmoothTx;
  msg.batteryMv = transmitterBatteryMilliVolts;

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
      lastAckedSeq = msg.ackSeq;

      // Mirror the receiver telemetry into the dashboard state object.
      telemetry.linkOk = msg.linkOk;
      telemetry.controllerPresent = msg.controllerPresent;
      telemetry.parkingBrake = msg.parkingBrake;
      telemetry.driveEnabled = msg.driveEnabled;
      telemetry.tankMode = msg.tankMode;
      telemetry.rssiSmooth = rssiSmoothTx;
      telemetry.rxBatteryMv = msg.batteryMv;
      telemetry.rxBatteryRaw = msg.batteryRaw;
      for (uint8_t i = 0; i < 4; ++i) telemetry.motorPwm[i] = msg.motorPwm[i];
      for (uint8_t i = 0; i < 8; ++i) telemetry.currentSense[i] = msg.currentSense[i];

      // ACK line: seq= is the last control packet the receiver decoded, so
      // lastsent - ackedSeq shows how far behind the link is running.
      printLogPrefix(Serial, F("TX"), F("ACK"), msg.ackSeq);
      Serial.print(F("lastsent="));
      Serial.print(lastSentSeq);
      Serial.print(F(" link="));
      Serial.print(msg.linkOk ? F("OK") : F("NO"));
      Serial.print(F(" rssi="));
      Serial.print(rssiRawTx);
      Serial.print(F(" rxbatt="));
      Serial.println(msg.batteryMv);
    }
  }
}

static void PS2ControllerCheck() {
  if (!haveController) {
    if (psx.begin()) {
      systemLog.line(F("Controller found"));
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
      // Controller is up: jump to the dashboard page (touch events can switch
      // back to the log page later).
      nextion.showDashboardPage();
    }
  } else {
    if (!psx.read()) {
      systemLog.line(F("Controller lost"));
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

  // Nextion UART is a separate hardware port; USB Serial stays available for the
  // computer Serial Monitor.
#if NEXTION_ENABLED
  Serial1.begin(NEXTION_BAUD);
#endif
  nextion.begin();
  systemLog.begin(&Serial, &nextion);

  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  if (!rfm.init()) {
    systemLog.line(F("RFM95 init FAILED"));
    while (1) {
      ;
    }
  }
  rfm.setFrequency(RF95_FREQ);
  // Fast+short-range LoRa: ~11ms time-on-air vs ~46ms at the default
  // Bw125Cr45Sf128. Must match the receiver's modem config.
  rfm.setModemConfig(RH_RF95::Bw500Cr45Sf128);
  systemLog.line(F("RFM95 TX ready"));

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
  updateDashboard();
  delay(5);
}

#endif
