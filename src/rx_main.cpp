#include "RoleConfig.h"

#ifdef RECEIVER

#include <Arduino.h>
#include <RH_RF95.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_MCP23X17.h>
#include <Adafruit_PWMServoDriver.h>

#include "AxisMap.h"
#include "ControllerState.h"
#include "DriveSystem.h"
#include "FirmwareInfo.h"
#include "Pca9685Pwm.h"
#include "Protocol.h"
#include "RadioConfig.h"

#ifndef RX_PRINT_DECODED_CONTROL
#define RX_PRINT_DECODED_CONTROL 1
#endif

#ifndef RX_PRINT_DRIVE_DEBUG
#define RX_PRINT_DRIVE_DEBUG 1
#endif

#ifndef RX_PRINT_DECODE_ERRORS
#define RX_PRINT_DECODE_ERRORS 1
#endif

#ifndef GAS_PEDAL_PIN
#define GAS_PEDAL_PIN A7 // adjust to your available analog pin
#endif

static const float PCA9685_PWM_FREQ_HZ = 50.0f;

static const uint8_t ADS_BATTERY_TOTAL_CHANNEL = 0;
static const uint8_t ADS_BATTERY_MIDPOINT_CHANNEL = 1;

// ADS1115 GAIN_ONE is +/-4.096 V, or 125 uV/count. Divider ratios are
// placeholders until the real resistor values are measured and calibrated.
static const unsigned long ADS1115_GAIN_ONE_UV_PER_COUNT = 125UL;
static const unsigned long BATTERY_TOTAL_DIVIDER_NUM = 11UL;
static const unsigned long BATTERY_TOTAL_DIVIDER_DEN = 1UL;
static const unsigned long BATTERY_MIDPOINT_DIVIDER_NUM = 11UL;
static const unsigned long BATTERY_MIDPOINT_DIVIDER_DEN = 1UL;

// PCA9685 channel assignments. Each BTS7960 uses one channel per direction.
static const uint8_t MOTOR_FL_RPWM = 0;
static const uint8_t MOTOR_FL_LPWM = 1;
static const uint8_t MOTOR_FL_REN = 10; // mcp port B2 or pin 1
static const uint8_t MOTOR_FL_LEN = 11; // mcp port B3 or pin 2
static const uint8_t MOTOR_FL_RIS = A0;
static const uint8_t MOTOR_FL_LIS = A1;

static const uint8_t MOTOR_FR_RPWM = 2;
static const uint8_t MOTOR_FR_LPWM = 3;
static const uint8_t MOTOR_FR_REN = 12; // mcp port B4 or pin 3
static const uint8_t MOTOR_FR_LEN = 13; // mcp port B5 or pin 4
static const uint8_t MOTOR_FR_RIS = A2;
static const uint8_t MOTOR_FR_LIS = A3;

static const uint8_t MOTOR_RL_RPWM = 4;
static const uint8_t MOTOR_RL_LPWM = 5;
static const uint8_t MOTOR_RL_REN = 8; // mcp port B0 or pin 5
static const uint8_t MOTOR_RL_LEN = 9; // mcp port B1 or pin 6
static const uint8_t MOTOR_RL_RIS = A4;
static const uint8_t MOTOR_RL_LIS = A5;

static const uint8_t MOTOR_RR_RPWM = 6;
static const uint8_t MOTOR_RR_LPWM = 7;
static const uint8_t MOTOR_RR_REN = 14; // mcp port B6 or pin 7
static const uint8_t MOTOR_RR_LEN = 15; // mcp port B7 or pin 8
static const uint8_t MOTOR_RR_RIS = 6;
static const uint8_t MOTOR_RR_LIS = 9;

static const uint8_t STEER_RPWM = 8;
static const uint8_t STEER_LPWM = 9;
static const uint8_t STEER_REN = 0; // freed MCP pins formerly used for software PWM
static const uint8_t STEER_LEN = 1;
static const uint8_t STEER_RIS = A8;
static const uint8_t STEER_LIS = A6;

static const uint8_t RX_PACKET_LED_PIN = 13;

static Adafruit_MCP23X17 mcp;
static Adafruit_PWMServoDriver pca9685;
static Adafruit_ADS1115 ads1115;
static Pca9685Pwm pwm(pca9685);
static bool adsReady = false;

static const MotorPins kFlPins = {
  MOTOR_FL_RPWM,
  MOTOR_FL_LPWM,
  { MOTOR_FL_LEN, PinSource::MCP_PIN },
  { MOTOR_FL_REN, PinSource::MCP_PIN },
  { MOTOR_FL_LIS, PinSource::MCU_PIN },
  { MOTOR_FL_RIS, PinSource::MCU_PIN }
};

static const MotorPins kFrPins = {
  MOTOR_FR_RPWM,
  MOTOR_FR_LPWM,
  { MOTOR_FR_LEN, PinSource::MCP_PIN },
  { MOTOR_FR_REN, PinSource::MCP_PIN },
  { MOTOR_FR_LIS, PinSource::MCU_PIN },
  { MOTOR_FR_RIS, PinSource::MCU_PIN }
};

static const MotorPins kRlPins = {
  MOTOR_RL_RPWM,
  MOTOR_RL_LPWM,
  { MOTOR_RL_LEN, PinSource::MCP_PIN },
  { MOTOR_RL_REN, PinSource::MCP_PIN },
  { MOTOR_RL_LIS, PinSource::MCU_PIN },
  { MOTOR_RL_RIS, PinSource::MCU_PIN }
};

static const MotorPins kRrPins = {
  MOTOR_RR_RPWM,
  MOTOR_RR_LPWM,
  { MOTOR_RR_LEN, PinSource::MCP_PIN },
  { MOTOR_RR_REN, PinSource::MCP_PIN },
  { MOTOR_RR_LIS, PinSource::MCU_PIN },
  { MOTOR_RR_RIS, PinSource::MCU_PIN }
};

static const MotorPins kSteerPins = {
  STEER_RPWM,
  STEER_LPWM,
  { STEER_LEN, PinSource::MCP_PIN },
  { STEER_REN, PinSource::MCP_PIN },
  { STEER_LIS, PinSource::MCU_PIN },
  { STEER_RIS, PinSource::MCU_PIN }
};

static DriveSystem drive(mcp, pwm, kFlPins, kFrPins, kRlPins, kRrPins, kSteerPins);
static RH_RF95 rfm(RFM95_CS, RFM95_INT);

static ControllerState currentState = kNeutralControllerState;
static ControllerState previousState = kNeutralControllerState;
static ButtonEdgeTracker buttonEdges;

enum Button : uint8_t {
  SELECT = 0,
  L3 = 1,
  R3 = 2,
  START = 3,
  UP = 4,
  RIGHT = 5,
  DOWN = 6,
  LEFT = 7,
  L2 = 8,
  R2 = 9,
  L1 = 10,
  R1 = 11,
  TRIANGLE = 12,
  CIRCLE = 13,
  CROSS = 14,
  SQUARE = 15,
  UNKNOWN = 255
};

static bool parkingBrake = false;
static bool driveEnabled = false;
static bool tankMode = false;
static bool controlDebugEnabled = (RX_PRINT_DECODED_CONTROL != 0);
static bool driveDebugEnabled = (RX_PRINT_DRIVE_DEBUG != 0);
static bool forceDriveDebugPrint = false;
static int16_t maximumSpeed = 255;
static const int16_t kMinimumSpeed = 25;

// Reject re-triggers of the same button within this window. PS2 pressure
// buttons (e.g. R2) can flicker across the digital threshold and would
// otherwise rapid-toggle latched actions like drive enable.
static const unsigned long BUTTON_DEBOUNCE_MS = 200;
static unsigned long lastButtonActionMs[16] = {0};

static int16_t leftYPWM = 0;
static int16_t leftXPWM = 0;
static int16_t rightYPWM = 0;
static int16_t rightXPWM = 0;
static uint8_t pedalDriveValue = 0;
static int16_t leftDriveCommand = 0;
static int16_t rightDriveCommand = 0;

static const AxisMap kLeftYMap = { 110, 110, 0, 145, 145, 255, 0, 255 };
static const AxisMap kLeftXMap = { 90, 90, 0, 145, 145, 255, 0, 255 };
static const AxisMap kRightYMap = { 105, 105, 0, 135, 135, 255, 0, 255 };
static const AxisMap kRightXMap = { 105, 105, 0, 135, 135, 255, 0, 255 };

static const unsigned long STATUS_PERIOD_MS = 250;
static const unsigned long LINK_TIMEOUT_MS = 500;
static unsigned long lastControlPacketMs = 0;
static unsigned long lastStatusSendMs = 0;
static uint8_t lastControlSeq = 0;
static int16_t rssiRawRx = 0;
static int16_t rssiSmoothRx = 0;
static uint16_t receiverBatteryTotalMilliVolts = 0;
static uint16_t receiverBatteryMidpointMilliVolts = 0;
static bool remoteControllerPresent = false;
static bool needButtonResync = true;
static bool firmwareBannerPrinted = false;
static ControllerState lastDebugState = kNeutralControllerState;
static bool lastDebugControllerPresent = false;
static bool haveDebugState = false;
static unsigned long lastControlDebugMs = 0;
static unsigned long lastDecodeErrorMs = 0;
static uint16_t decodeErrorCount = 0;
static const unsigned long CONTROL_DEBUG_PERIOD_MS = 1000;
static const unsigned long DECODE_ERROR_PERIOD_MS = 500;
static const unsigned long RX_PACKET_LED_ON_MS = 100;
static const unsigned long RX_PACKET_LED_OFF_MS = 100;
static bool rxPacketLedOn = false;
static bool rxPacketLedPending = false;
static unsigned long rxPacketLedLastTransitionMs = 0;

static void maybePrintFirmwareBanner() {
  if (firmwareBannerPrinted || !Serial) return;
  printFirmwareBanner(Serial, F("RECEIVER"));
  firmwareBannerPrinted = true;
}

static void startRxPacketLedPulse(unsigned long now) {
  digitalWrite(RX_PACKET_LED_PIN, HIGH);
  rxPacketLedOn = true;
  rxPacketLedPending = false;
  rxPacketLedLastTransitionMs = now;
}

static void noteRxPacketForLed() {
  unsigned long now = millis();
  if (!rxPacketLedOn && (now - rxPacketLedLastTransitionMs >= RX_PACKET_LED_OFF_MS)) {
    startRxPacketLedPulse(now);
  } else {
    rxPacketLedPending = true;
  }
}

static void updateRxPacketLed() {
  unsigned long now = millis();
  if (rxPacketLedOn) {
    if (now - rxPacketLedLastTransitionMs >= RX_PACKET_LED_ON_MS) {
      digitalWrite(RX_PACKET_LED_PIN, LOW);
      rxPacketLedOn = false;
      rxPacketLedLastTransitionMs = now;
    }
  } else if (rxPacketLedPending && (now - rxPacketLedLastTransitionMs >= RX_PACKET_LED_OFF_MS)) {
    startRxPacketLedPulse(now);
  }
}

static Button getButtonEnum(uint8_t bit) {
  if (bit > 15) return UNKNOWN;
  return static_cast<Button>(bit);
}

static void printDebugToggle(const __FlashStringHelper *name, bool enabled) {
  printLogPrefix(Serial, F("RX"), F("LOG"), lastControlSeq);
  Serial.print(name);
  Serial.print(F("="));
  Serial.println(enabled ? F("ON") : F("OFF"));
}

static void handleButtonPress(uint8_t bit) {
  if (bit > 15) return;
  unsigned long now = millis();
  if (now - lastButtonActionMs[bit] < BUTTON_DEBOUNCE_MS) return;
  lastButtonActionMs[bit] = now;

  switch (getButtonEnum(bit)) {
    case SELECT:
      parkingBrake = !parkingBrake;
      break;
    case START:
      tankMode = !tankMode;
      break;
    case UP:
      if (maximumSpeed < 255) maximumSpeed = (maximumSpeed + 25 > 255) ? 255 : (maximumSpeed + 25);
      break;
    case DOWN:
      if (maximumSpeed > kMinimumSpeed) {
        maximumSpeed = (maximumSpeed - 25 < kMinimumSpeed) ? kMinimumSpeed : (maximumSpeed - 25);
      }
      break;
    case LEFT:
      maximumSpeed = kMinimumSpeed;
      break;
    case RIGHT:
      maximumSpeed = 255;
      break;
    case L2:
      driveDebugEnabled = !driveDebugEnabled;
      forceDriveDebugPrint = driveDebugEnabled;
      printDebugToggle(F("DRV"), driveDebugEnabled);
      break;
    case R2:
      driveEnabled = !driveEnabled;
      break;
    case L1:
      controlDebugEnabled = !controlDebugEnabled;
      haveDebugState = false;
      printDebugToggle(F("CTRL"), controlDebugEnabled);
      break;
    default:
      break;
  }
}

static void onRisingEdge(uint8_t bit) { handleButtonPress(bit); }

static int16_t clampSpeed(int16_t pwm) {
  int16_t magnitude = abs(pwm);
  if (magnitude == 0) return 0;
  magnitude = constrain(magnitude, kMinimumSpeed, maximumSpeed);
  return (pwm < 0) ? -magnitude : magnitude;
}

static uint8_t getGasPedalDriveValue() {
  int raw = analogRead(GAS_PEDAL_PIN);
  if (raw < 0) raw = 0;
  if (raw > 1023) raw = 1023;
  return static_cast<uint8_t>((raw * 255L) / 1023L);
}

static uint16_t clampToU16(unsigned long value) {
  return static_cast<uint16_t>(value > 65535UL ? 65535UL : value);
}

static uint16_t scaleAdsReadingMilliVolts(int16_t raw,
                                          unsigned long dividerNum,
                                          unsigned long dividerDen) {
  if (raw < 0 || dividerDen == 0) return 0;
  unsigned long pinUv = static_cast<unsigned long>(raw) * ADS1115_GAIN_ONE_UV_PER_COUNT;
  unsigned long pinMv = pinUv / 1000UL;
  return clampToU16((pinMv * dividerNum) / dividerDen);
}

static void readReceiverBatteryTelemetry() {
  if (!adsReady) {
    receiverBatteryTotalMilliVolts = 0;
    receiverBatteryMidpointMilliVolts = 0;
    return;
  }

  int16_t totalRaw = ads1115.readADC_SingleEnded(ADS_BATTERY_TOTAL_CHANNEL);
  int16_t midpointRaw = ads1115.readADC_SingleEnded(ADS_BATTERY_MIDPOINT_CHANNEL);
  receiverBatteryTotalMilliVolts =
    scaleAdsReadingMilliVolts(totalRaw, BATTERY_TOTAL_DIVIDER_NUM, BATTERY_TOTAL_DIVIDER_DEN);
  receiverBatteryMidpointMilliVolts =
    scaleAdsReadingMilliVolts(midpointRaw, BATTERY_MIDPOINT_DIVIDER_NUM, BATTERY_MIDPOINT_DIVIDER_DEN);
}

static bool hasFreshControlLink() {
  return remoteControllerPresent && (millis() - lastControlPacketMs <= LINK_TIMEOUT_MS);
}

static void sendStatusHeartbeat() {
  unsigned long now = millis();
  if (now - lastStatusSendMs < STATUS_PERIOD_MS) return;
  lastStatusSendMs = now;

  readReceiverBatteryTelemetry();

  StatusMessage msg;
  msg.linkOk = hasFreshControlLink();
  msg.controllerPresent = remoteControllerPresent;
  msg.parkingBrake = parkingBrake;
  msg.driveEnabled = driveEnabled;
  msg.tankMode = tankMode;
  msg.rssiRaw = rssiRawRx;
  msg.rssiSmooth = rssiSmoothRx;
  msg.batteryTotalMv = receiverBatteryTotalMilliVolts;
  msg.batteryMidpointMv = receiverBatteryMidpointMilliVolts;
  msg.ackSeq = lastControlSeq;
  drive.getMotorCommands(msg.motorPwm);
  drive.readCurrentSense(msg.currentSense);

  uint8_t buffer[STATUS_MESSAGE_SIZE] = {0};
  encodeStatusMessage(msg, buffer);
  if (rfm.send(buffer, sizeof(buffer))) {
    rfm.waitPacketSent();
  }
}

static void printDecodedControlDebug(const ControlMessage &msg) {
  if (!controlDebugEnabled) return;

  unsigned long now = millis();
  bool changed = !haveDebugState ||
                 msg.controllerPresent != lastDebugControllerPresent ||
                 stateChanged(msg.state, lastDebugState);
  if (!changed && (now - lastControlDebugMs < CONTROL_DEBUG_PERIOD_MS)) return;

  printLogPrefix(Serial, F("RX"), F("CTRL"), msg.seq);
  Serial.print(F("present="));
  Serial.print(msg.controllerPresent ? F("Y") : F("N"));
  Serial.print(F(" rssi="));
  Serial.print(rssiRawRx);
  Serial.print(' ');
  printControllerState(msg.state, Serial);

  lastDebugState = msg.state;
  lastDebugControllerPresent = msg.controllerPresent;
  haveDebugState = true;
  lastControlDebugMs = now;
}

static void printDecodeErrorDebug(const uint8_t *buffer, uint8_t len) {
#if RX_PRINT_DECODE_ERRORS
  ++decodeErrorCount;
  unsigned long now = millis();
  if (now - lastDecodeErrorMs < DECODE_ERROR_PERIOD_MS) return;
  lastDecodeErrorMs = now;

  printLogPrefix(Serial, F("RX"), F("ERR"), 0);
  Serial.print(F("ignored#"));
  Serial.print(decodeErrorCount);
  Serial.print(F(" len="));
  Serial.print(len);
  if (len > 0) {
    Serial.print(F(" ver="));
    Serial.print(buffer[0]);
  }
  if (len > 1) {
    Serial.print(F(" type="));
    Serial.print(buffer[1]);
  }
  Serial.println();
#else
  (void)buffer;
  (void)len;
#endif
}

static bool isExpectedNonControlPacket(const uint8_t *buffer, uint8_t len) {
  return len == STATUS_MESSAGE_SIZE &&
         buffer[0] == PROTOCOL_VERSION &&
         buffer[1] == PACKET_STATUS;
}

static void handleIncomingPackets() {
  while (rfm.available()) {
    uint8_t buffer[MAX_WIRE_PACKET_SIZE] = {0};
    uint8_t len = sizeof(buffer);
    if (!rfm.recv(buffer, &len)) continue;
    noteRxPacketForLed();
    if (len < 2) continue;

    ControlMessage msg;
    if (decodeControlMessage(buffer, len, msg)) {
      currentState = msg.state;
      remoteControllerPresent = msg.controllerPresent;
      lastControlSeq = msg.seq;
      lastControlPacketMs = millis();
      rssiRawRx = rfm.lastRssi();
      rssiSmoothRx = smoothRssi(rssiRawRx, rssiSmoothRx);
      printDecodedControlDebug(msg);
    } else if (isExpectedNonControlPacket(buffer, len)) {
      // The transmitter also sends status heartbeats; the receiver only acts on control packets.
    } else {
      printDecodeErrorDebug(buffer, len);
    }
  }
}


static int16_t lastDbgLeftY = 0;
static int16_t lastDbgLeftX = 0;
static int16_t lastDbgRightY = 0;
static int16_t lastDbgRightX = 0;
static uint8_t lastDbgPedal = 0;
static int16_t lastDbgLeftDriveCommand = 0;
static int16_t lastDbgRightDriveCommand = 0;
static int16_t lastDbgMotorCommand[4] = {0, 0, 0, 0};
static uint8_t lastDbgMotorRDuty[4] = {0, 0, 0, 0};
static uint8_t lastDbgMotorLDuty[4] = {0, 0, 0, 0};
static bool lastDbgEnabled = false;
static bool lastDbgTankMode = false;
static bool lastDbgParkingBrake = false;
static bool haveDriveDebug = false;
static unsigned long lastDriveDebugMs = 0;

static uint8_t snapshotDuty(const SoftwarePwmSnapshot &snapshot) {
  return snapshot.valid ? snapshot.duty : 0;
}

static void printPwmSideDebug(char side, const SoftwarePwmSnapshot &snapshot) {
  Serial.print(' ');
  Serial.print(side);
  if (!snapshot.valid) {
    Serial.print(F("=NA"));
    return;
  }

  Serial.print(F("@"));
  Serial.print(snapshot.pinDef.pin);
  Serial.print(F("="));
  Serial.print(snapshot.duty);
  Serial.print(snapshot.state ? F("/H") : F("/L"));
}

static void printMotorPwmDebug(const __FlashStringHelper *name,
                               int16_t command,
                               const Bts7960PwmSnapshot &snapshot) {
  Serial.print(' ');
  Serial.print(name);
  Serial.print(F("(cmd="));
  Serial.print(command);
  printPwmSideDebug('R', snapshot.rpwm);
  printPwmSideDebug('L', snapshot.lpwm);
  Serial.print(F(")"));
}

// Mirror printDecodedControlDebug's gating: only emit the drive line when
// something changed or once per CONTROL_DEBUG_PERIOD_MS. Printing it every loop
// saturates the serial TX buffer and stalls loop(), which backlogs every other
// debug line and makes input look laggy/bursty.
static void maybePrintDriveDebug() {
  if (!driveDebugEnabled) return;

  int16_t motorCommand[4] = {0, 0, 0, 0};
  Bts7960PwmSnapshot motorPwm[4];
  drive.getLastCommands(motorCommand);
  drive.getPwmSnapshots(motorPwm);

  bool motorChanged = false;
  for (uint8_t i = 0; i < 4; ++i) {
    uint8_t rDuty = snapshotDuty(motorPwm[i].rpwm);
    uint8_t lDuty = snapshotDuty(motorPwm[i].lpwm);
    if (motorCommand[i] != lastDbgMotorCommand[i] ||
        rDuty != lastDbgMotorRDuty[i] ||
        lDuty != lastDbgMotorLDuty[i]) {
      motorChanged = true;
    }
  }

  unsigned long now = millis();
  bool changed = forceDriveDebugPrint || !haveDriveDebug ||
                 leftYPWM != lastDbgLeftY || leftXPWM != lastDbgLeftX ||
                 rightYPWM != lastDbgRightY || rightXPWM != lastDbgRightX ||
                 pedalDriveValue != lastDbgPedal ||
                 leftDriveCommand != lastDbgLeftDriveCommand ||
                 rightDriveCommand != lastDbgRightDriveCommand ||
                 motorChanged ||
                 driveEnabled != lastDbgEnabled || tankMode != lastDbgTankMode ||
                 parkingBrake != lastDbgParkingBrake;
  if (!changed && (now - lastDriveDebugMs < CONTROL_DEBUG_PERIOD_MS)) return;

  printLogPrefix(Serial, F("RX"), F("DRV"), lastControlSeq);
  Serial.print(F("pwmLY="));
  Serial.print(leftYPWM);
  Serial.print(F(" pwmLX="));
  Serial.print(leftXPWM);
  Serial.print(F(" pwmRY="));
  Serial.print(rightYPWM);
  Serial.print(F(" pwmSTR="));
  Serial.print(rightXPWM);
  Serial.print(F(" pedal="));
  Serial.print(pedalDriveValue);
  Serial.print(F(" drvL="));
  Serial.print(leftDriveCommand);
  Serial.print(F(" drvR="));
  Serial.print(rightDriveCommand);
  Serial.print(F(" EN="));
  Serial.print(driveEnabled ? F("ON") : F("OFF"));
  Serial.print(F(" TMD="));
  Serial.print(tankMode ? F("ON") : F("OFF"));
  Serial.print(F(" PBRK="));
  Serial.print(parkingBrake ? F("ON") : F("OFF"));
  printMotorPwmDebug(F("FL"), motorCommand[0], motorPwm[0]);
  printMotorPwmDebug(F("FR"), motorCommand[1], motorPwm[1]);
  printMotorPwmDebug(F("RL"), motorCommand[2], motorPwm[2]);
  printMotorPwmDebug(F("RR"), motorCommand[3], motorPwm[3]);
  Serial.println();

  lastDbgLeftY = leftYPWM;
  lastDbgLeftX = leftXPWM;
  lastDbgRightY = rightYPWM;
  lastDbgRightX = rightXPWM;
  lastDbgPedal = pedalDriveValue;
  lastDbgLeftDriveCommand = leftDriveCommand;
  lastDbgRightDriveCommand = rightDriveCommand;
  for (uint8_t i = 0; i < 4; ++i) {
    lastDbgMotorCommand[i] = motorCommand[i];
    lastDbgMotorRDuty[i] = snapshotDuty(motorPwm[i].rpwm);
    lastDbgMotorLDuty[i] = snapshotDuty(motorPwm[i].lpwm);
  }
  lastDbgEnabled = driveEnabled;
  lastDbgTankMode = tankMode;
  lastDbgParkingBrake = parkingBrake;
  haveDriveDebug = true;
  forceDriveDebugPrint = false;
  lastDriveDebugMs = now;
}

static void controlsDecision() {
  if (!hasFreshControlLink()) {
    currentState = kNeutralControllerState;
    previousState = kNeutralControllerState;
    buttonEdges.reset(0);
    needButtonResync = true;
    driveEnabled = false;
    leftYPWM = 0;
    leftXPWM = 0;
    rightYPWM = 0;
    rightXPWM = 0;
    pedalDriveValue = 0;
    leftDriveCommand = 0;
    rightDriveCommand = 0;
    drive.setEnabled(false);
    drive.applyDriveAll(0);
    return;
  }

  if (needButtonResync) {
    previousState = currentState;
    buttonEdges.reset(currentState.buttonWord);
    needButtonResync = false;
  } else if (stateChanged(currentState, previousState)) {
    previousState = currentState;
    buttonEdges.update(currentState.buttonWord, onRisingEdge, nullptr, nullptr);
  }

  leftYPWM = mapAxisSigned(currentState.leftY, kLeftYMap);
  leftXPWM = mapAxisSigned(currentState.leftX, kLeftXMap);
  rightYPWM = mapAxisSigned(currentState.rightY, kRightYMap);
  rightXPWM = mapAxisSigned(currentState.rightX, kRightXMap);

  pedalDriveValue = getGasPedalDriveValue();

<<<<<<< Updated upstream
  leftDriveCommand = (clampSpeed(leftYPWM) * pedalDriveValue) / 255;
  rightDriveCommand = (clampSpeed(tankMode ? rightYPWM : leftYPWM) * pedalDriveValue) / 255;

  drive.setEnabled(driveEnabled);
  drive.setParkingBrake(parkingBrake);
  drive.applyDrive(leftDriveCommand, rightDriveCommand, leftDriveCommand, rightDriveCommand);

  uint8_t steerPwm = static_cast<uint8_t>(constrain(rightXPWM + 127, 0, 255));
  analogWrite(STEER_PWM, steerPwm);
=======
  int16_t leftDrive = (clampSpeed(leftYPWM) * pedal) / 255;
  int16_t rightDrive = (clampSpeed(tankMode ? rightYPWM : leftYPWM) * pedal) / 255;
  int16_t steerDrive = rightXPWM;

  drive.setEnabled(driveEnabled);
  drive.setParkingBrake(parkingBrake);
  drive.applyDrive(leftDrive, rightDrive, leftDrive, rightDrive, steerDrive);

  maybePrintDriveDebug();
>>>>>>> Stashed changes
}

void setup() {
  Serial.begin(115200);
  maybePrintFirmwareBanner();

  pinMode(RX_PACKET_LED_PIN, OUTPUT);
  digitalWrite(RX_PACKET_LED_PIN, LOW);
  pinMode(GAS_PEDAL_PIN, INPUT);

  Wire.begin();
  Wire.setClock(400000);
  if (!mcp.begin_I2C()) {
    Serial.println("mcp begin error.");
    while (1) {
      Serial.println("mcp error");
    }
  }

  pwm.begin(PCA9685_PWM_FREQ_HZ);

  adsReady = ads1115.begin();
  if (adsReady) {
    ads1115.setGain(GAIN_ONE);
  } else {
    Serial.println(F("ADS1115 initialization failed; battery telemetry will report 0 mV"));
  }

  drive.begin();
  drive.setEnabled(false);
  drive.setParkingBrake(false);

  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  if (!rfm.init()) {
    Serial.println("RFM95 initialization failed");
    while (1) {
      Serial.println("RFM9x Error");
    }
  }

  rfm.setFrequency(RF95_FREQ);
  // Fast+short-range LoRa: ~11ms time-on-air vs ~46ms at the default
  // Bw125Cr45Sf128. Cuts the per-send blocking that can stall receiver control
  // and telemetry work. Must match the transmitter's modem config.
  rfm.setModemConfig(RH_RF95::Bw500Cr45Sf128);
  Serial.println("RFM95 initialized: RECEIVER");
}

void loop() {
  maybePrintFirmwareBanner();
  updateRxPacketLed();
  handleIncomingPackets();
  controlsDecision();
  sendStatusHeartbeat();
<<<<<<< Updated upstream
  pwmx.update();
  maybePrintDriveDebug();
=======
>>>>>>> Stashed changes
  updateRxPacketLed();
}

#endif
