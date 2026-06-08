#include "RoleConfig.h"

#ifdef RECEIVER

#include <Arduino.h>
#include <RH_RF95.h>
#include <Wire.h>
#include <Adafruit_MCP23X17.h>

#include "AxisMap.h"
#include "ControllerState.h"
#include "DriveSystem.h"
#include "FirmwareInfo.h"
#include "Protocol.h"
#include "RadioConfig.h"
#include "SoftwarePWMX.h"

#ifndef RX_PRINT_DECODED_CONTROL
#define RX_PRINT_DECODED_CONTROL 1
#endif

#ifndef RX_PRINT_DECODE_ERRORS
#define RX_PRINT_DECODE_ERRORS 1
#endif

#ifndef GAS_PEDAL_PIN
#define GAS_PEDAL_PIN A7 // adjust to your available analog pin
#endif

#ifndef RX_BATTERY_PIN
#define RX_BATTERY_PIN A6 // adjust to your battery sense pin
#endif

// Motor FL (Front Left) Pin Assignments
static const uint8_t MOTOR_FL_LPWM = 3; // mcp portA 3 or pin 24
static const uint8_t MOTOR_FL_RPWM = 2; // mcp portA 2 or pin 23
static const uint8_t MOTOR_FL_REN = 0; // mcp portB 0 or pin 1
static const uint8_t MOTOR_FL_LEN = 1; // mcp portB 1 or pin 2
static const uint8_t MOTOR_FL_RIS = A0;
static const uint8_t MOTOR_FL_LIS = A1;
// Motor FR (Front Right) Pin Assignments
static const uint8_t MOTOR_FR_RPWM = 1; // mcp portA 1 pin 22
static const uint8_t MOTOR_FR_LPWM = 0; // mcp portA 0 pin 21
static const uint8_t MOTOR_FR_REN = 2; // mcp portB 2 or pin 3
static const uint8_t MOTOR_FR_LEN = 3; // mcp portB 3 or pin 4
static const uint8_t MOTOR_FR_RIS = A2;
static const uint8_t MOTOR_FR_LIS = A3;
// Motor RL (Rear Left) Pin Assignments
static const uint8_t MOTOR_RL_RPWM = 7; // mcp portA or pin 28
static const uint8_t MOTOR_RL_LPWM = 6; // mcp portA or pin 27
static const uint8_t MOTOR_RL_REN = 4; // mcp portB or pin 5
static const uint8_t MOTOR_RL_LEN = 5; // mcp portB or pin 6
static const uint8_t MOTOR_RL_RIS = A4;
static const uint8_t MOTOR_RL_LIS = A5;
// Motor RR (Rear Right) Pin Assignments
static const uint8_t MOTOR_RR_RPWM = 5; // mcp portA or pin 26
static const uint8_t MOTOR_RR_LPWM = 4; // mcp portA or pin 25
static const uint8_t MOTOR_RR_REN = 6; // mcp portB or pin 7
static const uint8_t MOTOR_RR_LEN = 7; // mcp portB or pin 8
static const uint8_t MOTOR_RR_RIS = 6;
static const uint8_t MOTOR_RR_LIS = 9;
// Other Pin Assignments
static const uint8_t STEER_PWM = 3;
static const uint8_t RX_PACKET_LED_PIN = 13;

static Adafruit_MCP23X17 mcp;
static SoftwarePWMX pwmx(8, &mcp);

static const MotorPins kFlPins = {
  { MOTOR_FL_RPWM, PinSource::MCP_PIN },
  { MOTOR_FL_LPWM, PinSource::MCP_PIN },
  { MOTOR_FL_LEN, PinSource::MCP_PIN },
  { MOTOR_FL_REN, PinSource::MCP_PIN },
  { MOTOR_FL_LIS, PinSource::MCU_PIN },
  { MOTOR_FL_RIS, PinSource::MCU_PIN }
};

static const MotorPins kFrPins = {
  { MOTOR_FR_RPWM, PinSource::MCP_PIN },
  { MOTOR_FR_LPWM, PinSource::MCP_PIN },
  { MOTOR_FR_LEN, PinSource::MCP_PIN },
  { MOTOR_FR_REN, PinSource::MCP_PIN },
  { MOTOR_FR_LIS, PinSource::MCU_PIN },
  { MOTOR_FR_RIS, PinSource::MCU_PIN }
};

static const MotorPins kRlPins = {
  { MOTOR_RL_RPWM, PinSource::MCP_PIN },
  { MOTOR_RL_LPWM, PinSource::MCP_PIN },
  { MOTOR_RL_LEN, PinSource::MCP_PIN },
  { MOTOR_RL_REN, PinSource::MCP_PIN },
  { MOTOR_RL_LIS, PinSource::MCU_PIN },
  { MOTOR_RL_RIS, PinSource::MCU_PIN }
};

static const MotorPins kRrPins = {
  { MOTOR_RR_RPWM, PinSource::MCP_PIN },
  { MOTOR_RR_LPWM, PinSource::MCP_PIN },
  { MOTOR_RR_LEN, PinSource::MCP_PIN },
  { MOTOR_RR_REN, PinSource::MCP_PIN },
  { MOTOR_RR_LIS, PinSource::MCU_PIN },
  { MOTOR_RR_RIS, PinSource::MCU_PIN }
};

static DriveSystem drive(mcp, pwmx, kFlPins, kFrPins, kRlPins, kRrPins);
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
static int16_t maximumSpeed = 255;
static const int16_t kMinimumSpeed = 25;

static int16_t leftYPWM = 0;
static int16_t leftXPWM = 0;
static int16_t rightYPWM = 0;
static int16_t rightXPWM = 0;

static const AxisMap kLeftYMap = { 110, 110, 0, 145, 145, 255, 0, 255 };
static const AxisMap kLeftXMap = { 90, 90, 0, 145, 145, 255, 0, 255 };
static const AxisMap kRightYMap = { 105, 105, 0, 133, 133, 255, 0, 255 };
static const AxisMap kRightXMap = { 105, 105, 0, 135, 135, 255, 0, 255 };

static const unsigned long HEARTBEAT_PERIOD_MS = 100;
static const unsigned long LINK_TIMEOUT_MS = 500;
static unsigned long lastControlPacketMs = 0;
static unsigned long lastStatusSendMs = 0;
static int16_t rssiRawRx = 0;
static int16_t rssiSmoothRx = 0;
static uint16_t receiverBatteryMilliVolts = 0;
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

static void handleButtonPress(uint8_t bit) {
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
    case R2:
      driveEnabled = !driveEnabled;
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

static uint16_t readReceiverBatteryMilliVolts() {
  int raw = analogRead(RX_BATTERY_PIN);
  if (raw < 0) raw = 0;
  if (raw > 1023) raw = 1023;
  return static_cast<uint16_t>((static_cast<unsigned long>(raw) * 48000UL) / 1023UL);
}

static bool hasFreshControlLink() {
  return remoteControllerPresent && (millis() - lastControlPacketMs <= LINK_TIMEOUT_MS);
}

static void sendStatusHeartbeat() {
  unsigned long now = millis();
  if (now - lastStatusSendMs < HEARTBEAT_PERIOD_MS) return;
  lastStatusSendMs = now;

  StatusMessage msg;
  msg.linkOk = hasFreshControlLink();
  msg.controllerPresent = remoteControllerPresent;
  msg.rssiRaw = rssiRawRx;
  msg.rssiSmooth = rssiSmoothRx;
  msg.batteryMv = receiverBatteryMilliVolts;
  drive.getMotorPercents(msg.motorPct);

  uint8_t buffer[STATUS_MESSAGE_SIZE] = {0};
  encodeStatusMessage(msg, buffer);
  if (rfm.send(buffer, sizeof(buffer))) {
    rfm.waitPacketSent();
  }
}

static void printDecodedControlDebug(const ControlMessage &msg) {
#if RX_PRINT_DECODED_CONTROL
  unsigned long now = millis();
  bool changed = !haveDebugState ||
                 msg.controllerPresent != lastDebugControllerPresent ||
                 stateChanged(msg.state, lastDebugState);
  if (!changed && (now - lastControlDebugMs < CONTROL_DEBUG_PERIOD_MS)) return;

  Serial.print(F("RX control seq "));
  Serial.print(msg.seq);
  Serial.print(F(" | present: "));
  Serial.print(msg.controllerPresent ? F("YES") : F("NO"));
  Serial.print(F(" | RSSI: "));
  Serial.print(rssiRawRx);
  Serial.print(F(" | "));
  printControllerState(msg.state, Serial);

  lastDebugState = msg.state;
  lastDebugControllerPresent = msg.controllerPresent;
  haveDebugState = true;
  lastControlDebugMs = now;
#else
  (void)msg;
#endif
}

static void printDecodeErrorDebug(const uint8_t *buffer, uint8_t len) {
#if RX_PRINT_DECODE_ERRORS
  ++decodeErrorCount;
  unsigned long now = millis();
  if (now - lastDecodeErrorMs < DECODE_ERROR_PERIOD_MS) return;
  lastDecodeErrorMs = now;

  Serial.print(F("RX ignored packet #"));
  Serial.print(decodeErrorCount);
  Serial.print(F(" len: "));
  Serial.print(len);
  if (len > 0) {
    Serial.print(F(" version: "));
    Serial.print(buffer[0]);
  }
  if (len > 1) {
    Serial.print(F(" type: "));
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
static bool lastDbgEnabled = false;
static bool lastDbgTankMode = false;
static bool lastDbgParkingBrake = false;
static bool haveDriveDebug = false;
static unsigned long lastDriveDebugMs = 0;

// Mirror printDecodedControlDebug's gating: only emit the drive line when
// something changed or once per CONTROL_DEBUG_PERIOD_MS. Printing it every loop
// saturates the serial TX buffer and stalls loop(), which backlogs every other
// debug line and makes input look laggy/bursty.
static void maybePrintDriveDebug() {
  unsigned long now = millis();
  bool changed = !haveDriveDebug ||
                 leftYPWM != lastDbgLeftY || leftXPWM != lastDbgLeftX ||
                 rightYPWM != lastDbgRightY || rightXPWM != lastDbgRightX ||
                 driveEnabled != lastDbgEnabled || tankMode != lastDbgTankMode ||
                 parkingBrake != lastDbgParkingBrake;
  if (!changed && (now - lastDriveDebugMs < CONTROL_DEBUG_PERIOD_MS)) return;

  printDriveDebug(leftYPWM, leftXPWM, rightYPWM, rightXPWM,
                  driveEnabled, tankMode, parkingBrake, Serial);

  lastDbgLeftY = leftYPWM;
  lastDbgLeftX = leftXPWM;
  lastDbgRightY = rightYPWM;
  lastDbgRightX = rightXPWM;
  lastDbgEnabled = driveEnabled;
  lastDbgTankMode = tankMode;
  lastDbgParkingBrake = parkingBrake;
  haveDriveDebug = true;
  lastDriveDebugMs = now;
}

static void controlsDecision() {
  if (!hasFreshControlLink()) {
    currentState = kNeutralControllerState;
    previousState = kNeutralControllerState;
    buttonEdges.reset(0);
    needButtonResync = true;
    driveEnabled = false;
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

  uint8_t pedal = getGasPedalDriveValue();

  int16_t leftDrive = (clampSpeed(leftYPWM) * pedal) / 255;
  int16_t rightDrive = (clampSpeed(tankMode ? rightYPWM : leftYPWM) * pedal) / 255;

  drive.setEnabled(driveEnabled);
  drive.setParkingBrake(parkingBrake);
  drive.applyDrive(leftDrive, rightDrive, leftDrive, rightDrive);

  uint8_t steerPwm = static_cast<uint8_t>(constrain(rightXPWM + 127, 0, 255));
  analogWrite(STEER_PWM, steerPwm);

  maybePrintDriveDebug();
}

void setup() {
  Serial.begin(115200);
  maybePrintFirmwareBanner();

  pinMode(STEER_PWM, OUTPUT);
  pinMode(RX_PACKET_LED_PIN, OUTPUT);
  digitalWrite(RX_PACKET_LED_PIN, LOW);
  pinMode(GAS_PEDAL_PIN, INPUT);
  pinMode(RX_BATTERY_PIN, INPUT);

  if (!mcp.begin_I2C()) {
    Serial.println("mcp begin error.");
    while (1) {
      Serial.println("mcp error");
    }
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
  Serial.println("RFM95 initialized: RECEIVER");
}

void loop() {
  maybePrintFirmwareBanner();
  updateRxPacketLed();
  handleIncomingPackets();
  receiverBatteryMilliVolts = readReceiverBatteryMilliVolts();
  controlsDecision();
  sendStatusHeartbeat();
  pwmx.update();
  updateRxPacketLed();
}

#endif
