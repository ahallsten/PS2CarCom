#include "RoleConfig.h"

#ifdef RECEIVER

#include <Arduino.h>
#include <RH_RF95.h>
#include <Wire.h>
#include <Adafruit_MCP23X17.h>

#include "AxisMap.h"
#include "ControllerState.h"
#include "DriveSystem.h"
#include "Protocol.h"
#include "RadioConfig.h"
#include "SoftwarePWMX.h"

#ifndef GAS_PEDAL_PIN
#define GAS_PEDAL_PIN A7 // adjust to your available analog pin
#endif

#ifndef RX_BATTERY_PIN
#define RX_BATTERY_PIN A6 // adjust to your battery sense pin
#endif

// Motor FL (Front Left) Pin Assignments
static const uint8_t MOTOR_FL_LPWM = 13;
static const uint8_t MOTOR_FL_RPWM = 12;
static const uint8_t MOTOR_FL_REN = 0; // mcp portB 0 or pin 1
static const uint8_t MOTOR_FL_LEN = 1; // mcp portB 1 or pin 2
static const uint8_t MOTOR_FL_RIS = A0;
static const uint8_t MOTOR_FL_LIS = A1;
// Motor FR (Front Right) Pin Assignments
static const uint8_t MOTOR_FR_RPWM = 11;
static const uint8_t MOTOR_FR_LPWM = 10;
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

static ControllerState currentState = { 0, 0, 0, 0, 0 };
static ControllerState previousState = { 0, 0, 0, 0, 0 };
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
static unsigned long lastTxHeartbeatMs = 0;
static unsigned long lastStatusSendMs = 0;
static bool txLinkOk = false;
static int16_t rssiRawRx = 0;
static int16_t rssiSmoothRx = 0;
static float receiverBatteryVoltage = 0.0f;

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

static float readReceiverBatteryVoltage() {
  int raw = analogRead(RX_BATTERY_PIN);
  if (raw < 0) raw = 0;
  return raw * (48.0f / 1023.0f);
}

static void sendStatusHeartbeat() {
  unsigned long now = millis();
  if (now - lastStatusSendMs < HEARTBEAT_PERIOD_MS) return;
  lastStatusSendMs = now;

  StatusPacket pkt;
  pkt.linkOk = txLinkOk ? 1 : 0;
  pkt.rssiRaw = rssiRawRx;
  pkt.rssiSmooth = rssiSmoothRx;
  pkt.batteryV = receiverBatteryVoltage;
  drive.getMotorPercents(pkt.motorPct);

  rfm.send(reinterpret_cast<uint8_t*>(&pkt), sizeof(pkt));
}

static void handleIncomingPackets() {
  while (rfm.available()) {
    uint8_t buffer[sizeof(StatusPacket)] = {0};
    uint8_t len = sizeof(buffer);
    if (!rfm.recv(buffer, &len)) continue;
    if (len < 1) continue;

    uint8_t type = buffer[0];
    if (type == PACKET_CONTROL && len == sizeof(ControlPacket)) {
      ControlPacket pkt;
      memcpy(&pkt, buffer, sizeof(ControlPacket));
      currentState = pkt.state;
      lastTxHeartbeatMs = millis();
      txLinkOk = true;
      rssiRawRx = rfm.lastRssi();
      rssiSmoothRx = smoothRssi(rssiRawRx, rssiSmoothRx);
    } else if (type == PACKET_STATUS && len == sizeof(StatusPacket)) {
      lastTxHeartbeatMs = millis();
      txLinkOk = true;
      rssiRawRx = rfm.lastRssi();
      rssiSmoothRx = smoothRssi(rssiRawRx, rssiSmoothRx);
    }
  }
}

static void controlsDecision() {
  unsigned long now = millis();
  if (now - lastTxHeartbeatMs > LINK_TIMEOUT_MS) {
    txLinkOk = false;
    drive.setEnabled(false);
    drive.applyDriveAll(0);
    return;
  }

  if (stateChanged(currentState, previousState)) {
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

  printDriveDebug(leftYPWM, leftXPWM, rightYPWM, rightXPWM,
                  driveEnabled, tankMode, parkingBrake, Serial);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  pinMode(STEER_PWM, OUTPUT);
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
  handleIncomingPackets();
  receiverBatteryVoltage = readReceiverBatteryVoltage();
  controlsDecision();
  sendStatusHeartbeat();
  pwmx.update();
}

#endif
