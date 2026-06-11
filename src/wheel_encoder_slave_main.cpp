#include "RoleConfig.h"

#ifdef WHEEL_ENCODER_SLAVE

#include <Arduino.h>
#include <Wire.h>

#include "FirmwareInfo.h"
#include "WheelEncoderProtocol.h"

static const uint8_t TCA9548A_ADDRESS = 0x70;
static const uint8_t AS5600_ADDRESS = 0x36;
static const uint8_t AS5600_ANGLE_REGISTER = 0x0E;
static const uint16_t AS5600_COUNTS_PER_TURN = 4096;
static const int16_t AS5600_HALF_SCALE = 2048;
static const unsigned long SAMPLE_PERIOD_MS = 20;

static const uint8_t kWheelTcaChannel[4] = {0, 1, 2, 3};

static volatile uint8_t txBuffer[WHEEL_ENCODER_PACKET_SIZE] = {0};
static WheelEncoderMessage latestMessage;
static uint16_t lastRaw[4] = {0, 0, 0, 0};
static bool haveRaw[4] = {false, false, false, false};
static unsigned long lastWheelSampleMs[4] = {0, 0, 0, 0};
static unsigned long lastSampleMs = 0;
static bool firmwareBannerPrinted = false;

static int16_t clampI16(long value) {
  if (value > 32767L) return 32767;
  if (value < -32768L) return -32768;
  return static_cast<int16_t>(value);
}

static int16_t unwrapDelta(uint16_t raw, uint16_t previous) {
  int16_t delta = static_cast<int16_t>(raw) - static_cast<int16_t>(previous);
  if (delta > AS5600_HALF_SCALE) {
    delta -= AS5600_COUNTS_PER_TURN;
  } else if (delta < -AS5600_HALF_SCALE) {
    delta += AS5600_COUNTS_PER_TURN;
  }
  return delta;
}

static bool selectTcaChannel(uint8_t channel) {
  if (channel > 7) return false;
  Wire.beginTransmission(TCA9548A_ADDRESS);
  Wire.write(static_cast<uint8_t>(1U << channel));
  return Wire.endTransmission() == 0;
}

static bool readAs5600Raw(uint16_t &raw) {
  Wire.beginTransmission(AS5600_ADDRESS);
  Wire.write(AS5600_ANGLE_REGISTER);
  if (Wire.endTransmission(false) != 0) return false;

  if (Wire.requestFrom(AS5600_ADDRESS, static_cast<uint8_t>(2)) != 2) return false;
  uint8_t high = Wire.read();
  uint8_t low = Wire.read();
  raw = ((static_cast<uint16_t>(high) & 0x0F) << 8) | low;
  return raw < AS5600_COUNTS_PER_TURN;
}

static void onI2cRequest() {
  uint8_t local[WHEEL_ENCODER_PACKET_SIZE];
  for (uint8_t i = 0; i < WHEEL_ENCODER_PACKET_SIZE; ++i) {
    local[i] = txBuffer[i];
  }
  Wire.write(local, WHEEL_ENCODER_PACKET_SIZE);
}

static void onI2cReceive(int count) {
  while (count-- > 0 && Wire.available()) {
    (void)Wire.read();
  }
}

static void publishLatestMessage() {
  uint8_t encoded[WHEEL_ENCODER_PACKET_SIZE] = {0};
  encodeWheelEncoderMessage(latestMessage, encoded);
  noInterrupts();
  for (uint8_t i = 0; i < WHEEL_ENCODER_PACKET_SIZE; ++i) {
    txBuffer[i] = encoded[i];
  }
  interrupts();
}

static void sampleWheel(uint8_t index, unsigned long now) {
  uint16_t raw = 0;
  bool selected = selectTcaChannel(kWheelTcaChannel[index]);
  bool ok = selected && readAs5600Raw(raw);
  if (!selected) {
    latestMessage.status |= WHEEL_ENCODER_STATUS_TCA_FAULT;
  }
  if (!ok) {
    latestMessage.status |= WHEEL_ENCODER_STATUS_SENSOR_FAULT;
    latestMessage.delta[index] = 0;
    latestMessage.speedCountsPerSecond[index] = 0;
    return;
  }

  latestMessage.raw[index] = raw;
  if (!haveRaw[index]) {
    haveRaw[index] = true;
    lastRaw[index] = raw;
    lastWheelSampleMs[index] = now;
    latestMessage.delta[index] = 0;
    latestMessage.speedCountsPerSecond[index] = 0;
    return;
  }

  int16_t delta = unwrapDelta(raw, lastRaw[index]);
  unsigned long elapsedMs = now - lastWheelSampleMs[index];
  latestMessage.delta[index] = delta;
  latestMessage.speedCountsPerSecond[index] =
    elapsedMs > 0 ? clampI16((static_cast<long>(delta) * 1000L) / elapsedMs) : 0;
  lastRaw[index] = raw;
  lastWheelSampleMs[index] = now;
}

static void sampleAllWheels() {
  unsigned long now = millis();
  if (now - lastSampleMs < SAMPLE_PERIOD_MS) return;
  lastSampleMs = now;

  latestMessage.status = 0;
  for (uint8_t i = 0; i < 4; ++i) {
    sampleWheel(i, now);
  }
  ++latestMessage.sequence;
  publishLatestMessage();
}

static void maybePrintFirmwareBanner() {
  if (firmwareBannerPrinted || !Serial) return;
  printFirmwareBanner(Serial, F("WHEEL_ENCODER_SLAVE"));
  Serial.print(F("I2C slave address: 0x"));
  Serial.println(WHEEL_ENCODER_SLAVE_I2C_ADDRESS, HEX);
  Serial.print(F("TCA9548A address: 0x"));
  Serial.println(TCA9548A_ADDRESS, HEX);
  firmwareBannerPrinted = true;
}

void setup() {
  Serial.begin(115200);
  Wire.begin(WHEEL_ENCODER_SLAVE_I2C_ADDRESS);
  Wire.onRequest(onI2cRequest);
  Wire.onReceive(onI2cReceive);
  publishLatestMessage();
  maybePrintFirmwareBanner();
}

void loop() {
  maybePrintFirmwareBanner();
  sampleAllWheels();
}

#endif
