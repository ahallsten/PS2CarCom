#include "SoftwarePWMX.h"

SoftwarePWMX::SoftwarePWMX(uint8_t maxChannels, Adafruit_MCP23X17 *mcp)
  : _max(maxChannels), _mcp(mcp) {
  _ch = (PWMChannelX *)malloc(sizeof(PWMChannelX) * _max);
  for (uint8_t i = 0; i < _max; ++i) {
    _ch[i].pinDef.pin = 255; // mark unused
    _ch[i].pinDef.source = PinSource::MCP_PIN;
  }
}

SoftwarePWMX::~SoftwarePWMX() {
  free(_ch);
}

void SoftwarePWMX::pinModeX(PinDef pin, uint8_t mode) {
  if (pin.source == PinSource::MCP_PIN && _mcp)
    _mcp->pinMode(pin.pin, mode);
  else
    pinMode(pin.pin, mode);
}

void SoftwarePWMX::digitalWriteX(PinDef pin, uint8_t val) {
  if (pin.source == PinSource::MCP_PIN && _mcp)
    _mcp->digitalWrite(pin.pin, val);
  else
    digitalWrite(pin.pin, val);
}

int8_t SoftwarePWMX::addChannel(PinDef pin, uint8_t duty, unsigned long period_us) {
  for (uint8_t i = 0; i < _max; ++i) {
    if (_ch[i].pinDef.pin == 255) {
      _ch[i].pinDef = pin;
      pinModeX(pin, OUTPUT);
      digitalWriteX(pin, LOW);
      _ch[i].duty = duty;
      _ch[i].period_us = period_us;
      _ch[i].on_time_us = dutyToMicros(duty, period_us);
      _ch[i].last_time_us = micros();
      _ch[i].state = false;
      return (int8_t)i;
    }
  }
  return -1;
}

void SoftwarePWMX::removeChannel(int8_t idx) {
  if (idx < 0 || idx >= _max) return;
  if (_ch[idx].pinDef.pin != 255) {
    digitalWriteX(_ch[idx].pinDef, LOW);
    _ch[idx].pinDef.pin = 255;
  }
}

void SoftwarePWMX::setDuty(int8_t idx, uint8_t duty) {
  if (idx < 0 || idx >= _max) return;
  if (_ch[idx].pinDef.pin == 255) return;
  _ch[idx].duty = duty;
  _ch[idx].on_time_us = dutyToMicros(duty, _ch[idx].period_us);
}

void SoftwarePWMX::setPeriod(int8_t idx, unsigned long period_us) {
  if (idx < 0 || idx >= _max) return;
  if (_ch[idx].pinDef.pin == 255) return;
  _ch[idx].period_us = period_us;
  _ch[idx].on_time_us = dutyToMicros(_ch[idx].duty, period_us);
}

unsigned long SoftwarePWMX::dutyToMicros(uint8_t duty, unsigned long period_us) {
  return (unsigned long)(((unsigned long)duty * (unsigned long)period_us) / 255UL);
}

void SoftwarePWMX::update() {
  unsigned long now = micros();
  for (uint8_t i = 0; i < _max; ++i) {
    if (_ch[i].pinDef.pin == 255) continue;
    PWMChannelX &c = _ch[i];

    if (c.duty == 0) {
      if (c.state) {
        digitalWriteX(c.pinDef, LOW);
        c.state = false;
      }
      c.last_time_us = now;
      continue;
    }
    if (c.duty >= 255) {
      if (!c.state) {
        digitalWriteX(c.pinDef, HIGH);
        c.state = true;
      }
      c.last_time_us = now;
      continue;
    }

    unsigned long elapsed = now - c.last_time_us;

    if (!c.state) {
      unsigned long off_time = c.period_us - c.on_time_us;
      if (elapsed >= off_time) {
        digitalWriteX(c.pinDef, HIGH);
        c.state = true;
        c.last_time_us = now;
      }
    } else {
      if (elapsed >= c.on_time_us) {
        digitalWriteX(c.pinDef, LOW);
        c.state = false;
        c.last_time_us = now;
      }
    }
  }
}
