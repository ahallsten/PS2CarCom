#include "SystemLog.h"

void SystemLog::begin(Print *usb, NextionDisplay *display) {
  _usb = usb;
  _display = display;
  _head = 0;
  _count = 0;
}

void SystemLog::push(const char *text) {
  char *slot = _ring[_head];
  strncpy(slot, text, SYSLOG_LINE_LEN - 1);
  slot[SYSLOG_LINE_LEN - 1] = '\0';
  _head = (_head + 1) % SYSLOG_LINES;
  if (_count < SYSLOG_LINES) ++_count;
}

void SystemLog::line(const char *text) {
  if (_usb) _usb->println(text);
  push(text);
  renderToNextion();
}

void SystemLog::line(const __FlashStringHelper *text) {
  char buf[SYSLOG_LINE_LEN];
  // Copy the flash string into RAM (truncating) so it can be both printed and
  // stored in the ring buffer.
  strncpy_P(buf, reinterpret_cast<const char *>(text), SYSLOG_LINE_LEN - 1);
  buf[SYSLOG_LINE_LEN - 1] = '\0';
  if (_usb) _usb->println(buf);
  push(buf);
  renderToNextion();
}

void SystemLog::renderToNextion() {
  if (!_display) return;

  // Compose oldest-to-newest into one multiline string, then push it as the log
  // text. Built on the stack to keep static RAM use low.
  char combined[SYSLOG_LINES * (SYSLOG_LINE_LEN + 2) + 1];
  combined[0] = '\0';

  uint8_t idx = (_head + SYSLOG_LINES - _count) % SYSLOG_LINES;
  for (uint8_t i = 0; i < _count; ++i) {
    strcat(combined, _ring[idx]);
    if (i + 1 < _count) strcat(combined, "\r\n");
    idx = (idx + 1) % SYSLOG_LINES;
  }

  _display->appendLogLine(combined);
}
