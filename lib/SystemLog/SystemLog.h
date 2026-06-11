#ifndef SYSTEM_LOG_H
#define SYSTEM_LOG_H

#include <Arduino.h>
#include "NextionDisplay.h"

/**
 * @file SystemLog.h
 * @brief Tiny logging multiplexer: mirrors short log lines to USB Serial and the
 *        Nextion log page.
 *
 * This is intentionally not a drop-in replacement for every Serial.print() in
 * the firmware. Existing Serial output is left untouched; route only the few
 * lines you also want on the Nextion log page through SystemLog. A small ring
 * buffer keeps the most recent lines so the log page can be repainted (e.g.
 * after switching back to it).
 */

// Ring buffer geometry. RAM cost is SYSLOG_LINES * SYSLOG_LINE_LEN bytes.
#ifndef SYSLOG_LINES
#define SYSLOG_LINES 6
#endif
#ifndef SYSLOG_LINE_LEN
#define SYSLOG_LINE_LEN 28
#endif

class SystemLog {
public:
  /**
   * @brief Bind output sinks. Either may be null.
   *
   * @param usb USB Serial (or any Print) for the computer Serial Monitor.
   * @param display Nextion backend whose log page mirrors the lines, or null.
   */
  void begin(Print *usb, NextionDisplay *display);

  /** @brief Log a RAM string line. */
  void line(const char *text);

  /** @brief Log a flash (F("...")) string line. */
  void line(const __FlashStringHelper *text);

  /** @brief Repaint the Nextion log page from the ring buffer. */
  void renderToNextion();

private:
  void push(const char *text);

  Print *_usb = nullptr;
  NextionDisplay *_display = nullptr;
  char _ring[SYSLOG_LINES][SYSLOG_LINE_LEN];
  uint8_t _head = 0;   ///< index of the next slot to write
  uint8_t _count = 0;  ///< number of valid lines (<= SYSLOG_LINES)
};

#endif
