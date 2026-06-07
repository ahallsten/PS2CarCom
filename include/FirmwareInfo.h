#ifndef FIRMWARE_INFO_H
#define FIRMWARE_INFO_H

#include <Arduino.h>
#include "Protocol.h"

#ifndef FIRMWARE_VERSION
#define FIRMWARE_VERSION "unknown"
#endif

#ifndef FIRMWARE_GIT_SHA
#define FIRMWARE_GIT_SHA "unknown"
#endif

#ifndef FIRMWARE_GIT_BRANCH
#define FIRMWARE_GIT_BRANCH "unknown"
#endif

#ifndef FIRMWARE_BUILD_UTC
#define FIRMWARE_BUILD_UTC __DATE__ " " __TIME__
#endif

/**
 * @brief Print a compact firmware identity banner to a serial-like stream.
 *
 * @param out Destination stream, usually Serial.
 * @param role Human-readable firmware role such as TRANSMITTER or RECEIVER.
 */
inline void printFirmwareBanner(Print &out, const __FlashStringHelper *role) {
  out.println();
  out.println(F("PS2CarCom firmware"));
  out.print(F("Role: "));
  out.println(role);
  out.print(F("Version: "));
  out.println(FIRMWARE_VERSION);
  out.print(F("Git branch: "));
  out.println(FIRMWARE_GIT_BRANCH);
  out.print(F("Git SHA: "));
  out.println(FIRMWARE_GIT_SHA);
  out.print(F("Build UTC: "));
  out.println(FIRMWARE_BUILD_UTC);
  out.print(F("Protocol: "));
  out.println(PROTOCOL_VERSION);
}

#endif
