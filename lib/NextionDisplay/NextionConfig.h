#ifndef NEXTION_CONFIG_H
#define NEXTION_CONFIG_H

/**
 * @file NextionConfig.h
 * @brief Compile-time configuration and HMI component names for the Nextion backend.
 *
 * All page and component names live here so they can be edited in one place to
 * match whatever you build in the Nextion editor. The firmware never requires
 * the HMI file to exist at compile time; unknown component names simply produce
 * harmless Nextion "invalid component" responses at runtime.
 */

// Master enable. Define NEXTION_ENABLED=0 (e.g. as a build flag) to compile out
// all Nextion UART traffic while leaving the NextionDisplay API callable as
// no-ops, so the rest of the firmware is unaffected.
#ifndef NEXTION_ENABLED
#define NEXTION_ENABLED 1
#endif

// Nextion UART baud. Nextion's factory default is 9600. After you set a higher
// baud in the HMI (program.s: `baud=115200`), update this to match.
#ifndef NEXTION_BAUD
#define NEXTION_BAUD 9600UL
#endif

// Dashboard widget refresh rate in Hz. Widgets are pushed at this rate from the
// main loop, never every iteration. Keep modest (5-10 Hz) to avoid flooding the
// UART or starving control timing.
#ifndef NEXTION_DASH_HZ
#define NEXTION_DASH_HZ 8
#endif

// ---- Page names (must match the page names in the Nextion editor) ----
#define NEX_PAGE_LOG  "log"   ///< Scrolling raw-log page
#define NEX_PAGE_DASH "dash"  ///< Dashboard widgets page

// ---- Log page components ----
#define NEX_LOG_TEXT   "log.tLog"   ///< Multiline text box that shows recent log lines

// ---- Dashboard page: status widgets ----
#define NEX_DASH_RSSI    "dash.tRssi"     ///< Smoothed RSSI (number/text)
#define NEX_DASH_PARK    "dash.tPark"     ///< Parking brake state
#define NEX_DASH_ENABLE  "dash.tEnable"   ///< Drive enable state
#define NEX_DASH_TANK    "dash.tTank"     ///< Tank-drive mode state
#define NEX_DASH_LINK    "dash.tLink"     ///< Link health: OK / STALE / LOST
#define NEX_DASH_TXBATT  "dash.tTxBatt"   ///< Transmitter battery millivolts
#define NEX_DASH_RXBATT  "dash.tRxBatt"   ///< Receiver battery millivolts

// ---- Dashboard page: per-motor PWM widgets (signed -255..255) ----
#define NEX_DASH_PWM_FL  "dash.tPwmFL"
#define NEX_DASH_PWM_FR  "dash.tPwmFR"
#define NEX_DASH_PWM_RL  "dash.tPwmRL"
#define NEX_DASH_PWM_RR  "dash.tPwmRR"

// ---- Dashboard page: per-motor BTS7960 current-sense widgets (raw ADC) ----
// L_IS / R_IS for each of the four motors.
#define NEX_DASH_IS_FL_L "dash.tIsFLl"
#define NEX_DASH_IS_FL_R "dash.tIsFLr"
#define NEX_DASH_IS_FR_L "dash.tIsFRl"
#define NEX_DASH_IS_FR_R "dash.tIsFRr"
#define NEX_DASH_IS_RL_L "dash.tIsRLl"
#define NEX_DASH_IS_RL_R "dash.tIsRLr"
#define NEX_DASH_IS_RR_L "dash.tIsRRl"
#define NEX_DASH_IS_RR_R "dash.tIsRRr"

#endif
