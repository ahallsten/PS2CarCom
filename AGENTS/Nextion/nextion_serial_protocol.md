# Nextion Serial Protocol Draft

Created: 2026-06-29

## 1. Goal

Define a simple, testable UART interface between the transmitter MCU and Nextion HMI.

## 2. Transport

- Physical: UART TX/RX plus common ground.
- Logic level: confirm with selected MCU/display. Use level shifting if required.
- Baud: start with a conservative rate, then raise after stable testing if needed.
- Command terminator: every Nextion command ends with bytes `0xFF 0xFF 0xFF`.

## 3. MCU to display commands

Use direct Nextion commands wrapped by firmware helpers.

Examples:

| Purpose | Command payload before terminator |
|---|---|
| Show page | `page p_drive` |
| Set text | `t_link.txt="LINK OK"` |
| Set armed state | `t_arm.txt="SAFE"` |
| Set number | `n_spdlim.val=125` |
| Set progress/gauge | `j_txbat.val=82` |
| Set brightness | `dim=60` |

## 4. Firmware helper API draft

```cpp
class NextionDisplay {
public:
    void begin(Stream& port);
    void poll();
    void showPage(const __FlashStringHelper* page);
    void setText(const __FlashStringHelper* obj, const char* value);
    void setNumber(const __FlashStringHelper* obj, int32_t value);
    void setGauge(const __FlashStringHelper* obj, uint8_t value);
    void setBrightness(uint8_t percent);
    bool isPresent() const;
};
```

Implementation rules:

- All writes append `0xFF 0xFF 0xFF`.
- Avoid dynamic allocation on AVR where possible.
- Cache last values and skip unchanged updates.
- Keep command buffers small and bounded.
- Make display failure nonfatal.

## 5. Display to MCU commands

Option A: Nextion built-in touch event frames.

- Use Send Component ID for buttons.
- MCU maps page ID + component ID to an event.
- Good for simple designs.

Option B: Custom printable command tokens from touch event code.

Examples:

- `printh 23 04 FF FF FF` for event ID 4.
- Or print ASCII-like tokens if using a parser designed for them.

Recommendation for Rev A: use built-in touch events or simple numeric custom events, then keep the mapping in `event_map.csv`.

## 6. Fault tolerance

- Display missing: transmitter still works.
- Display reset: MCU resends current page/state when it detects the display again or on heartbeat.
- UART noise: ignore unknown events and require valid event IDs.
- Startup race: delay or handshake before first full update.
- Stale display: include a heartbeat indicator or update counter on diagnostics page.

## 7. Test plan

1. Display power-only test.
2. UART hello test.
3. Page switching test.
4. Text update test.
5. Number/gauge update test.
6. Touch event test.
7. Display disconnect while transmitter runs.
8. Display reconnect/reboot while transmitter runs.
9. High update-rate diagnostics stress test.
10. Low-battery/fault display test.

