# Other Circuit Planning

Planning notes for receiver and transmitter power control, protection, charger
input handling, motor noise suppression, and auto-shutdown behavior.

> **Core recommendation:** do not use a 555 timer as the main 30-minute power
> controller. Let the Feather decide when the receiver is idle, then have it
> command a proper soft-latch or load-switch circuit to shut logic power off.

## At a Glance

| Area | Recommended Direction |
| --- | --- |
| Receiver logic power | LTC2955-style pushbutton power controller or MOSFET soft latch |
| Receiver motor power | Fuse plus manual disconnect, contactor, or high-current latching relay |
| 30-minute auto-off | Firmware timer that asserts `KILL` on the power latch |
| Motion failsafe | Stop motors after 250-500 ms without valid radio packets |
| Charger input protection | LTC4368 or equivalent reverse/OV/surge protection |
| Logic/accessory protection | TPS2660/TPS2663 eFuse class devices where current is reasonable |
| Low-voltage disconnect | Optional Victron BatteryProtect in the load path only |
| Motor noise | Motor capacitors, twisted leads, local bulk capacitance, bus TVS |
| Transmitter power | PowerBoost 1000C with switch wired to `EN`, not battery positive |

## Useful Search Terms

These circuits are common. Useful names to search:

- Soft-latch power switch
- Pushbutton power controller
- Load switch
- High-side switch
- Low-voltage disconnect
- eFuse
- Ideal-diode controller
- PowerPath controller
- Contactor
- Latching relay

## Recommended Receiver Power Architecture

Use three power domains:

```text
24 V battery pack
   |
   +-- main fuse, close to battery
   |
   +-- manual service disconnect / key switch / emergency stop
   |
   +-- charger connection / charge-controller branch
   |
   +-- logic branch:
   |      protected input
   |        -> soft-latch controller
   |        -> buck converter
   |        -> Feather / MCP / radio
   |
   +-- motor branch:
          high-current fuse
            -> contactor or manual switch
            -> BTS7960 motor bus
```

The important idea is that **logic power** and **motor power** should not be
treated the same.

- The Feather, RFM9x, MCP23017, and control electronics are low-current loads
  that can be switched with MOSFET/load-switch circuitry.
- The motors can draw high surge current, so they need real DC-rated fuses and
  a manual disconnect, contactor, or high-current rated latching relay.

## Auto Turnoff: Use a Soft Power Latch

A normal non-latching relay is a poor fit for "stay on indefinitely, then turn
off later" because the coil burns power the whole time. A standard SPDT relay
also is not necessary unless an auxiliary contact is needed.

Better choices:

1. MOSFET soft latch controlling the buck converter `EN` pin
2. Pushbutton power-controller IC controlling the buck converter `EN` pin
3. Latching relay or bistable contactor for high-current motor power
4. Low-voltage disconnect module for battery protection

### Clean Receiver Logic Power Flow

```text
Momentary ON button
       |
       v
LTC2955 / soft-latch controller
       |
       +---- enables 5-36 V buck converter
       |
       +---- wakes Feather
       |
       +---- Feather keeps running
       |
       +---- after 30 min no valid command, Feather asserts KILL
       |
       v
Power latch turns buck off
```

The **LTC2955** is a strong fit for a 24 V lead-acid pack because it accepts
**1.5 V to 36 V**, draws about **1.2 uA**, has pushbutton on/off control,
supports automatic turn-on through a voltage-monitor input, and has a `KILL`
input so the microcontroller can shut itself down. ([Analog Devices][1])

The older **LTC2954** is also a pushbutton on/off controller, but its input
range tops out at **26.4 V**. That is too low for a 24 V lead-acid pack during
charge, because the pack may reach about **29.4 V**. ([Analog Devices][2])

### Concrete Receiver Recommendation

```text
24 V pack
  -> small fuse
  -> LTC2955 always-on input

LTC2955 EN
  -> buck converter EN

Feather GPIO
  -> LTC2955 KILL

Momentary button
  -> LTC2955 PB

Charger-present divider
  -> Feather input and/or LTC2955 voltage-monitor input
```

If the buck converter does not expose an `EN` pin, choose one that does or put a
MOSFET/load switch in front of the buck.

## 30-Minute Auto-Off Logic

The 30-minute timeout should mostly be firmware:

```cpp
uint32_t lastValidCommandMs = 0;

void onValidRadioPacket()
{
  lastValidCommandMs = millis();
}

void loop()
{
  if (millis() - lastValidCommandMs > 30UL * 60UL * 1000UL)
  {
    stopAllMotors();
    delay(100);
    requestPowerOff(); // assert LTC2955 KILL or release soft latch
  }
}
```

Use two different timeouts:

| Timeout | Purpose | Behavior |
| --- | --- | --- |
| 250-500 ms without packet | Motion safety | Command motors to zero |
| 30 min without command | Battery preservation | Power down receiver logic |

The first timeout protects people and hardware. The second protects the battery.

## Easy Power-On Behavior

Use a **momentary pushbutton** or **key switch** as the wake input. Do not
require opening the enclosure or disconnecting the battery.

Target behavior:

```text
Press ON button:
    logic powers up
    motors remain disabled until software initializes cleanly

Plug in charger:
    logic may optionally power up
    motors are forcibly disabled

No radio command for 30 minutes:
    motors off
    logic shuts itself off

Press ON again:
    wakes immediately
```

This gives consumer-product behavior without much complexity.

## Off-the-Shelf Low-Voltage Disconnect

The **Victron BatteryProtect 12/24 V** family is a relevant off-the-shelf
low-voltage disconnect. Victron describes it as a device that disconnects
nonessential loads before the battery is completely discharged. ([Victron
Energy][3])

Important caution: Victron says BatteryProtect is **unidirectional**, so it
should be used either in a load path or in a charger path, not as a bidirectional
pass-through between charger and battery. ([Victron Energy][3])

For this vehicle, it could sit in the **load branch**, not the common
battery/charger path:

```text
battery -> fuse -> BatteryProtect -> buck + logic loads
battery -> fuse -> motor contactor -> BTS7960s
charger -> charge controller -> battery
```

For a learning-focused custom design, the cleaner choice is still the
LTC2955-style latch for logic power. Use BatteryProtect if proven
low-voltage-disconnect behavior is more important than designing that section.

## Vehicle Protection Stack

### 1. Fuses First

Put a fuse as close to the battery pack positive terminal as possible, then use
branch fuses:

```text
main battery fuse
motor branch fuse
logic/buck branch fuse
charger input/output fuse
```

For a kids' ride-on vehicle, use automotive-style fuses, MIDI/MEGA/ANL fuses,
or other DC-rated fusing sized to the wire gauge and expected stall current.

> **Rule:** the fuse protects the wire, not the electronics.

### 2. Manual Disconnect / Key Switch / Emergency Stop

A software shutoff is not enough. Add a physical disconnect that an adult can
operate:

```text
battery -> fuse -> key switch / service disconnect -> rest of system
```

For the motor branch, the Feather should also control a **motor contactor
enable** or at least all BTS7960 enable pins. When the charger is plugged in,
the motor branch should be impossible to energize.

### 3. Charger Input Protection

If using a DIY charger IC design, protect the DC input before it reaches the
charger controller:

```text
DC jack / charge port
   |
   fuse or resettable fuse
   |
   reverse-polarity / overvoltage / surge protection
   |
   charger circuit
   |
   battery pack
```

Good IC families to study:

**LTC4368**  
2.5-60 V operating range, OV protection to 100 V, reverse supply protection to
-40 V, back-to-back N-channel MOSFET control, and bidirectional
circuit-breaker behavior. This is the first part to study for charger input
protection. ([Analog Devices][4])

**LTC4365**  
2.5-34 V operating range, OV protection to 60 V, and reverse supply protection
to -40 V. Useful for lower-voltage systems, but tight if a "36 V" supply can be
near or above 36 V. ([Analog Devices][5])

**TPS2660**  
4.2-60 V eFuse with integrated reverse input polarity protection down to -60 V,
adjustable current limit, soft-start, UVLO/OVP, and reverse-current blocking.
([Texas Instruments][6])

**TPS2663**  
Higher-current 60 V eFuse class with up to 6 A adjustable current limit and a
low 31 milliohm integrated FET. Reverse-polarity support uses an external
N-channel FET. ([Texas Instruments][7])

Use eFuses for **logic, charger input, accessories, and display rails**. Do not
use them for raw motor stall current unless the design is carefully sized for
that current.

### 4. Reverse Polarity Protection

For the **charger input**, use MOSFET-based reverse protection instead of a
diode bridge. A diode bridge wastes power and drops voltage.

Professional approaches:

```text
P-channel MOSFET ideal diode      simple, okay at modest current
N-channel ideal diode controller  better, lower loss, more complex
eFuse / surge stopper             best integrated protection
```

For the **main 24 V battery pack**, rely on keyed connectors, fusing, and
physical layout first. High-current MOSFET reverse protection for the entire
motor bus is possible, but it becomes a serious thermal and layout design.

### 5. Motor Noise Suppression

Do **not** add a diode bridge to each motor. That is usually not what is wanted
with a bidirectional H-bridge. Flyback diodes are for simple one-direction
relay/transistor loads; an H-bridge already provides current recirculation paths
through the MOSFETs, body diodes, and driver structure.

Use this instead:

```text
0.1 uF ceramic directly across each motor's terminals
optional 0.047-0.1 uF from each terminal to motor case, if suitable
twisted motor leads
short motor leads where possible
motor wires routed far from logic/radio/analog lines
bulk capacitance near each BTS7960 power input
TVS diode across the 24 V motor bus
```

Pololu's motor-noise guidance points in this direction: capacitors across motor
terminals are usually the most effective suppression, twisted/short motor leads
help, and bulk decoupling near electronics helps protect the logic rail from
dips and resets. ([Pololu][8])

Starting point for this system:

```text
Each motor:
    100 nF ceramic across motor terminals

Each BTS7960 board:
    470 uF to 2200 uF electrolytic across VMOTOR/GND near board
    100 nF ceramic close to board supply pins if accessible

24 V bus:
    TVS diode around 33 V standoff class, sized for expected surge energy
    main bulk capacitor near motor power distribution
```

Because a 24 V lead-acid pack may be charged up around **29.4 V**, do not use a
TVS with a standoff voltage too close to normal charge voltage. Pick a part whose
working standoff is above the highest expected normal bus voltage.

### 6. Logic Supply Filtering

Add filtering before the microcontroller buck and after the buck.

A practical stack:

```text
24 V pack
  |
  fuse
  |
  reverse / surge protection
  |
  47-220 uF electrolytic
  |
  buck converter
  |
  100-470 uF low-ESR electrolytic on 5 V / 3.3 V rail
  |
  0.1 uF ceramic at every IC/module
```

If motor noise still causes resets:

```text
24 V logic branch -> ferrite bead or small inductor -> buck input
```

Also use a **star ground** approach: motor currents should not flow through the
same skinny ground path used by Feather/MCP/radio logic.

## Receiver Design I Would Actually Build

```text
24 V PACK+
   |
   |-- MAIN FUSE
   |
   |-- KEY / SERVICE SWITCH
   |
   +-- CHARGER BRANCH
   |      charge port
   |      fuse
   |      LTC4368 or equivalent input protection
   |      lead-acid charger circuit
   |      battery pack
   |
   +-- LOGIC BRANCH
   |      0.5-2 A fuse
   |      LTC2955 pushbutton power controller
   |      buck converter EN controlled by LTC2955
   |      buck output -> Feather / RFM9x / MCP23017 / sensors
   |
   +-- MOTOR BRANCH
          high-current fuse
          manual disconnect or contactor
          BTS7960 bus capacitance
          four BTS7960 drivers
```

Control signals:

```text
Feather GPIO -> BTS7960 enable pins
Feather GPIO -> motor contactor driver
Feather GPIO -> LTC2955 KILL
charger_present -> Feather input
battery voltage dividers -> Feather ADCs
```

Firmware safety:

```text
On boot:
    motors disabled
    validate radio link
    only then allow motor enable

If charger_present:
    motor contactor off
    BTS7960 enables off
    ignore drive commands

If no packet for 250-500 ms:
    command zero speed

If no packet for 30 min:
    stop motors
    save state if needed
    assert KILL
```

## About 555 Timers

A 555 is fine for blinking LEDs or simple monostables. It is not a good primary
30-minute power governor here.

For long timing, a chip like the **LTC6991 TimerBlox** is much more appropriate.
It supports timing periods from **1 ms to 9.5 hours** and is intended for
long-duration timing events, watchdog timers, and periodic wake-up applications.
([Analog Devices][9])

Another relevant chip is **TPL5111**, a 35 nA nano-power timer intended for
power-gating duty-cycled battery systems. It supports selectable intervals from
**100 ms to 7200 s** and has a manual power-on input. ([Texas Instruments][10])

That said, in this receiver, the Feather is already awake and already knows
radio activity, so the cleanest 30-minute timer is firmware plus a pushbutton
power controller.

## Transmitter Side With PowerBoost 1000C

The PowerBoost 1000C is a reasonable transmitter power module if the total 5 V
current is within its real thermal/current capability. Adafruit describes it as
a 5.2 V boost converter with a built-in LiPo charger and load sharing, capable
of running the project while charging. ([Adafruit][11])

The key detail for the desired switch behavior: the `EN` pin can be connected to
ground to completely turn off the boost output. ([Adafruit][11])

Adafruit also says it has load sharing, automatically switches to USB power when
available, and can charge-and-boost at the same time, but it must have a LiPo
attached. ([Adafruit][11])

Recommended transmitter wiring:

```text
LiPo -> PowerBoost 1000C
USB -> PowerBoost charger input

PowerBoost 5.2 V output -> main 5 V transmitter rail
PowerBoost EN -> physical ON/OFF switch to GND
```

Use the switch on **EN**, not in series with the battery. That way the
transmitter can charge whether the switch is ON or OFF.

### PowerBoost Cautions

The PowerBoost 1000C is a **1 A-class** boost/charger board. Adafruit notes it
can provide 1000 mA+ if the battery can handle it, but the charger/boost system
has limits and needs a high-quality USB supply if heavily loaded while charging.
([Adafruit][11])

The Nextion display backlight can be a meaningful part of the load. Measure
actual current with:

- Display at full brightness
- Radio transmitting
- Controller active
- Any LEDs or buzzers on

## Professional Transmitter Extras

Recommended additions:

- **LiPo protection:** use a protected LiPo pack or protected cell assembly.
  Add a small fuse or polyfuse in the battery lead if practical.
- **Power switch:** switch PowerBoost `EN`, not battery positive.
- **Display power:** consider a separate high-side load switch for Nextion
  `VCC`; firmware can dim, sleep, or turn the display off after inactivity.
- **UART level shifting:** check Nextion `TX` level. If Nextion UART is 5 V and
  Feather is 3.3 V, divide or level-shift Nextion `TX` into Feather `RX`.
- **ESD protection:** USB has some protection on the module, but external
  controls/connectors need ESD-conscious layout. Series resistors on long
  button/switch lines are useful.
- **Battery monitoring:** add LiPo voltage divider or fuel gauge, plus
  low-battery warning on screen and buzzer.
- **Human interface:** add power LED, visible charge LED, low-battery indicator,
  link-quality/no-link warning, and throttle-neutral-at-boot requirement.

Also consider **separate display backlight control**. The Nextion display may
dominate transmitter current. If the PowerBoost is near its limit, dimming or
switching the display after inactivity will matter more than optimizing
microcontroller sleep.

## What Would Make This Professional?

Prioritize these in order:

1. Main battery fuse close to the pack
2. Branch fuses for motor, charger, and logic
3. Physical key switch or service disconnect
4. Charger-present motor lockout
5. Software radio failsafe: stop motors after 250-500 ms without a valid packet
6. Auto-shutdown after 30 min through LTC2955 `KILL` / soft latch
7. Low-voltage cutoff for lead-acid pack protection
8. Battery midpoint monitoring for the two 12 V batteries
9. Motor bus TVS, motor capacitors, and twisted motor wiring
10. Bulk capacitors near BTS7960 boards
11. Reverse-polarity / OV / surge protection on charger input
12. Watchdog timer enabled in firmware
13. Temperature monitoring on motor drivers and/or battery compartment
14. Separate logic and motor grounding strategy
15. Keyed connectors, strain relief, and wire gauge sized for stall current

> **Design rule:** use hardware to make dangerous states impossible, and use
> software to make normal behavior pleasant.

Software can decide "30 minutes idle," but hardware should still enforce fusing,
charger lockout, reverse protection, and a physical off path.

## Concrete Parts Direction

| Need | Suggested Direction |
| --- | --- |
| Receiver logic power latch | LTC2955 |
| Optional long-duration timer | LTC6991 or TPL5111, though firmware timer is probably enough |
| Charger-input protection | LTC4368 if building the charger yourself |
| Logic/accessory branch eFuse | TPS2660 or TPS2663, depending on current |
| Off-the-shelf low-voltage load disconnect | Victron BatteryProtect, used only in load path |
| Motor power switching | Fuse plus manual disconnect or latching contactor, not a small SPDT relay |
| Transmitter power | PowerBoost 1000C with switch on `EN`, plus worst-case current measurement |

[1]: https://www.analog.com/en/products/ltc2955.html "LTC2955 Datasheet and Product Info | Analog Devices"
[2]: https://www.analog.com/en/products/ltc2954.html "LTC2954 Datasheet and Product Info | Analog Devices"
[3]: https://www.victronenergy.com/battery_protect/battery-protect "BatteryProtect | Victron Energy"
[4]: https://www.analog.com/en/products/ltc4368.html "LTC4368 Datasheet and Product Info | Analog Devices"
[5]: https://www.analog.com/en/products/ltc4365.html "LTC4365 Datasheet and Product Info | Analog Devices"
[6]: https://www.ti.com/product/TPS2660 "TPS2660 data sheet, product information and support | TI.com"
[7]: https://www.ti.com/product/TPS2663 "TPS2663 data sheet, product information and support | TI.com"
[8]: https://www.pololu.com/docs/0J15/9 "Pololu - 9. Dealing with Motor Noise"
[9]: https://www.analog.com/en/products/ltc6991.html "LTC6991 Datasheet and Product Info | Analog Devices"
[10]: https://www.ti.com/product/TPL5111 "TPL5111 data sheet, product information and support | TI.com"
[11]: https://www.adafruit.com/product/2465 "PowerBoost 1000C"
