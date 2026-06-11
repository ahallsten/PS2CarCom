# 🔋 Charger Planning — 24 V Lead-Acid Pack

> **TL;DR** — Charge the two 12 V lead-acid batteries as **one 24 V series pack**, monitor the
> midpoint so you can see each battery individually, and add a **24 V balancer** (or a manual
> balance routine). **Do not** leave 14.7 V/battery applied forever — that's an absorption/boost
> voltage, not a long-term float voltage.

---

## Contents

- [Charge voltage targets](#charge-voltage-targets)
- [The most practical game plan](#-the-most-practical-game-plan)
- [Why one 24 V charger is usually okay](#-why-one-24-v-charger-is-usually-okay)
- [Correcting the “14.7 V forever” idea](#-correcting-the-147-v-forever-idea)
- [Off-the-shelf plan A — one 24 V smart charger](#-off-the-shelf-plan-a--one-24-v-smart-charger)
- [Off-the-shelf plan B — two 12 V chargers](#-off-the-shelf-plan-b--two-12-v-chargers)
- [DIY plan A — a real lead-acid charger IC](#-diy-plan-a--a-real-lead-acid-charger-ic)
- [DIY plan B — CC/CV buck + MCU float switch](#-diy-plan-b--cccv-buck--mcu-controlled-float-switch)
- [Voltage measurement circuit](#-voltage-measurement-circuit)
- [Powering the MCU while plugged in](#-powering-the-microcontroller-while-plugged-in)
- [Ranked recommendations](#-my-ranked-recommendations)
- [References](#references)

---

## Charge voltage targets

For a 24 V pack made from two 12 V SLA/AGM batteries, the healthy charging targets are roughly:

| Stage                      |          Per cell | One 12 V battery |   Two in series |
| -------------------------- | ----------------: | ---------------: | --------------: |
| Absorption / cyclic charge | ~2.40–2.45 V/cell |     ~14.4–14.7 V |   ~28.8–29.4 V  |
| Float / standby            | ~2.25–2.30 V/cell |     ~13.5–13.8 V |   ~27.0–27.6 V  |

> 📖 **Power-Sonic's SLA guide:** sealed lead-acid batteries are best charged with
> **constant-voltage, current-limited charging** — fast/cyclic charge up to ~2.45 V/cell, float
> at ~2.25–2.30 V/cell. Full charge in cyclic service is reached when voltage hits
> 2.45 ± 0.05 V/cell **and** charge current falls below 0.01 C; after that, disconnect or switch
> to float. ([Power-Sonic][1])

---

## 🛠️ The most practical game plan

Use a **commercial 24 V lead-acid smart charger** across the full 24 V string, then use your
Feather only for monitoring, display/logging, interlocks, and safety shutdown. This is the
simplest architecture that doesn't sacrifice battery health.

**Wiring strategy:**

```text
                CHARGE PORT
             +---------------+
             | 24 V SLA      |
AC/DC charger| smart charger |
             +-------+-------+
                     |
                     + ---- fuse ---- PACK+
                                      |
                                  [Battery 2]
                                      |
                         MIDPOINT ----+---- ADC_MID divider
                                      |
                                  [Battery 1]
                                      |
PACK- / GND --------------------------+---- MCU GND

PACK+ ---- main fuse ---- main switch ---- 5-36 V buck ---- Feather / logic
PACK+ ---- motor fuse/contact ---- BTS7960 motor power

CHARGER_PRESENT signal ---- Feather input
Feather output ------------ disables motor EN pins while plugged in
```

The charger handles the actual chemistry, while your code watches:

```text
V_pack  = voltage from PACK+ to PACK-
V_low   = voltage from midpoint to PACK-
V_high  = V_pack - V_low
```

That gives you independent visibility into **both** 12 V batteries using only two analog inputs.

---

## ✅ Why one 24 V charger is usually okay

It's generally safe to charge the two batteries in series as a 24 V pack — **if they are the same
type, capacity, age, and have lived the same life.** Power-Sonic says lead-acid strings can be
charged in series, but warns that battery differences can make one battery overcharge while
another undercharges; use batteries of the same manufacturer, Ah rating, age, and history.
([Power-Sonic][1])

> ⚠️ **Imbalance watch-points** (from midpoint monitoring):
> - **> 0.2–0.3 V** difference during charge or rest → warning sign.
> - **≥ 0.5 V** difference → stop and investigate.

For extra battery life, add a **24 V battery balancer**. The Victron Battery Balancer is built to
equalize two series-connected 12 V batteries — it turns on above ~27 V and draws up to 1 A from
the higher-voltage battery so the two converge. ([Victron Energy][2]) A clean solution for your
exact "two 12 V batteries in series" case.

---

## ❌ Correcting the “14.7 V forever” idea

A 12 V lead-acid battery is **not** happy forever at 14.7 V — that's near the cyclic absorption
voltage. Great for charging, but once the battery is full, leaving it there causes excess current,
water decomposition, heating, aging, and in bad cases thermal runaway. Power-Sonic explicitly
warns that too-high charge voltage after full charge causes excessive current, water
decomposition, premature aging, and thermal runaway. ([Power-Sonic][1])

**The healthy algorithm:**

```text
1. Bulk:       current-limited charge, usually C/10 to C/5
2. Absorb:     hold ~29.4 V pack voltage until current tapers
3. Terminate:  when current < 0.01 C, or after a safety timeout
4. Float:      drop to ~27.0–27.6 V indefinitely
```

Initial charge current should not exceed **0.3 C** for cyclic SLA charging. ([Power-Sonic][1])
So per battery:

| Battery capacity | Max initial charge current (0.3 C) |
| ---------------- | ---------------------------------: |
| 7 Ah             |                            ~2.1 A  |
| 12 Ah            |                            ~3.6 A  |
| 18 Ah            |                            ~5.4 A  |

> 💡 For long life, pick **C/10 to C/5**, not the absolute max. A 12 Ah pack is very comfortable
> with a 2–3 A charger.

---

## 🛒 Off-the-shelf plan A — one 24 V smart charger

**This is what I would actually build into the toy car.**

| Charger | Notes |
| ------- | ----- |
| **Victron Blue Smart IP65 24 V** | 5 A / 8 A / 13 A models; configurable lead-acid/AGM/gel/flooded profiles, storage mode, temp compensation, power-supply mode. ([Victron Energy][3]) |
| **Power-Sonic ACX / A-C series** | VRLA chargers that auto-transition fast → float; aimed at cyclic apps like mobility equipment, medical devices, backup systems. ([Power-Sonic][4]) |
| **NOCO GENIUSPRO25** | Pricier / probably overkill: 6/12/24 V, temp compensation, maintenance, desulfation/repair modes, 24 V power-supply mode at 12.5 A. ([NOCO][5]) |

> 🎯 **Sweet spot:** Victron 24 V / 5 A or 8 A class for ~18–35 Ah batteries. For smaller
> 7–12 Ah SLA, use a lower-current charger or a low-power mode.

---

## 🔀 Off-the-shelf plan B — two 12 V chargers

Works very well for balancing — **but only if the two chargers have isolated outputs.**

```text
12 V charger #1 across lower battery
12 V charger #2 across upper battery
```

Charging both at once prevents one drifting from the other. **The trap:**

> ⚠️ Do **not** use two non-isolated DC buck charger modules from the same supply unless you've
> verified input-output isolation.

Many cheap buck modules share input negative and output negative. Connect two of those across a
series string and you can short the midpoint or bypass a battery. Two separate certified
AC-powered 12 V SLA smart chargers are usually isolated; two random DC-DC boards usually are not.

Attractive if you really want independent charge control, but for a vehicle/toy build it adds
wiring and more things to mount.

---

## 🔧 DIY plan A — a real lead-acid charger IC

For DIY, avoid designing the AC mains side. Use an external certified DC supply, then put the
charger electronics inside the car.

| IC | What it offers |
| -- | -------------- |
| **TI UC3906** | Classic SLA charger controller: internal 3-state logic, voltage + current control, lead-acid temp tracking, external pass device. Available in prototyping-friendly PDIP. ([Texas Instruments][6]) |
| **TI BQ24450** | Standalone lead-acid charge controller: regulates voltage + current, temp-compensated reference, float or dual-voltage float/boost, external pass transistor; datasheet shows bulk/boost/float states. ([Texas Instruments][7], [Texas Instruments][8]) |
| **ADI LTC4020** | Much more sophisticated: 55 V buck-boost multi-chemistry charger with PowerPath, lead-acid algorithms, 4.5–55 V in, up to 55 V out. Excellent for an integrated 24 V charger/power-path, but not a minimal beginner circuit. ([Analog Devices][9]) |

> 🌡️ **Thermal note:** the UC3906/BQ24450 linear-pass approach can dissipate a lot of heat
> depending on input voltage and charge current — thermal design matters. Use UC3906/BQ24450 for
> **one 12 V charger per battery**, or LTC4020 for a true integrated 24 V power-path design.

---

## ⚙️ DIY plan B — CC/CV buck + MCU-controlled float switch

The "minimal components, but still decent" DIY route.

**Bill of materials:**

```text
36 V DC supply brick, current rated appropriately
CC/CV buck converter
current sense
battery temperature sensor
relay or MOSFET/resistor network to switch voltage setpoint
MCU firmware state machine
fuse
reverse-current blocking diode or ideal diode
```

**Buck current limit:**

```text
I_charge = 0.1 C to 0.2 C
absolute max = 0.3 C
```

**MCU state machine:**

```text
if battery very low:
    precharge at low current

bulk:
    buck in current limit
    voltage rises toward 29.4 V

absorption:
    hold 29.4 V
    wait until charge current < 0.01 C
    or until safety timeout expires

float:
    switch buck feedback to 27.2–27.6 V
    stay here indefinitely
```

**Temperature compensation** is desirable, especially outside 5–35 °C. Power-Sonic lists ~-2
mV/cell/°C for float and ~-6 mV/cell/°C for cyclic, with voltage reduced at higher temperatures.
([Power-Sonic][1]) For a 24 V pack (12 cells):

```text
float compensation:      12 cells * -2 mV/C  = -24 mV/C
absorption compensation: 12 cells * -6 mV/C  = -72 mV/C
```

> ⚠️ Reasonable DIY, but **not** as foolproof as a commercial SLA smart charger. Do this only if
> you want the learning experience.

---

## 📐 Voltage measurement circuit

Use two dividers: one for full pack, one for midpoint.

For a 3.3 V ADC, a simple robust choice (divider ratio **11:1**):

```text
PACK+ ---- 100k ---- ADC_PACK ---- 10k ---- GND
                           |
                         100 nF
                           |
                          GND

MIDPOINT ---- 100k ---- ADC_MID ---- 10k ---- GND
                            |
                          100 nF
                            |
                           GND
```

| Input | Voltage | At ADC |
| ----- | ------: | -----: |
| Pack  | 32.0 V  | 2.91 V |
| Pack  | 29.4 V  | 2.67 V |
| Midpoint | 14.7 V | 1.34 V |

Then compute:

```cpp
float adcToVoltage(float adcVolts)
{
    return adcVolts * 11.0f; // for 100k / 10k divider
}

float packV = adcToVoltage(adcPackVolts);
float lowBatteryV = adcToVoltage(adcMidVolts);
float highBatteryV = packV - lowBatteryV;
float imbalanceV = highBatteryV - lowBatteryV;
```

> 💡 **Tips:** use 1% resistors, put the 100 nF cap physically close to the ADC pin, and discard
> the first ADC reading after switching channels. If the divider is permanently connected while
> the Feather is off, use high-value resistors (as above) or switch the divider with a small
> MOSFET/analog switch so it doesn't slowly drain the pack or back-power the MCU through
> protection diodes.

---

## 🔌 Powering the microcontroller while plugged in

You have two options:

- **Simple:** leave the 5–36 V buck on the pack side. When the charger is plugged in, the pack
  rises to charging/float voltage and the buck powers the Feather indefinitely. Fine if the MCU
  load is small compared with charger current.
- **Better:** feed the buck from a diode-OR / ideal-diode-OR of:

  ```text
  battery pack
  charger/DC input
  ```

  So the MCU runs whenever either the pack or charger input exists.

Also add a `CHARGER_PRESENT` signal to the Feather and **disable the BTS7960 enables whenever the
charger is plugged in**. For a kids' ride-on toy, make "plugged in" physically and electrically
prevent driving.

> ⚠️ Smart chargers can get confused by a large load on the battery while charging — they use
> charge-current taper to decide when the battery is full. Your MCU load is small, but the
> **motors must be locked out while charging.**

---

## 🏆 My ranked recommendations

| Rank | Approach | Summary |
| ---- | -------- | ------- |
| 🥇 **Best overall** | One quality **24 V SLA/AGM smart charger** + **24 V battery balancer** + two ADC dividers + charger-present detect + motor-drive lockout. | Minimal custom power electronics, excellent battery life. |
| 🥈 **Best balancing, simple parts** | Two **isolated** 12 V SLA smart chargers (one per battery) + two ADC dividers. | Electrically clean *if* outputs are truly isolated; mechanically less tidy. |
| 🥉 **Best DIY learning route** | A **UC3906/BQ24450** charger per 12 V battery, or an **LTC4020**-based integrated 24 V power-path charger. | For when you want to build the charger yourself. |
| ⚠️ **Least recommended but workable** | A generic CC/CV buck set to 29.4 V — only with current limiting, termination, float switching, temp monitoring, fuse, and timeout. | A plain 29.4 V supply left connected forever is **not** a healthy charger. |

> ⭐ **My pick for this project:** **Victron Blue Smart IP65 24 V charger + Victron Battery
> Balancer + your Feather monitoring both battery voltages.** The cleanest intersection of simple,
> safe, maintainable, and battery-friendly.

---

## References

[1]: https://www.power-sonic.com/blog/how-to-charge-a-lead-acid-battery/ "How to Charge a Lead Acid Battery | Power Sonic"
[2]: https://www.victronenergy.com/batteries/battery-balancer "Battery Balancer | Victron Energy"
[3]: https://www.victronenergy.com/chargers/blue-smart-ip65-charger "Blue Smart IP65 Charger | Victron Energy"
[4]: https://www.power-sonic.com/chargers/a-c-series/ "Power Sonic Chargers | Efficient & Reliable Energy Charging Solutions"
[5]: https://no.co/geniuspro25 "NOCO - 25A Professional Battery Charger - GENIUSPRO25"
[6]: https://www.ti.com/product/UC3906 "UC3906 data sheet, product information and support | TI.com"
[7]: https://www.ti.com/product/BQ24450 "BQ24450 data sheet, product information and support | TI.com"
[8]: https://www.ti.com/lit/ds/symlink/bq24450.pdf "Integrated Charge Controller for Lead-Acid Batteries datasheet (Rev. C)"
[9]: https://www.analog.com/en/products/ltc4020.html "LTC4020 Datasheet and Product Info | Analog Devices"

1. [How to Charge a Lead Acid Battery — Power-Sonic][1]
2. [Battery Balancer — Victron Energy][2]
3. [Blue Smart IP65 Charger — Victron Energy][3]
4. [A-C Series Chargers — Power-Sonic][4]
5. [GENIUSPRO25 — NOCO][5]
6. [UC3906 — Texas Instruments][6]
7. [BQ24450 — Texas Instruments][7]
8. [BQ24450 Datasheet (Rev. C) — Texas Instruments][8]
9. [LTC4020 — Analog Devices][9]
