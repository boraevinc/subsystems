# Heating controller: how it works and how to use it

Sketch: [heating_control/heating_control.ino](heating_control/heating_control.ino)

## What it does

It keeps the water at the temperature you ask for. The spec is any setpoint from 25 to 35 °C, held within ±0.5 °C, starting from inlet water at 10–20 °C.

It reads a thermistor to measure the temperature, and switches the heater on and off quickly (PWM) to deliver the right amount of power.

## What it's connected to

| Pin | Connected to |
|---|---|
| A0 | Thermistor voltage divider (**3V3** → 10 kΩ → A0 → thermistor → GND) |
| D9 | Heater MOSFET gate (1 kHz PWM; heater on the battery's 12 V output) |

**Board:** Arduino Nano ESP32 (3.3 V logic). The MOSFET gate is driven at 3.3 V instead of 5 V. The IRLB8748 is designed to switch at low gate voltages, but check at bring-up that the drain drops close to 0 V when the heater is fully on. The PWM runs at 1 kHz to keep the MOSFET cool.

For the full wiring, see the EEE build guide.

---

## How the code works

### The main loop

There are two timers. Neither uses `delay()`:

- **Every 100 ms:** read the temperature (`readTemperature`).
- **Every 1 s:** decide the heater power (`controlStep`), update the 60-second statistics (`updateWindow`), and print a log line (`logLine`).

Water heats slowly, so deciding once a second is plenty.

### 1. Reading the temperature

1. **Read A0 32 times in millivolts and average.** `analogReadMilliVolts()` uses the ESP32's factory ADC calibration. Averaging removes noise, much of it from the heater's PWM.
2. **Convert the voltage to the thermistor's resistance:** `R = 10 kΩ × V ÷ (3300 mV − V)`. The 3300 is `VCC_MV`, the Nano's 3V3 pin. Measure that pin once with a multimeter and put in the real value.

   The divider **must** run from 3V3, not 5 V: the Nano ESP32's analog inputs only accept 0–3.3 V.
3. **Convert resistance to temperature** with the Beta equation:

   `1/T = 1/298.15 + ln(R / 10000) / 4220`   (T in kelvin)

   4220 is the thermistor's B value, from the ND06P00103K listing.
4. **Smooth it** with a light filter.
5. **Apply your calibration:** `temp = CAL_GAIN × raw + CAL_OFFSET`.

The thermistor's resistance is only ±10% accurate, so **calibrating against a real thermometer is essential**. See "How to test" below.

Near 30 °C the divider changes by about 37 mV per °C, and the ADC resolves about 1 mV, so no op-amp is needed for the ±0.5 °C spec.

| Water temperature | Expected voltage at A0 |
|---|---|
| 10 °C | 2.24 V |
| 20 °C | 1.85 V |
| 25 °C | 1.65 V |
| 30 °C | 1.46 V |
| 35 °C | 1.28 V |
| 40 °C | 1.11 V |

### 2. Working out the heater power

The controller works in **watts**, then converts to PWM duty: `duty = watts ÷ 48 W`.

- 12 V across 3 Ω is 4 A, which is **48 W**. The element is only rated for **30 W**.
- So power is **capped at 28 W** (`P_MAX`), about 58% duty. You can't go over it, even in manual mode.

There are three ways to control it:

**PI control (`s 30`).** This is the main mode.

```
power = Kp × error + integral          (error = setpoint − temperature)
```

- The **P part** gives big power when it's cold and less as the temperature gets close.
- The **I part** learns how much power is needed to hold the setpoint against heat loss.
- The I part **only runs within 2 °C of the setpoint**. During the long warm-up the heater is at full power anyway; if the I part kept adding up during that time, the temperature would shoot way past the setpoint. That's anti-windup.
- The I part is never negative, because the heater can't cool.

**Bang-bang (`b 30`).** Heater fully on (28 W) below setpoint − 0.2 °C, fully off above setpoint + 0.2 °C. It's simple and useful as a comparison in the report.

**Manual (`m 20`).** A fixed power, with no feedback. Use it for the step test that tells you how your system behaves.

### 3. Safety

Any of these turns the heater **off** and **latches** a fault, so it stays off until you clear it:

| Fault | Trigger | Usually means |
|---|---|---|
| SENSOR | A0 below 50 mV or above 3000 mV | Thermistor unplugged or shorted |
| OVER-TEMPERATURE | Above 40 °C | Something has gone wrong; let it cool below 38 °C |
| NO TEMPERATURE RISE | Heater near full power for 5 minutes but less than 0.5 °C rise | Thermistor out of the water, heater not connected, or MOSFET dead |

- To clear a fault, send `x` or a new `s`, `b` or `m`. It's only accepted once the cause has gone, for example the sensor is reconnected or the water is below 38 °C.
- The heater is **off at power-up**. Nothing heats until you send a command.

### 4. Logging and statistics

- `l 1` prints CSV once a second: `t_ms,mode,setpoint,temp,temp_uncal,mV,power_W,duty,err,integ_W` (`mV` is the voltage at A0).
- `l 2` prints in Serial Plotter format: the setpoint, ±0.5 °C lines around it, the temperature, and the power.
- `?` shows:
  - mode, temperature, power and gains
  - calibration values
  - **time since the last setpoint change**
  - **how long the temperature has stayed within ±0.5 °C** (this gives you settling time)
  - over the last 60 s: mean, min, max, average power, and **% of time within ±0.5 °C**

### 5. Ready for the ESP32 link

`setHeatSetpoint(c)`, `setHeatBangBang(c)`, `setHeatManual(watts)`, `heaterOff()`, `getTemperature()`, `getHeatSetpoint()`, `getHeaterPower()` and `heaterFaulted()` are ready for the ESP32 link (task 2) to call.

---

## Settings you might change

These are at the top of the sketch.

| Constant | Default | What it is |
|---|---|---|
| `THERM_ON_TOP` | false | Set to true if the divider is wired the other way round |
| `VCC_MV` | 3300 | Voltage of the Nano's 3V3 pin in mV. Measure it and update |
| `R_FIXED` | 10000 | Divider resistor in ohms |
| `BETA` | 4220 | Thermistor B value |
| `CAL_GAIN`, `CAL_OFFSET` | 1.0, 0.0 | **Replace these with your calibration result** |
| `HEATER_V` | 12.0 | Change to 6.0 if the heater runs from the 6 V output |
| `Kp` | 10 | P gain, in watts per °C of error |
| `Ki` | 0.05 | I gain |
| `hyst` | 0.2 | Bang-bang half-band, in °C |
| `T_CUTOFF` | 40 | Over-temperature limit |
| `NORISE_S` | 300 | Seconds for the no-rise check |

---

## How to use it

1. Upload the sketch with Tools → Board set to **Arduino Nano ESP32** (esp32 core 3.x, Pin Numbering "By Arduino pin"). Open the Serial Monitor at **115200 baud**, line ending **Newline**.
2. Commands:

| Command | Does |
|---|---|
| `s 30` | PI control to 30 °C (`s 0` turns the heater off) |
| `b 30` | Bang-bang control to 30 °C |
| `h 0.3` | Set the bang-bang hysteresis |
| `m 20` | Fixed 20 W, no feedback |
| `kp 12` / `ki 0.08` | Change the gains live |
| `cal1 21.4` | Calibration point 1: type the **thermometer's** reading |
| `cal2 36.8` | Calibration point 2, at a different temperature |
| `cal0` | Clear the calibration |
| `l 0` / `l 1` / `l 2` | Logging off / CSV / plotter |
| `x` | Heater off (also clears a fault) |
| `?` | Full status |

---

## How to test and tune

**Safety first:** the element gets hot enough to burn. Only power it fully under water, and never touch it.

1. **Check the sensor with the heater unplugged.** Send `?`. The temperature should be near room temperature.
   - If it's wildly wrong (say −20 °C or 90 °C), the divider is wired the other way round. Set `THERM_ON_TOP = true`.
   - `?` also shows the A0 voltage. Compare it with the table under "Reading the temperature".
2. **Calibrate.**
   1. Put the thermistor and a lab thermometer side by side in room-temperature water and stir. Wait 1 minute, then send `cal1 <thermometer reading>`.
   2. Do the same in warm water (about 35–40 °C) and send `cal2 <reading>`.
   3. Copy the printed `CAL_GAIN` and `CAL_OFFSET` into the sketch.
   4. Check a third temperature to quote your accuracy.
3. **Open-loop step test.** Send `l 1` (logging to a file in CoolTerm), then `m 20`, and record about 10 minutes.
   - The slope of the temperature (°C per minute) and any delay before it starts rising describe your system. Use them with the Ziegler-Nichols method from the slides, or just as a guide for tuning by hand.
   - Then send `x`.
4. **Closed loop.** Send `s 30` and watch with `l 2`.
   - If it overshoots past 30.5 °C, lower `kp`.
   - If it stays slightly below the setpoint for a long time, raise `ki` a little.
5. **Compare** with bang-bang (`b 30`) for the report.
6. **Save** your final `Kp` and `Ki` into the sketch.

**Heating is slow.** At 28 W into about 1 litre, expect roughly 0.4 °C per minute, so 15 → 30 °C takes about 30–40 minutes. Use less water for quicker tests.

### What to record for the report

- **Calibration table:** thermometer reading against sketch reading at 2–3 temperatures, with the error at each.
- **Step test:** temperature against time at fixed power.
- **Closed-loop run:** warm-up curve, overshoot, settling time (from `?`), and "% within ±0.5 °C" once settled.
- **Setpoint change** (for example 28 → 32 °C) to show it adjusts on the fly.
- **PI against bang-bang:** the same setpoint with both, compared on overshoot and how tightly each holds.
- **Disturbance:** add a cup of cold water and show it recovers.

---

## Troubleshooting

| Problem | Likely cause |
|---|---|
| `FAULT: SENSOR` at start | Thermistor not connected, or the A0 wire is off |
| Temperature very wrong before calibration | Divider wired the other way round. Set `THERM_ON_TOP` |
| `FAULT: NO TEMPERATURE RISE` | Thermistor not in the water, heater not powered, or 12 V lead unplugged |
| Heats but very slowly | Normal for this setup. Check `?` shows power near 28 W during warm-up |
| Overshoots by more than 0.5 °C | `kp` too high, or the element is still hot after switching off. Lower `kp` |
| Temperature reading jumps when the heater switches | Grounding problem (see the EEE guide), or the 0.1 µF capacitor on A0 is missing |
| `rejected: fault still active` | The fault's cause is still there, for example water still above 38 °C |
