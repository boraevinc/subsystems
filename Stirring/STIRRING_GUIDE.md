# Stirring controller: how it works and how to use it

Sketch: [stirring_control/stirring_control.ino](stirring_control/stirring_control.ino)

## What it does

It keeps the propeller turning at the speed you ask for. The spec is any setpoint from 500 to 1500 RPM, held within ±20 RPM.

It counts pulses from the Hall sensor to measure the speed, then adjusts the motor power to correct it. It does this 100 times a second.

## What it's connected to

| Pin | Connected to |
|---|---|
| D2 | Hall sensor signal (70 pulses per turn of the output shaft) |
| D10 | Motor MOSFET gate (PWM, 0–1023 = 0–5 V) |
| D13 | On-board LED, blinks once per turn |

For the full wiring, see the EEE build guide.

---

## How the code works

### The main loop

Every 10 ms the sketch does four things:

1. **Measure the speed** (`measureSpeed`)
2. **Work out the motor power** (`controlStep`)
3. **Update the 1-second statistics** (`updateWindow`)
4. **Print a log line** every 10th time, so 10 times a second (`logLine`)

Between those 10 ms ticks it checks the Serial Monitor for commands you've typed. There's no `delay()` anywhere, so nothing ever freezes.

### 1. Measuring speed

- Each time a pulse arrives, an **interrupt** (`hallISR`) runs. It only counts the pulse and notes the time.
  - It ignores pulses that arrive impossibly fast (faster than 1800 RPM), because those are electrical noise.
- Every 10 ms, `measureSpeed()` asks two questions: how many new pulses arrived, and how much time passed between the first and the last of them? Then:

  `RPM = pulses ÷ time × 60 ÷ 70`

- If no pulse arrives for 100 ms, the speed is set to 0. Until then, the reading drops smoothly instead of freezing at the last value.

> **Why this differs from the starter code:** the starter sketch did float maths inside the interrupt, and read that value without pausing interrupts. Very occasionally that produces a garbage reading. Here the interrupt does almost nothing, and the main loop copies its values safely.

### 2. Working out the motor power (the controller)

Motor power is a PWM value from 0 to 1023. Three parts are added together:

```
PWM = feedforward  +  Kp × error  +  integral
       (a guess)       (P part)      (I part)
```

- **error** = target speed − measured speed.
- **Feedforward:** a first guess from a straight line, `ffGain × RPM + ffOffset`. It gets the motor close to the right speed immediately, so the PI part only has to fix small errors. You get the numbers from the **sweep** (see below).
- **P part:** reacts to the current error. A bigger error means a bigger correction.
- **I part:** adds up the error over time. It removes the small leftover error that P alone can't fix.
  - It stops adding up while the motor is already at full or zero power. This is called **anti-windup**, and it stops a big overshoot afterwards.

There are also three protections:

- **Setpoint ramp:** when you change the setpoint, the target moves towards it at 500 RPM per second instead of jumping. This protects the gears, as Moodle warns.
- **Smooth switching:** going from manual mode to automatic doesn't make the motor jump.
- **Stall protection:** if the power is high (500 or more) but no pulses arrive for 1.5 s, the motor turns off and the sketch reports `FAULT: stall`.

```
setpoint ─► ramp ─► [ feedforward + P + I ] ─► PWM ─► MOSFET ─► motor
                         ▲                                        │
                         └──────────── measured RPM ◄── Hall sensor
```

### 3. Modes

| Mode | What it means |
|---|---|
| OFF | Motor stopped (this is the state at power-up) |
| AUTO | Closed-loop speed control (`s` command) |
| MANUAL | Fixed motor power, no feedback (`m` command) |
| SWEEP | Automatic open-loop test (`w` command) |
| FAULT | A stall was detected. The motor stays off until you send `x`, `s` or `m` |

### 4. Logging and statistics

- `l 1` prints CSV 10 times a second: `t_ms,mode,setpoint,ref,rpm,rpm_avg,pwm,err`. Capture it to a file with CoolTerm and plot it in Excel.
- `l 2` prints in Serial Plotter format: the setpoint, ±20 RPM lines around it, the RPM, a smoothed RPM, and the PWM.
- `?` shows the last second's mean, min and max RPM, the average duty cycle, and **% of time within ±20 RPM**. That last figure is your spec evidence.

### 5. Ready for the ESP32 link

`setStirSetpoint(rpm)`, `setStirManual(pwm)`, `stopMotor()`, `getStirRPM()`, `getStirSetpoint()` and `getStirPWM()` are ready for the ESP32 link (task 2) to call.

---

## Settings you might change

These are at the top of the sketch.

| Constant | Default | What it is |
|---|---|---|
| `PIN_HALL`, `PIN_MOTOR` | 2, 10 | Pins. Change these if the EEE side wires it differently |
| `Kp` | 0.5 | P gain (PWM per RPM of error) |
| `Ki` | 3.0 | I gain |
| `ffGain`, `ffOffset` | 0.9, 80 | Feedforward line. **Replace these with the sweep result** |
| `rampRate` | 500 | How fast the target moves, in RPM per second |
| `SP_MIN`, `SP_MAX` | 500, 1500 | Allowed setpoint range |
| `STALL_PWM` | 500 | Power above which the shaft must be turning |

---

## How to use it

1. Open the sketch in the Arduino IDE. Choose the board (Arduino Uno, or Arduino Nano) and the port, then upload.
   - On a Nano, if the upload fails, try Processor → "ATmega328P (Old Bootloader)".
2. Open the Serial Monitor. Set it to **250000 baud** and line ending **Newline**.
3. Type commands:

| Command | Does |
|---|---|
| `s 800` | Hold 800 RPM (`s 0` ramps down and stops) |
| `m 400` | Fixed power 400 out of 1023, no feedback |
| `w` | Run the open-loop sweep (about 1 minute) |
| `kp 0.6` / `ki 4` | Change the gains live |
| `ff 0.85 95` | Set the feedforward gain and offset |
| `r 300` | Set the ramp rate (RPM/s) |
| `l 0` / `l 1` / `l 2` | Logging off / CSV / plotter |
| `x` | Stop now (also clears a fault) |
| `?` | Status, gains and the last second's statistics |

---

## How to test and tune

1. **First spin:** send `m 300`. The shaft should turn and the LED should blink once per turn. Send `?` to see the RPM, then `x`.
2. **Sweep:** send `w` and wait about a minute.
   - It steps the power from 0 to 1023 and prints `SWEEP,pwm,rpm` lines.
   - At the end it prints your **real maximum RPM** and a suggested line like `ff 0.8731 92.4`. Send that line.
   - Save the table, because it's the motor's open-loop characteristic for the report.
3. **Tune:** send `l 2` and open the Serial Plotter. Send `s 600`, then `s 900`, and watch.
   - Send `ki 0`, then raise `kp` until the speed starts to oscillate. Halve that value.
   - Raise `ki` until the leftover error disappears within about 1 s, without big overshoot.
4. **Check the spec:** at a few setpoints (500, 750, 1000), wait about 5 s and send `?`. You want "in+/-20" close to 100%.
5. **Tachometer:** compare the sketch's RPM with the tachometer at 2–3 speeds. Book the 15-minute slot after tuning.
6. **Save:** copy your final `Kp`, `Ki`, `ffGain` and `ffOffset` into the defaults at the top of the sketch.

### What to record for the report

- **Sweep table:** a graph of PWM against RPM, showing the real speed range.
- **Step responses:** CSV logs for 500→1000 and 1000→500. From them, read off rise time, overshoot and settling time.
- **Steady-state accuracy:** "% within ±20 RPM" and the min/max at each setpoint.
- **Tachometer comparison:** sketch RPM against tachometer RPM in a small table.

---

## Troubleshooting

| Problem | Likely cause |
|---|---|
| `FAULT: stall` straight away | Motor not wired, MOSFET wrong way round, or the shaft is jammed |
| RPM shows 0 but the shaft turns | Hall signal isn't reaching D2, or the sensor isn't powered |
| RPM jumps around a lot | Long or untwisted sensor wire. Ask EEE for the 4.7 kΩ pull-up and 1 nF filter |
| Can't reach the setpoint | The motor's maximum is lower than the setpoint (check the sweep). The PWM will sit at 1023 |
| Oscillates around the setpoint | `kp` or `ki` too high. Lower them |
| Arduino resets when the motor starts | Motor drawing too much from USB. Check the 470 µF capacitor and the motor current |
| Nothing in the Serial Monitor | Wrong baud rate. It must be 250000 |
