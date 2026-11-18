# pH controller: how it works and how to use it

Sketch: [ph_control/ph_control.ino](ph_control/ph_control.ino)

## What it does

It keeps the liquid at the pH you ask for. The spec is any setpoint from 3 to 7, with a default of 5.

It reads the pH probe, then adds small doses of acid or alkali with two pumps until the pH is right.

## What it's connected to

| Pin | Connected to |
|---|---|
| A1 | pH board output (DFRobot SEN0161), **through a 4.7 kΩ / 10 kΩ divider** |
| D5 | Acid pump MOSFET gate (pump A, on the 6 V output) |
| D6 | Alkali pump MOSFET gate (pump B, on the 6 V output) |

**Board:** Arduino Nano ESP32 (3.3 V logic, pins not 5 V tolerant). The pH board runs from 5 V (the Nano's VBUS pin) and can output up to about 4 V at high pH. The divider scales that down to a safe level for A1, and the sketch undoes it in software.

For the full wiring, see the EEE build guide.

---

## How the code works

### The main loop

There are three timers, and none of them uses `delay()`:

- **Every 20 ms:** take one reading from A1 and store it in a list of the last 40 readings.
- **Every 10 ms:** update the running pump, ramping it up, holding it, or ramping it down.
- **Every 1 s:** work out the pH, decide whether to dose, update the 60-second statistics, and print a log line.

### 1. Reading the pH

1. **Take a robust average.** Sort the last 40 readings (0.8 s worth), throw away the 10 highest and 10 lowest, and average the rest. Spikes from the pumps switching don't affect it.
2. **Convert to the board's output voltage.** Each reading is in millivolts at A1, from `analogReadMilliVolts()`, which uses the ESP32's factory ADC calibration. Dividing by the divider ratio (0.68) gives the voltage the pH board actually put out: `V = A1 mV ÷ 1000 ÷ 0.68`.
3. **Convert volts to pH** with your calibration line: `pH = CAL_SLOPE × V + CAL_INTERCEPT`. Before you calibrate, the defaults are 3.5 and 0, which is the formula from DFRobot's sample code.
4. **Optional temperature correction.** The probe's response changes slightly with temperature. Tell the sketch the liquid temperature with `t <°C>` and it corrects for it. In the combined sketch, the heating subsystem will supply this automatically.

### 2. Deciding when to dose ("dose and wait")

PI control would go wrong here. After a dose, the pH only changes once the liquid has **mixed**, which takes several seconds. A PI loop would keep pumping during that time and badly overshoot. So the sketch works like a careful person with a dropper:

```
      ┌────────────► READY: measure pH
      │                  │
      │      within ±0.15 of setpoint? ── yes ──► do nothing, check again in 1 s
      │                  │ no
      │                  ▼
      │      DOSING: run ONE pump for a time proportional to the error
      │              (0.2 s minimum, 3 s maximum)
      │                  │
      │                  ▼
      └──────── MIXING: wait 15 s for it to mix in
```

- **Which pump:** pH too low → alkali (pump B). pH too high → acid (pump A).
- **How long:** `dose time = Kdose × error`. With `Kdose = 1000`, an error of 0.5 pH gives a 0.5 s dose.
- **Dead band (±0.15):** no dosing when close enough. This stops it constantly pumping tiny amounts.
- **Reverse band (0.3):** switching reagent needs a bigger error. So if a dose of alkali overshoots slightly, the sketch won't immediately add acid, and the two pumps don't fight.

### 3. Protecting the pumps

- **Soft start:** every pump run ramps up over 0.3 s and ramps down at the end. Moodle warns that starting at full power shears the gears.
- **Interlock:** the two pumps can never run at the same time.
- **Stirring interlock:** `stirringRunning()` decides whether dosing is allowed. It always returns true in this standalone sketch. In the combined sketch it will only allow dosing while the stirrer is running, because without stirring a dose won't mix.

### 4. Safety

Any of these stops the pumps and **latches** a fault until you clear it:

| Fault | Trigger | Usually means |
|---|---|---|
| SENSOR | A1 below 30 mV or above 3000 mV, or pH outside 0–14 | Probe or board unplugged |
| RUNAWAY | 15 doses in a row in the same direction without moving the pH by at least 0.1 | Empty reservoir, blocked tube, dead pump, or probe out of the liquid |

- Clear a fault with `x` or a new `s`.
- Pumps are **off at power-up**. The sketch only monitors until you send `s 5`.
- The sensor fault only applies in automatic mode. Manual pump tests (`pa`, `pb`) still work with the probe unplugged or in its storage bottle.

### 5. Logging and statistics

- `l 1` prints CSV once a second: `t_ms,mode,state,setpoint,pH,volts,acid_pwm,base_pwm,acid_mL,base_mL,temp_C`.
- `l 2` prints in Serial Plotter format: the setpoint, the dead-band lines, the pH, and the acid and alkali pumps as 0/1.
- `?` shows:
  - mode, state, pH and voltage
  - all settings and the calibration
  - **total acid and alkali dosed in mL**
  - over the last 60 s: mean, min and max pH, and **% of time inside the dead band**

### 6. Ready for the ESP32 link

`setPHSetpoint(p)`, `stopDosing()`, `getPH()`, `getPHSetpoint()`, `phFaulted()`, `getAcidML()`, `getBaseML()` and `setPHTemperature(c)` are ready for the ESP32 link (task 2) to call.

---

## Settings you might change

These are at the top of the sketch.

| Constant | Default | What it is |
|---|---|---|
| `PH_DIV_TOP`, `PH_DIV_BOTTOM` | 4700, 10000 | The divider resistors on A1. Set `PH_DIV_TOP` to 0 if no divider is fitted (only safe if the board can never output more than 3.3 V) |
| `CAL_SLOPE`, `CAL_INTERCEPT` | 3.5, 0.0 | **Replace these with your calibration result** |
| `FLOW_ML_PER_S` | 1.0 | Pump flow rate. **Measure it** (see below) |
| `pumpPwm` | 200 | Pump running speed (0–255) |
| `deadband` | 0.15 | No dosing within ± this many pH units |
| `reverseBand` | 0.30 | Error needed to switch reagent |
| `Kdose` | 1000 | Milliseconds of pumping per pH unit of error |
| `MIX_MS` | 15000 | Mixing wait after each dose |
| `DOSE_MIN_MS`, `DOSE_MAX_MS` | 200, 3000 | Shortest and longest single dose |

---

## How to use it

1. Upload the sketch with Tools → Board set to **Arduino Nano ESP32** (esp32 core 3.x, Pin Numbering "By Arduino pin"). Open the Serial Monitor at **115200 baud**, line ending **Newline**.
2. Commands:

| Command | Does |
|---|---|
| `s 5` | Automatic control to pH 5 (`s 0` stops dosing) |
| `db 0.2` | Set the dead band |
| `rb 0.4` | Set the reverse band |
| `kd 800` | Set the dose length per pH unit |
| `mix 20` | Set the mixing wait, in seconds |
| `pa 2000` | Run the acid pump once for 2000 ms |
| `pb 2000` | Run the alkali pump once for 2000 ms |
| `pwm 180` | Set the pump speed |
| `t 30` | Tell the sketch the liquid is at 30 °C |
| `cal 7` | Calibration point: probe is in a pH 7 buffer |
| `cal0` | Clear the calibration points |
| `l 0` / `l 1` / `l 2` | Logging off / CSV / plotter |
| `x` | Stop the pumps (also clears a fault) |
| `?` | Full status |

A manual pump command (`pa` or `pb`) interrupts automatic control. Send `s 5` again afterwards.

---

## How to test and tune

1. **Sensor check:** with the probe in its storage bottle, send `?`. You should see `sensor=ok` and a steady voltage.
2. **Calibrate.** Use at least two buffers; pH 4, 7 and 10 is ideal.
   1. Rinse the probe in distilled water. Put it in the pH 7 buffer, wait about 1 minute, and send `cal 7`.
   2. Rinse, put it in the pH 4 buffer, wait, and send `cal 4`. Optionally rinse again and do `cal 10`.
   3. Copy the printed `CAL_SLOPE` and `CAL_INTERCEPT` into the sketch.
   4. A slope near 3.5 is normal. The sketch warns you if it's far off, which means an old or dirty probe, or mixed-up buffers.
   5. Check another buffer to quote your accuracy.
3. **Pumps.** With the tubes in water:
   1. Send `pa 2000`. Only pump A should run, starting smoothly, with water moving towards the vessel.
   2. Do the same with `pb 2000` for pump B.
4. **Flow rate:** send `pa 5000` into a measuring cylinder. mL ÷ 5 = mL per second. Put that in `FLOW_ML_PER_S` so the logs show real volumes.
5. **Closed loop in a test beaker.** Use the diluted solutions the TAs prepare, never the raw bottles.
   1. Make sure the stirrer is running. Send `s 5` and `l 2`.
   2. Step the setpoint 5 → 4 → 6.
   3. If one dose shoots past the setpoint, lower `kd`.
   4. If it creeps there very slowly, raise `kd`.
   5. If it reacts to a dose only after the next dose has already started, raise `mix`.
6. **Save** your final values into the sketch.
7. **Put the storage bottle back on the probe** when you finish. A dry probe gets damaged.

### What to record for the report

- **Calibration:** buffer pH against voltage, the fitted line, and the error on a check buffer.
- **Flow rates** of both pumps.
- **Step responses:** pH against time for 5 → 4 and 4 → 6, with the doses marked (the CSV has the pump columns).
- **Steady state:** "% in band" and the min/max pH once settled.
- **Reagent used:** mL of acid and alkali per setpoint change.
- **Your chosen tolerance:** the spec doesn't give one for pH, so justify yours. The ±0.15 dead band is a starting point.

---

## Troubleshooting

| Problem | Likely cause |
|---|---|
| `sensor=BAD` | Board not powered, output not on A1, or probe not connected to the board |
| Reading drifts or is stuck | Probe dry or dirty. Rinse, soak in the storage solution, recalibrate |
| Reading jumps when a pump switches | Grounding problem (see the EEE guide). Keep the board wires away from the pump wires |
| pH moves the wrong way after a dose | Pumps or tubes swapped. Pump A must be acid on D5 |
| `FAULT: RUNAWAY` | Reservoir empty, tube blocked, pump not running, or probe out of the liquid |
| Pump runs backwards | Swap the two wires at the pump |
| Keeps overshooting back and forth | `kd` too high or `mix` too short. Also try a larger `rb` |
| `rejected: pH sensor reading is bad` | The sketch won't start dosing without a valid reading |
