/*
  ph_simple.ino  -  Bioreactor pH control, simple version
  (v1 and v2 are kept unchanged in ../ph_control and ../ph_control_v2)

  Board: Arduino Nano ESP32, esp32 core 3.x, Pin Numbering "By Arduino pin".

  WIRING
    A1 <- pH board (SEN0161) output, through a divider:
            board out --[4.7k]--+-- A1
                                |
                              [10k]
                                |
                               GND
    D5 -> acid pump MOSFET gate     (pump A)
    D6 -> alkali pump MOSFET gate   (pump B)
    Each pump: +6 V -> pump -> MOSFET drain, source -> GND,
    flyback diode across the pump, 10k from gate to GND.

  HOW IT WORKS
    1. Read the pH (average of 50 readings).
    2. If pH is within DEADBAND of the setpoint: do nothing.
    3. Otherwise run ONE pump (acid if pH too high, alkali if too low)
       for  error x msPerPh  milliseconds.
    4. Wait at least MIX_MS, and until the reading stops moving (the probe
       can take ~1 min to settle), then go back to 1.

    It LEARNS the dose size. pH isn't linear: near pH 6-7 a drop of acid
    can move it a whole unit, near pH 4 the same drop barely moves it, and
    buffered liquids soak up acid. So after each dose it looks at how far
    the pH actually moved and sets the next dose from that:
        msPerPh = 0.7 x (dose time / pH change)
    (the 0.7 makes the next dose fall a little short rather than overshoot).
    If a dose barely moved the pH, the next one is doubled. Acid and alkali
    are learned separately, because the two solutions won't be equally strong.

    Safety: dosing stops if
      - the sensor reading is impossible (wire off, board unpowered),
      - a dose moved pH the wrong way (acid/alkali tubes swapped),
      - 3 full-size doses in a row did nothing (empty reservoir, blocked
        tube, probe out of the liquid, or reagent far too weak), or
      - over 2 minutes of pumping (~120 mL) for one setpoint (overshooting
        back and forth, or reagent too weak for this liquid).
    Commands typed while a pump is running are handled once it stops
    (a dose is at most ~10.6 s).

  WHAT IT CAN AND CAN'T DO (tested against a titration model)
    Works: tap water or culture medium, with reagent around 0.1-0.5 M.
    Can't: hold pH 6-7 in DISTILLED water. With nothing to buffer it, the
    pumps' smallest dose (~0.4 mL) moves the pH by 1-4 units. Use tap water
    or medium, or much weaker reagent, if you need 6-7.

  SERIAL MONITOR (115200 baud, "Newline")
    s 5       hold pH at 5 (allowed 3-7)
    x         stop dosing  (s 0 does the same)
    pa 2000   run acid pump for 2000 ms   (priming / testing)
    pb 2000   run alkali pump for 2000 ms
  It prints "pH:...,Setpoint:...,Volts:..." every second (Serial
  Plotter works too).

  CALIBRATION
    Probe in pH 7 buffer, wait 1 min, note "Volts". Rinse, pH 4 buffer,
    wait, note "Volts". Type both numbers below and upload again.
*/

// ---------- Pins ----------
const int PIN_PH   = A1;
const int PIN_ACID = D5;
const int PIN_BASE = D6;

// ---------- Calibration (edit these) ----------
const float VOLTS_AT_PH7 = 2.00;   // DFRobot typical values; measure yours
const float VOLTS_AT_PH4 = 1.14;

const float DIVIDER = 10.0 / (4.7 + 10.0);   // the 4.7k / 10k divider on A1

// ---------- Control settings ----------
const float DEADBAND    = 0.2;      // pH: close enough, don't dose
const float START_MS_PER_PH = 1000; // first guess at dose time per pH unit; the sketch learns the real value
const float MAX_MS_PER_PH   = 60000;
const unsigned long DOSE_MIN_MS = 200;
const unsigned long DOSE_MAX_MS = 10000;
const unsigned long MIX_MS      = 30000;   // wait at least this long after each dose...
const unsigned long MIX_MAX_MS  = 120000;  // ...and longer (up to this) until the reading stops moving
const float STEADY_PH = 0.03;              // "stopped moving" = changed less than this in 5 s
const int   MAX_NO_EFFECT   = 3;    // full-size doses in a row that did nothing -> stop
const unsigned long MAX_PUMP_MS_PER_RUN = 120000;   // total pumping per "s" command (~120 mL)

// ---------- Pump settings ----------
const int PUMP_SPEED = 200;         // PWM 0-255
const int RAMP_MS    = 300;         // soft start/stop (protects pump gears)

// ---------- State ----------
float setpoint = 0;                 // 0 = not controlling
float msPerPh[2] = {START_MS_PER_PH, START_MS_PER_PH};   // learned: [0] acid, [1] alkali
unsigned long lastDoseMs = 0;
unsigned long pumpMsThisRun = 0;
int   noEffectCount = 0;
bool  mixing = false;
unsigned long doseEndTime = 0;
int   lastPumpPin = -1;             // pump used for the last dose (-1 = none yet)
float phBeforeDose = 0;
unsigned long lastPrintTime = 0;
float phFiveSecAgo = 0;
unsigned long lastSteadyCheck = 0;
bool  steady = false;               // pH changed less than STEADY_PH in the last 5 s

// Average 50 readings (~0.1 s) and undo the divider: volts at the pH board.
float readVolts() {
  long sum = 0;
  for (int i = 0; i < 50; i++) {
    sum += analogReadMilliVolts(PIN_PH);
    delay(2);
  }
  return sum / 50.0 / 1000.0 / DIVIDER;
}

// Straight line through the two calibration points.
float voltsToPH(float volts) {
  float phPerVolt = (7.0 - 4.0) / (VOLTS_AT_PH7 - VOLTS_AT_PH4);
  return 7.0 + (volts - VOLTS_AT_PH7) * phPerVolt;
}

// Run one pump: ramp up, hold, ramp down. Only one pump ever runs at a time,
// because this function doesn't return until the pump has stopped.
void runPump(int pin, unsigned long ms) {
  for (int i = 1; i <= 10; i++) { ledcWrite(pin, PUMP_SPEED * i / 10); delay(RAMP_MS / 10); }
  delay(ms);
  for (int i = 9; i >= 0; i--)  { ledcWrite(pin, PUMP_SPEED * i / 10); delay(RAMP_MS / 10); }
}

void stopControl(const char* reason) {
  setpoint = 0;
  mixing = false;
  ledcWrite(PIN_ACID, 0);
  ledcWrite(PIN_BASE, 0);
  Serial.print("# STOPPED: ");
  Serial.println(reason);
}

void handleSerial() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  line.toLowerCase();
  float value = line.substring(line.indexOf(' ') + 1).toFloat();

  if (line.startsWith("s ") && value >= 3 && value <= 7) {
    setpoint = value;
    pumpMsThisRun = 0;
    noEffectCount = 0;
    mixing = false;
    Serial.print("# setpoint ");
    Serial.println(setpoint);
  } else if (line == "x" || line == "s 0") {
    stopControl("by command");
  } else if ((line.startsWith("pa ") || line.startsWith("pb ")) && value >= 1 && value <= 10000) {
    setpoint = 0;                                  // manual run stops automatic control
    int pin = line.startsWith("pa") ? PIN_ACID : PIN_BASE;
    Serial.println(pin == PIN_ACID ? "# acid pump" : "# alkali pump");
    runPump(pin, (unsigned long)value);
    Serial.println("# pump done");
  } else {
    Serial.println("# commands: s <3-7> | x | pa <ms> | pb <ms>   (pump time max 10000)");
  }
}

void setup() {
  // Pumps off first.
  pinMode(PIN_ACID, OUTPUT);
  pinMode(PIN_BASE, OUTPUT);
  digitalWrite(PIN_ACID, LOW);
  digitalWrite(PIN_BASE, LOW);
  ledcAttach(PIN_ACID, 1000, 8);    // 1 kHz PWM, duty 0-255
  ledcAttach(PIN_BASE, 1000, 8);
  ledcWrite(PIN_ACID, 0);
  ledcWrite(PIN_BASE, 0);

  Serial.begin(115200);
  delay(1500);                      // let the USB serial connect
  Serial.setTimeout(100);
  Serial.println("# pH controller (simple). Pumps OFF. Send s 5 to start.");
}

void loop() {
  handleSerial();

  float volts = readVolts();
  float ph = voltsToPH(volts);
  bool sensorOk = (volts > 0.05 && volts < 4.3 && ph > 0 && ph < 14);

  // Print once a second.
  if (millis() - lastPrintTime >= 1000) {
    lastPrintTime = millis();
    Serial.print("pH:");         Serial.print(ph, 2);
    Serial.print(",Setpoint:");  Serial.print(setpoint, 2);
    Serial.print(",Volts:");     Serial.println(volts, 3);
  }

  // Every 5 s: is the reading still moving? (The probe takes up to ~1 min to settle.)
  if (millis() - lastSteadyCheck >= 5000) {
    lastSteadyCheck = millis();
    steady = fabs(ph - phFiveSecAgo) < STEADY_PH;
    phFiveSecAgo = ph;
  }

  if (setpoint == 0) return;                          // not controlling

  if (!sensorOk) { stopControl("pH sensor reading is impossible (check wiring/probe)"); return; }

  // Let the last dose mix in, and wait for the reading to settle before judging it.
  unsigned long sinceDose = millis() - doseEndTime;
  if (mixing && (sinceDose < MIX_MS || (!steady && sinceDose < MIX_MAX_MS))) return;
  if (!mixing && !steady) return;                     // don't start a dose while pH is still moving

  // Dose finished mixing: alkali should have raised pH, acid lowered it.
  if (mixing) {
    mixing = false;
    int p = (lastPumpPin == PIN_BASE) ? 1 : 0;
    float moved = (p == 1) ? ph - phBeforeDose : phBeforeDose - ph;   // + = the right way
    if (moved < -0.2) { stopControl("last dose moved pH the WRONG way (acid/alkali tubes or pumps swapped?)"); return; }
    if (moved > 0.05) {
      // Learn: this dose moved pH by 'moved', so 1 pH needs lastDoseMs / moved.
      // Use 70% of that so the next dose falls a little short rather than overshooting.
      msPerPh[p] = constrain(0.7 * lastDoseMs / moved, (float)DOSE_MIN_MS, MAX_MS_PER_PH);
      noEffectCount = 0;
    } else {
      msPerPh[p] = min(msPerPh[p] * 2, MAX_MS_PER_PH);   // barely moved: try a bigger dose
      if (lastDoseMs >= DOSE_MAX_MS && ++noEffectCount >= MAX_NO_EFFECT) {
        stopControl("full-size doses have no effect (reservoir empty? tube blocked? probe out of the liquid? reagent too weak?)");
        return;
      }
    }
  }

  float error = setpoint - ph;
  if (fabs(error) <= DEADBAND) return;                // close enough

  if (pumpMsThisRun >= MAX_PUMP_MS_PER_RUN) {
    stopControl("used too much reagent for this setpoint (overshooting back and forth? reagent too weak?)");
    return;
  }

  int pin = (error > 0) ? PIN_BASE : PIN_ACID;        // pH too low -> add alkali
  int p = (pin == PIN_BASE) ? 1 : 0;
  unsigned long doseMs = constrain((unsigned long)(fabs(error) * msPerPh[p]), DOSE_MIN_MS, DOSE_MAX_MS);
  Serial.print(pin == PIN_BASE ? "# dosing alkali " : "# dosing acid ");
  Serial.print(doseMs);
  Serial.println(" ms");

  phBeforeDose = ph;
  lastPumpPin = pin;
  runPump(pin, doseMs);
  lastDoseMs = doseMs;
  pumpMsThisRun += doseMs;
  mixing = true;
  doseEndTime = millis();
}
