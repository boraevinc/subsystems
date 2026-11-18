/*
  ph_control.ino
  Bioreactor - pH subsystem: closed-loop pH control by acid/alkali dosing

  Board : Arduino Nano ESP32 (ESP32-S3, 3.3 V logic). WiFi is NOT used: the
          separate Espressif ESP32 does networking over a wired UART link.
          Arduino IDE: Boards Manager "esp32 by Espressif Systems" 3.x,
          board "Arduino Nano ESP32", Pin Numbering "By Arduino pin (default)".
  Spec  : hold pH at a setpoint in 3-7 (default 5). Setpoint adjustable on
          the fly (Serial Monitor now; the ESP32 link calls setPHSetpoint()).

  Hardware:
    - PH-100ATC probe + DFRobot SEN0161 board (powered from VBUS 5 V, GND,
      analog out). Board output is roughly linear in pH (DFRobot sample code
      uses pH = 3.5 * V + offset); the real line comes from calibration.
      The board can output up to ~4 V at high pH but the Nano ESP32's analog
      inputs only accept 0-3.3 V, so its output goes to A1 through a divider:
        SEN0161 out --- 4.7k ---+--- A1
                                |
                               10k --- GND        (ratio 0.68)
      The sketch undoes the divider (PH_DIV_TOP / PH_DIV_BOTTOM), so the
      calibration constants are in board volts. Set PH_DIV_TOP = 0 if no
      divider is fitted. Readings use analogReadMilliVolts() (factory ADC
      calibration), so no VREF constant is needed.
    - Two 6 V / 1 A peristaltic pumps, each on a logic-level N-MOSFET
      (IRLB8748) with a 1N4001 flyback diode, powered from the 6 V rail:
        acid pump   -> D5     alkali pump -> D6
      8-bit LEDC PWM at 1 kHz (0-255), gates driven at 3.3 V.

  Why "dose and wait" rather than PI:
    pH reacts to a dose only after the liquid has mixed (seconds of dead
    time), and the titration curve is very non-linear. A continuous PI loop
    would keep pumping during the dead time and overshoot. So:
      1. Measure (trimmed mean of 40 samples over 0.8 s).
      2. If |error| > deadband: run ONE pump for a time proportional to the
         error (clamped between DOSE_MIN_MS and DOSE_MAX_MS).
      3. Wait MIX_MS for the dose to mix in, then go back to 1.
    Reversing direction (acid after alkali or vice versa) needs a larger
    error (reverseBand) so the two pumps don't fight each other.

  Pump protection (Moodle: "power should be ramped up steadily ... gears will
  shear, this is especially critical for the pumps"):
    - Every pump start ramps up over PUMP_RAMP_MS and ramps down at the end.
    - The two pumps can never run at the same time.

  Safety (pumps off and FAULT latched):
    - Sensor: voltage at a rail or reading outside pH 0-14.
    - Runaway: MAX_RUN doses in a row in the same direction without pH
      moving toward the setpoint by RUN_MIN_PROGRESS -> empty reservoir,
      blocked tube, dead pump or probe out of the liquid.
    - Optional interlock: only dose while the stirrer is running
      (stirringRunning(); always true in this standalone sketch).
    Pumps start OFF at power-up (monitor only); send "s 5" to start.

  Temperature compensation (optional):
    The probe's slope scales with absolute temperature (Nernst). The sketch
    corrects about pH 7 using tempC (set with "t <C>"; in the merged sketch
    the heating subsystem's reading feeds it). Calibrate at a known tempC.

  ---------------------------------------------------------------------------
  Serial Monitor: 115200 baud, line ending "Newline"
    s <pH>       automatic control to setpoint (3-7), 0 = stop dosing
    db <pH>      deadband (no dosing within +/- this)
    rb <pH>      reverse band (error needed to switch reagent)
    kd <ms/pH>   dose time per pH unit of error
    mix <s>      mixing wait after each dose
    pa <ms>      run the ACID pump once for <ms> (priming / flow test)
    pb <ms>      run the ALKALI pump once for <ms>
    pwm <0-255>  pump running speed
    t <C>        liquid temperature for compensation
    cal <pH>     add a calibration point: probe in buffer of this pH
    cal0         clear calibration points (back to defaults)
    l <0|1|2>    logging: 0 off, 1 CSV (1 Hz), 2 Serial Plotter (1 Hz)
    x            stop pumps immediately (also clears a fault)
    ?            status, settings, calibration, last 60 s statistics

  Calibration (do this first, at least 2 buffers, ideally pH 4, 7 and 10):
    1. Rinse the probe in distilled water, dip in the pH 7 buffer, wait
       ~1 min for the reading to settle, send "cal 7".
    2. Rinse, pH 4 buffer, wait, "cal 4".  (Optional: rinse, pH 10, "cal 10".)
    3. Copy the printed CAL_SLOPE / CAL_INTERCEPT into the defaults below.
    4. Check another buffer to quote accuracy. Put the storage bottle back
       on the probe afterwards so it doesn't dry out.

  Tuning:
    1. "pa 5000" into a measuring cylinder to get each pump's flow rate;
       put it in FLOW_ML_PER_S (gives dosed volume in the logs).
    2. "s 5", "l 2". Step the setpoint 5 -> 4 -> 6.
    3. If one dose overshoots past the setpoint: lower kd. If it creeps up
       slowly: raise kd. If it reacts to a dose after the next one has already
       started: raise mix.
*/

#if !defined(ARDUINO_ARCH_ESP32)
#error "This sketch targets the Arduino Nano ESP32. Select it under Tools > Board."
#endif

// ============================ Pins ============================
const uint8_t PIN_PH    = A1;     // ADC1 channel, through the 4.7k/10k divider
const uint8_t PIN_ACID  = D5;
const uint8_t PIN_BASE  = D6;

// ============================ PWM (LEDC) ============================
const uint32_t PUMP_PWM_HZ   = 1000;
const uint8_t  PUMP_PWM_BITS = 8;     // 0-255
const uint8_t  ACID_PWM_CH   = 2;     // channels only used by esp32 core 2.x
const uint8_t  BASE_PWM_CH   = 3;

// ============================ Sensor ============================
const float PH_DIV_TOP    = 4700.0;   // divider resistor from board output to A1 (0 = none)
const float PH_DIV_BOTTOM = 10000.0;  // divider resistor from A1 to GND
const float PH_DIV = PH_DIV_BOTTOM / (PH_DIV_TOP + PH_DIV_BOTTOM);  // 0.68
const byte  PH_SAMPLES = 40;           // ring buffer, one sample every 20 ms = 0.8 s
const byte  PH_TRIM = 10;              // drop 10 lowest and 10 highest
const unsigned long SAMPLE_MS = 20;
const float MV_FAULT_LOW  = 30.0;      // at the A1 pin: board unpowered / output shorted
const float MV_FAULT_HIGH = 3000.0;    // at the A1 pin: near the ADC's top end (board ~4.4 V, off the pH scale)

float CAL_SLOPE     = 3.5;             // pH per volt      (from calibration)
float CAL_INTERCEPT = 0.0;             // pH at 0 V        (from calibration)
float CAL_TEMP_C    = 25.0;            // temperature at which calibration was done
float tempC         = 25.0;            // liquid temperature, for compensation
const byte MAX_CAL_POINTS = 3;

// ============================ Pumps ============================
const unsigned long PUMP_TICK_MS = 10;
const unsigned long PUMP_RAMP_MS = 300;
byte pumpPwm = 200;                    // running speed 0-255
const float FLOW_ML_PER_S = 1.0;       // measure with "pa 5000" and update

// ============================ Control ============================
const unsigned long CONTROL_MS = 1000;
const float SP_MIN = 3.0, SP_MAX = 7.0;
const float SP_DEFAULT = 5.0;          // spec default (used by the merged sketch)

float deadband    = 0.15;   // pH
float reverseBand = 0.30;   // pH
float Kdose       = 1000;   // ms of pumping per pH unit of error   (tune)
unsigned long MIX_MS = 15000;          // mixing wait after each dose  (tune)
const unsigned long DOSE_MIN_MS = 200;
const unsigned long DOSE_MAX_MS = 3000;

const bool REQUIRE_STIRRING = true;
const byte MAX_RUN = 15;               // same-direction doses without progress -> fault
const float RUN_MIN_PROGRESS = 0.1;    // pH

// ============================ Statistics ============================
const byte WINDOW_S = 60;

// ============================ State ============================
enum Mode : byte { MODE_OFF, MODE_AUTO, MODE_MANUAL, MODE_FAULT };
enum CtrlState : byte { ST_READY, ST_DOSING, ST_MIXING };
enum Pump : byte { PUMP_NONE, PUMP_ACID, PUMP_BASE };
enum Fault : byte { F_NONE, F_SENSOR, F_RUNAWAY };

Mode mode = MODE_OFF;
CtrlState ctrl = ST_READY;
Fault fault = F_NONE;

// sensor
int   phRing[PH_SAMPLES];
byte  ringIdx = 0, ringFill = 0;
float phVolts = 0, ph = 7.0, err = 0;
bool  sensorOk = false;

// calibration points
float calV[MAX_CAL_POINTS], calPH[MAX_CAL_POINTS];
byte  calN = 0;

// pumps
Pump  activePump = PUMP_NONE;
int   pumpOut = 0;                     // current PWM on the active pump
unsigned long doseStartMs = 0, doseLenMs = 0;
bool  pumpStopping = false;
unsigned long acidMsTotal = 0, baseMsTotal = 0;

// control
float setpoint = 0;
unsigned long mixStartMs = 0;
Pump  lastDose = PUMP_NONE;
byte  runCount = 0;
float runStartPH = 0;

// timing
unsigned long lastSampleMs = 0, lastPumpMs = 0, lastControlMs = 0;

// 60 s window stats
struct WindowStats { float mean, minV, maxV, inBandPct; };
WindowStats lastWin = {0, 0, 0, 0};
float winSum, winMin, winMax;
int   winInBand;
byte  winCount = 0;

byte logMode = 0;
char cmdBuf[32];
byte cmdLen = 0;

// ============================ PWM helpers ============================
// (Defined after the enums: the Arduino IDE inserts function prototypes
// above the first function, so types used in signatures must come first.)
void pwmBegin(uint8_t pin, uint8_t ch, uint32_t hz, uint8_t bits) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(pin, hz, bits);
  (void)ch;
#else
  ledcSetup(ch, hz, bits);
  ledcAttachPin(pin, ch);
#endif
}

void pwmWrite(uint8_t pin, uint8_t ch, uint32_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(pin, duty);
  (void)ch;
#else
  ledcWrite(ch, duty);
  (void)pin;
#endif
}

void acidWrite(int duty) { pwmWrite(PIN_ACID, ACID_PWM_CH, (uint32_t)constrain(duty, 0, 255)); }
void baseWrite(int duty) { pwmWrite(PIN_BASE, BASE_PWM_CH, (uint32_t)constrain(duty, 0, 255)); }

// ============================ Interlock hook ============================
// In the merged sketch, return true only when the stirrer is at speed.
bool stirringRunning() { return true; }

// ============================ Sensor ============================
void sampleADC() {
  phRing[ringIdx] = analogReadMilliVolts(PIN_PH);   // millivolts at the A1 pin
  ringIdx = (ringIdx + 1) % PH_SAMPLES;
  if (ringFill < PH_SAMPLES) ringFill++;
}

// Trimmed mean of the ring buffer (robust to spikes from the pumps).
float trimmedMeanADC() {
  int tmp[PH_SAMPLES];
  byte n = ringFill;
  for (byte i = 0; i < n; i++) tmp[i] = phRing[i];
  for (byte i = 1; i < n; i++) {           // insertion sort, n <= 40
    int v = tmp[i]; int j = i - 1;
    while (j >= 0 && tmp[j] > v) { tmp[j + 1] = tmp[j]; j--; }
    tmp[j + 1] = v;
  }
  byte trim = (n == PH_SAMPLES) ? PH_TRIM : n / 4;
  long sum = 0;
  for (byte i = trim; i < n - trim; i++) sum += tmp[i];
  return (float)sum / (n - 2 * trim);
}

float voltsToPH(float v) {
  float phLin = CAL_SLOPE * v + CAL_INTERCEPT;
  // Nernst slope scales with absolute temperature; pivot about pH 7.
  return 7.0 + (phLin - 7.0) * (CAL_TEMP_C + 273.15) / (tempC + 273.15);
}

// Pin millivolts -> volts at the SEN0161 output (undoes the divider).
float boardVolts(float pinMv) { return pinMv / 1000.0 / PH_DIV; }

bool updatePH() {
  if (ringFill < 4) return false;
  float pinMv = trimmedMeanADC();
  if (pinMv < MV_FAULT_LOW || pinMv > MV_FAULT_HIGH) return false;
  phVolts = boardVolts(pinMv);
  float p = voltsToPH(phVolts);
  if (p < 0 || p > 14) return false;
  ph = p;
  return true;
}

// ============================ Pumps ============================

void allPumpsOff() {
  acidWrite(0);
  baseWrite(0);
  if (activePump != PUMP_NONE) {
    unsigned long ran = millis() - doseStartMs;
    if (activePump == PUMP_ACID) acidMsTotal += ran; else baseMsTotal += ran;
  }
  activePump = PUMP_NONE;
  pumpOut = 0;
  pumpStopping = false;
}

// Starts one dose. Refuses if a pump is already running (interlock).
bool startDose(Pump p, unsigned long ms) {
  if (activePump != PUMP_NONE || p == PUMP_NONE || ms == 0) return false;
  activePump = p;
  doseStartMs = millis();
  doseLenMs = ms;
  pumpOut = 0;
  pumpStopping = false;
  return true;
}

// Every 10 ms: ramp the active pump up, hold, ramp down, finish.
void pumpTick() {
  if (activePump == PUMP_NONE) return;
  int step = max(1, (int)((long)pumpPwm * PUMP_TICK_MS / PUMP_RAMP_MS));
  unsigned long elapsed = millis() - doseStartMs;
  if (!pumpStopping && elapsed >= doseLenMs) pumpStopping = true;

  if (pumpStopping) {
    pumpOut -= step;
    if (pumpOut <= 0) { allPumpsOff(); return; }
  } else if (pumpOut < pumpPwm) {
    pumpOut = min(pumpOut + step, (int)pumpPwm);
  }
  // Interlock: the other pump is always forced off.
  if (activePump == PUMP_ACID) { baseWrite(0); acidWrite(pumpOut); }
  else                         { acidWrite(0); baseWrite(pumpOut); }
}

// ============================ Faults ============================
const char* faultName(Fault f) {
  switch (f) {
    case F_SENSOR:  return "SENSOR (probe voltage at a rail or pH out of 0-14)";
    case F_RUNAWAY: return "RUNAWAY (dosing has no effect: reservoir/tube/pump/probe?)";
    default:        return "none";
  }
}

void tripFault(Fault f) {
  allPumpsOff();
  mode = MODE_FAULT;
  fault = f;
  ctrl = ST_READY;
  Serial.print(F("# FAULT: ")); Serial.println(faultName(f));
  Serial.println(F("# Pumps off. Fix the cause, then send x or a new command."));
}

// ============================ Public setters (for ESP32 link later) ============================
void stopDosing() {
  allPumpsOff();
  mode = MODE_OFF;
  fault = F_NONE;
  ctrl = ST_READY;
  setpoint = 0;
  runCount = 0;
}

// 0 stops dosing. Returns false if out of range or the sensor is bad.
bool setPHSetpoint(float p) {
  if (p == 0) { stopDosing(); return true; }
  if (p < SP_MIN || p > SP_MAX || !sensorOk) return false;
  if (mode != MODE_AUTO) { ctrl = ST_READY; runCount = 0; lastDose = PUMP_NONE; }
  setpoint = p;
  fault = F_NONE;
  mode = MODE_AUTO;
  return true;
}

bool manualDose(Pump p, unsigned long ms) {
  if (ms == 0 || ms > 60000UL) return false;
  if (activePump != PUMP_NONE) return false;
  if (mode == MODE_AUTO) { setpoint = 0; }
  mode = MODE_MANUAL;
  fault = F_NONE;
  return startDose(p, ms);
}

float getPH()            { return ph; }
float getPHSetpoint()    { return setpoint; }
bool  phFaulted()        { return mode == MODE_FAULT; }
float getAcidML()        { return acidMsTotal / 1000.0 * FLOW_ML_PER_S; }
float getBaseML()        { return baseMsTotal / 1000.0 * FLOW_ML_PER_S; }
void  setPHTemperature(float c) { if (c > 0 && c < 60) tempC = c; }

// ============================ Control decision (every 1 s) ============================
void runawayCheck(Pump dir) {
  if (dir != lastDose || runCount == 0) { runCount = 0; runStartPH = ph; }
  runCount++;
  if (runCount >= MAX_RUN) {
    // progress toward the setpoint since the first dose of this run
    float progress = (dir == PUMP_BASE) ? ph - runStartPH : runStartPH - ph;
    if (progress < RUN_MIN_PROGRESS) { tripFault(F_RUNAWAY); return; }
    runCount = 0; runStartPH = ph;
  }
}

void controlStep(unsigned long now) {
  err = (setpoint > 0) ? setpoint - ph : 0;

  if (mode == MODE_MANUAL && activePump == PUMP_NONE) mode = MODE_OFF;
  if (mode != MODE_AUTO) return;

  switch (ctrl) {
    case ST_DOSING:
      if (activePump == PUMP_NONE) { ctrl = ST_MIXING; mixStartMs = now; }
      return;
    case ST_MIXING:
      if (now - mixStartMs < MIX_MS) return;
      ctrl = ST_READY;
      __attribute__((fallthrough));   // mixing done: decide straight away
    case ST_READY: {
      if (REQUIRE_STIRRING && !stirringRunning()) return;
      float aerr = fabs(err);
      if (aerr <= deadband) { runCount = 0; return; }
      Pump dir = (err > 0) ? PUMP_BASE : PUMP_ACID;   // pH too low -> add alkali
      if (lastDose != PUMP_NONE && dir != lastDose && aerr <= reverseBand) return;

      runawayCheck(dir);
      if (mode == MODE_FAULT) return;

      unsigned long ms = (unsigned long)(Kdose * aerr);
      ms = constrain(ms, DOSE_MIN_MS, DOSE_MAX_MS);
      if (startDose(dir, ms)) {
        lastDose = dir;
        ctrl = ST_DOSING;
      }
      return;
    }
  }
}

// ============================ Calibration ============================
void printCal() {
  Serial.print(F("# CAL_SLOPE="));     Serial.print(CAL_SLOPE, 4);
  Serial.print(F(" CAL_INTERCEPT="));  Serial.print(CAL_INTERCEPT, 4);
  Serial.print(F(" CAL_TEMP_C="));     Serial.print(CAL_TEMP_C, 1);
  Serial.println(F("  (copy into the sketch defaults)"));
}

void addCalPoint(float bufferPH) {
  if (ringFill < PH_SAMPLES) { Serial.println(F("# wait for readings to fill")); return; }
  float v = boardVolts(trimmedMeanADC());
  if (calN >= MAX_CAL_POINTS) calN = 0;   // start over after 3
  calV[calN] = v; calPH[calN] = bufferPH; calN++;
  Serial.print(F("# cal point ")); Serial.print(calN);
  Serial.print(F(": pH ")); Serial.print(bufferPH, 2);
  Serial.print(F(" at ")); Serial.print(v, 4); Serial.println(F(" V"));
  if (calN < 2) { Serial.println(F("# need at least one more buffer")); return; }

  // least-squares line pH = slope*V + intercept
  float sx = 0, sy = 0, sxx = 0, sxy = 0;
  for (byte i = 0; i < calN; i++) {
    sx += calV[i]; sy += calPH[i]; sxx += calV[i] * calV[i]; sxy += calV[i] * calPH[i];
  }
  float den = calN * sxx - sx * sx;
  if (fabs(den) < 1e-6) { Serial.println(F("# voltages too close: check the probe")); return; }
  CAL_SLOPE = (calN * sxy - sx * sy) / den;
  CAL_INTERCEPT = (sy - CAL_SLOPE * sx) / calN;
  CAL_TEMP_C = tempC;
  printCal();
  if (fabs(CAL_SLOPE) < 2.0 || fabs(CAL_SLOPE) > 5.0)
    Serial.println(F("# WARNING: slope far from ~3.5 pH/V - probe may be old/dirty or a buffer mixed up"));
}

// ============================ Statistics & logging ============================
void updateWindow() {
  if (winCount == 0) { winSum = 0; winMin = 99; winMax = -99; winInBand = 0; }
  winSum += ph;
  if (ph < winMin) winMin = ph;
  if (ph > winMax) winMax = ph;
  if (mode == MODE_AUTO && fabs(setpoint - ph) <= deadband) winInBand++;
  if (++winCount >= WINDOW_S) {
    lastWin.mean = winSum / WINDOW_S;
    lastWin.minV = winMin;
    lastWin.maxV = winMax;
    lastWin.inBandPct = 100.0 * winInBand / WINDOW_S;
    winCount = 0;
  }
}

void printCsvHeader() {
  Serial.println(F("t_ms,mode,state,setpoint,pH,volts,acid_pwm,base_pwm,acid_mL,base_mL,temp_C"));
}

void logLine() {
  int acidPwm = (activePump == PUMP_ACID) ? pumpOut : 0;
  int basePwm = (activePump == PUMP_BASE) ? pumpOut : 0;
  if (logMode == 1) {
    Serial.print(millis());          Serial.print(',');
    Serial.print((int)mode);         Serial.print(',');
    Serial.print((int)ctrl);         Serial.print(',');
    Serial.print(setpoint, 2);       Serial.print(',');
    Serial.print(ph, 3);             Serial.print(',');
    Serial.print(phVolts, 4);        Serial.print(',');
    Serial.print(acidPwm);           Serial.print(',');
    Serial.print(basePwm);           Serial.print(',');
    Serial.print(getAcidML(), 2);    Serial.print(',');
    Serial.print(getBaseML(), 2);    Serial.print(',');
    Serial.println(tempC, 1);
  } else if (logMode == 2) {
    Serial.print(F("Setpoint:")); Serial.print(setpoint, 2);
    Serial.print(F(",Lower:"));   Serial.print(setpoint - deadband, 2);
    Serial.print(F(",Upper:"));   Serial.print(setpoint + deadband, 2);
    Serial.print(F(",pH:"));      Serial.print(ph, 3);
    Serial.print(F(",Acid:"));    Serial.print(acidPwm > 0 ? 1 : 0);
    Serial.print(F(",Alkali:"));  Serial.println(basePwm > 0 ? 1 : 0);
  }
}

const char* modeName() {
  switch (mode) {
    case MODE_OFF:    return "OFF";
    case MODE_AUTO:   return "AUTO";
    case MODE_MANUAL: return "MANUAL";
    default:          return "FAULT";
  }
}

const char* stateName() {
  switch (ctrl) {
    case ST_DOSING: return "DOSING";
    case ST_MIXING: return "MIXING";
    default:        return "READY";
  }
}

void printStatus() {
  Serial.print(F("# mode="));      Serial.print(modeName());
  Serial.print(F(" state="));      Serial.print(stateName());
  Serial.print(F(" setpoint="));   Serial.print(setpoint, 2);
  Serial.print(F(" pH="));         Serial.print(ph, 3);
  Serial.print(F(" V="));          Serial.print(phVolts, 4);
  Serial.print(F(" sensor="));     Serial.println(sensorOk ? F("ok") : F("BAD"));
  if (mode == MODE_FAULT) { Serial.print(F("# fault: ")); Serial.println(faultName(fault)); }
  Serial.print(F("# deadband="));  Serial.print(deadband, 2);
  Serial.print(F(" reverse="));    Serial.print(reverseBand, 2);
  Serial.print(F(" Kdose="));      Serial.print(Kdose, 0);
  Serial.print(F("ms/pH mix="));   Serial.print(MIX_MS / 1000);
  Serial.print(F("s pumpPwm="));   Serial.print(pumpPwm);
  Serial.print(F(" temp="));       Serial.println(tempC, 1);
  printCal();
  Serial.print(F("# dosed: acid=")); Serial.print(getAcidML(), 1);
  Serial.print(F("mL alkali="));     Serial.print(getBaseML(), 1);
  Serial.println(F("mL"));
  Serial.print(F("# last 60 s: mean=")); Serial.print(lastWin.mean, 3);
  Serial.print(F(" min="));        Serial.print(lastWin.minV, 3);
  Serial.print(F(" max="));        Serial.print(lastWin.maxV, 3);
  Serial.print(F(" in-band="));    Serial.print(lastWin.inBandPct, 0);
  Serial.println('%');
}

void printHelp() {
  Serial.println(F("# Commands: s <pH> | db <v> | rb <v> | kd <ms/pH> | mix <s> | pa <ms> | pb <ms> | pwm <0-255> | t <C> | cal <pH> | cal0 | l <0|1|2> | x | ?"));
}

// ============================ Serial commands ============================
void handleCommand(char* line) {
  for (char* p = line; *p; p++) *p = tolower(*p);
  char* cmd = strtok(line, " ,");
  char* a1  = strtok(NULL, " ,");
  if (!cmd) return;

  if (!strcmp(cmd, "s") && a1) {
    if (setPHSetpoint(atof(a1))) { Serial.print(F("# setpoint ")); Serial.println(setpoint, 2); }
    else if (!sensorOk) Serial.println(F("# rejected: pH sensor reading is bad"));
    else Serial.println(F("# rejected: setpoint must be 0 or 3-7"));
  } else if (!strcmp(cmd, "db") && a1) {
    float v = atof(a1);
    if (v > 0 && v < 1) { deadband = v; if (reverseBand < v) reverseBand = v; printStatus(); }
    else Serial.println(F("# rejected: deadband must be 0-1"));
  } else if (!strcmp(cmd, "rb") && a1) {
    float v = atof(a1);
    if (v >= deadband && v < 2) { reverseBand = v; printStatus(); }
    else Serial.println(F("# rejected: reverse band must be >= deadband and < 2"));
  } else if (!strcmp(cmd, "kd") && a1) {
    float v = atof(a1);
    if (v > 0) { Kdose = v; printStatus(); }
  } else if (!strcmp(cmd, "mix") && a1) {
    long s = atol(a1);
    if (s >= 1 && s <= 600) { MIX_MS = s * 1000UL; printStatus(); }
    else Serial.println(F("# rejected: mix must be 1-600 s"));
  } else if ((!strcmp(cmd, "pa") || !strcmp(cmd, "pb")) && a1) {
    Pump p = (cmd[1] == 'a') ? PUMP_ACID : PUMP_BASE;
    if (manualDose(p, (unsigned long)atol(a1))) {
      Serial.print(p == PUMP_ACID ? F("# acid pump ") : F("# alkali pump "));
      Serial.print(a1); Serial.println(F(" ms"));
    } else Serial.println(F("# rejected: a pump is running or time not 1-60000 ms"));
  } else if (!strcmp(cmd, "pwm") && a1) {
    int v = atoi(a1);
    if (v >= 50 && v <= 255) { pumpPwm = v; printStatus(); }
    else Serial.println(F("# rejected: pump pwm must be 50-255"));
  } else if (!strcmp(cmd, "t") && a1) {
    setPHTemperature(atof(a1)); printStatus();
  } else if (!strcmp(cmd, "cal") && a1) {
    addCalPoint(atof(a1));
  } else if (!strcmp(cmd, "cal0")) {
    calN = 0; CAL_SLOPE = 3.5; CAL_INTERCEPT = 0.0; CAL_TEMP_C = 25.0; printCal();
  } else if (!strcmp(cmd, "l") && a1) {
    logMode = constrain(atoi(a1), 0, 2);
    if (logMode == 1) printCsvHeader();
  } else if (!strcmp(cmd, "x")) {
    stopDosing(); Serial.println(F("# pumps off"));
  } else if (!strcmp(cmd, "?")) {
    printStatus();
  } else {
    printHelp();
  }
}

void pollSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (cmdLen) { cmdBuf[cmdLen] = 0; handleCommand(cmdBuf); cmdLen = 0; }
    } else if (cmdLen < sizeof(cmdBuf) - 1) {
      cmdBuf[cmdLen++] = c;
    }
  }
}

// ============================ Setup / loop ============================
void setup() {
  pinMode(PIN_ACID, OUTPUT);
  pinMode(PIN_BASE, OUTPUT);
  digitalWrite(PIN_ACID, LOW);
  digitalWrite(PIN_BASE, LOW);
  pwmBegin(PIN_ACID, ACID_PWM_CH, PUMP_PWM_HZ, PUMP_PWM_BITS);
  pwmBegin(PIN_BASE, BASE_PWM_CH, PUMP_PWM_HZ, PUMP_PWM_BITS);
  allPumpsOff();

  Serial.begin(115200);
  Serial.println(F("# Bioreactor pH controller ready. Pumps are OFF (monitor only)."));
  printHelp();

  lastSampleMs = lastPumpMs = lastControlMs = millis();
}

void loop() {
  unsigned long now = millis();

  if (now - lastSampleMs >= SAMPLE_MS) {
    lastSampleMs += SAMPLE_MS;
    sampleADC();
  }

  if (now - lastPumpMs >= PUMP_TICK_MS) {
    lastPumpMs += PUMP_TICK_MS;
    pumpTick();
  }

  if (now - lastControlMs >= CONTROL_MS) {
    lastControlMs += CONTROL_MS;
    sensorOk = updatePH();
    // Only automatic dosing depends on the reading; manual priming still works
    // with the probe disconnected or in its storage bottle.
    if (!sensorOk && mode == MODE_AUTO) tripFault(F_SENSOR);
    else {
      controlStep(now);
      updateWindow();
    }
    logLine();
  }

  pollSerial();
}
