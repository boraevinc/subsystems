/*
  heating_control.ino
  Bioreactor - heating subsystem: closed-loop temperature control

  Board : Arduino Uno / Nano (ATmega328P, 16 MHz)
  Spec  : hold temperature at a setpoint in 25-35 C, within +/-0.5 C.
          Inlet water 10-20 C. Setpoint adjustable on the fly (Serial Monitor
          now; the ESP32 link calls setHeatSetpoint() later).

  Hardware:
    - Thermistor: ND06P00103K, 10k NTC, B25/85 = 4220 K (+/-3%), R25 +/-10%
      (the 10% tolerance is why the 2-point calibration below matters).
      Voltage divider:  5V --- R_FIXED (10k) ---+--- NTC --- GND
                                                 |
                                                 A0   (optional 0.1-1 uF to GND
                                                       to filter heater PWM noise)
      If the EEE side wires it the other way round, set THERM_ON_TOP = true.
      Ratiometric: the result doesn't depend on the exact 5 V value.
    - Heater: 3 ohm, 30 W element, low-side IRLB8748 MOSFET, gate on D9.
      At 12 V it would draw 4 A = 48 W, OVER its 30 W rating, so power is
      capped at P_MAX (28 W, ~58% duty). If you run it from the 6 V rail,
      change HEATER_V to 6.0 (then 12 W max).

  Controller (output is heater POWER in watts, converted to PWM duty):
    - PI : P = Kp*e + I.  The integral only runs within I_BAND of the setpoint
           and never pushes further into saturation, so it doesn't wind up
           during the long warm-up. I is clamped 0..P_MAX (it learns the power
           needed to balance heat losses at the setpoint).
    - Bang-bang with hysteresis (for comparison in the report).
    - Manual fixed power (open-loop step test -> plant model for tuning).
    Control update every 1 s; temperature sampled every 100 ms
    (16x oversampled + exponential filter).

  Safety (heater off and FAULT latched):
    - Sensor open/short (ADC near 0 or 1023).
    - Over-temperature above T_CUTOFF.
    - No-rise watchdog: heater near full power for NORISE_S seconds but the
      temperature rose less than NORISE_MIN_C -> thermistor out of the water,
      heater disconnected or MOSFET dead.
    Clear with "x", or any new s/m/b command once the condition has gone.
    The heater starts OFF at power-up; send "s 30" to start.

  ---------------------------------------------------------------------------
  Serial Monitor: 250000 baud, line ending "Newline"
    s <C>         PI control to setpoint (25-35 C), 0 = heater off
    b <C>         bang-bang control to setpoint (25-35 C)
    h <C>         bang-bang hysteresis (default 0.2 C)
    m <W>         manual fixed heater power, 0..P_MAX watts
    kp <W/C>      proportional gain
    ki <W/(C*s)>  integral gain
    cal1 <C>      calibration point 1: send the THERMOMETER reading
    cal2 <C>      calibration point 2 (at a different temperature)
    cal0          clear calibration
    l <0|1|2>     logging: 0 off, 1 CSV (1 Hz), 2 Serial Plotter (1 Hz)
    x             heater off (also clears a fault)
    ?             status, gains, calibration, last 60 s statistics

  Calibration (do this first):
    1. Thermistor and a lab thermometer side by side in room-temperature
       water, stirred. Wait 1 min. Send "cal1 <thermometer reading>".
    2. Same in warm water (~35-40 C). Send "cal2 <thermometer reading>".
    3. Copy the printed CAL_GAIN / CAL_OFFSET into the defaults below.
    4. Check a third temperature to quote the accuracy in the report.

  Tuning:
    1. Open-loop step: "l 1", "m 20", log ~10 min of temperature rise.
       The slope gives the plant gain (C/s per W); the delay before it starts
       rising gives the dead time. Use these with Ziegler-Nichols / Cohen-Coon
       (slides) or tune by hand:
    2. "s 30". Raise kp until it oscillates around the setpoint, then halve.
    3. Raise ki until the offset goes within ~5 min, without overshoot past 0.5 C.
    4. Copy final Kp, Ki into the defaults below.
*/

// ============================ Pins ============================
const byte PIN_THERM  = A0;
const byte PIN_HEATER = 9;    // Timer1 (OC1A), same timer setup as the stirrer on D10

// ============================ Thermistor ============================
const bool  THERM_ON_TOP = false;   // false: NTC between A0 and GND
const float R_FIXED = 10000.0;      // divider resistor, ohms
const float R25     = 10000.0;      // NTC resistance at 25 C
const float BETA    = 4220.0;       // B25/85 from the ND06 series listing
const float T25_K   = 298.15;
const byte  ADC_SAMPLES = 16;       // oversampling per reading
const int   ADC_FAULT_LOW  = 15;    // below: thermistor shorted (or divider swapped)
const int   ADC_FAULT_HIGH = 1008;  // above: thermistor open / unplugged
const float TEMP_FILTER = 0.2;      // EMA weight per 100 ms sample

float CAL_GAIN   = 1.0;             // T = CAL_GAIN * T_beta + CAL_OFFSET
float CAL_OFFSET = 0.0;

// ============================ Heater ============================
const float HEATER_V = 12.0;
const float HEATER_R = 3.0;
const float P_FULL   = HEATER_V * HEATER_V / HEATER_R;  // 48 W at 12 V
const float P_RATED  = 30.0;
const float P_MAX    = min(28.0, P_FULL);               // stay under the rating
const int   PWM_MAX  = 1023;

// ============================ Control ============================
const unsigned long SAMPLE_MS  = 100;
const unsigned long CONTROL_MS = 1000;
const float CONTROL_DT = CONTROL_MS / 1000.0;
const float SP_MIN = 25.0, SP_MAX = 35.0;
const float SPEC_BAND = 0.5;

float Kp   = 10.0;    // W per C of error          (tune)
float Ki   = 0.05;    // W per (C * s)             (tune)
float hyst = 0.2;     // bang-bang half-band, C
const float I_BAND = 2.0;   // integrate only when |error| < this

// ============================ Safety ============================
const float T_CUTOFF      = 40.0;
const float T_RESUME      = 38.0;   // must be below this to clear an over-temp fault
const unsigned long NORISE_S = 300; // 5 min
const float NORISE_MIN_C  = 0.5;
const float NORISE_POWER  = 0.8;    // fraction of P_MAX that counts as "near full"

// ============================ Statistics ============================
const byte WINDOW_S = 60;

// ============================ State ============================
enum Mode : byte { MODE_OFF, MODE_PI, MODE_BANG, MODE_MANUAL, MODE_FAULT };
enum Fault : byte { F_NONE, F_SENSOR, F_OVERTEMP, F_NORISE };
Mode  mode  = MODE_OFF;
Fault fault = F_NONE;

float adcAvg = 0, tempRaw = 0, temp = 0;
bool  tempValid = false;   // at least one good reading since power-up
bool  sensorOk  = false;   // latest reading was in range
float setpoint = 0, integ = 0, err = 0, power = 0, manualPower = 0;
bool  bangOn = false;

unsigned long lastSampleMs = 0, lastControlMs = 0;
unsigned long spChangeMs = 0, inBandSinceMs = 0;
bool inBand = false;

// no-rise watchdog
bool  noriseArmed = false;
unsigned long noriseStartMs = 0;
float noriseStartT = 0;

// calibration points (raw beta temperature, true thermometer temperature)
float calRaw1 = 0, calTrue1 = 0;
bool  calHave1 = false;

// 60 s window stats
struct WindowStats { float mean, minV, maxV, powerMean, inBandPct; };
WindowStats lastWin = {0, 0, 0, 0, 0};
float winSum, winMin, winMax, winPowerSum;
int   winInBand;
byte  winCount = 0;

byte logMode = 0;
char cmdBuf[32];
byte cmdLen = 0;

// ============================ Temperature ============================
// Returns false on a sensor fault.
bool readTemperature() {
  unsigned int sum = 0;
  for (byte i = 0; i < ADC_SAMPLES; i++) sum += analogRead(PIN_THERM);
  adcAvg = (float)sum / ADC_SAMPLES;

  if (adcAvg < ADC_FAULT_LOW || adcAvg > ADC_FAULT_HIGH) return false;

  float rNtc = THERM_ON_TOP ? R_FIXED * (1023.0 - adcAvg) / adcAvg
                            : R_FIXED * adcAvg / (1023.0 - adcAvg);
  float invT = 1.0 / T25_K + log(rNtc / R25) / BETA;
  float tBeta = 1.0 / invT - 273.15;

  if (!tempValid) { tempRaw = tBeta; tempValid = true; }
  else tempRaw += TEMP_FILTER * (tBeta - tempRaw);
  temp = CAL_GAIN * tempRaw + CAL_OFFSET;
  return true;
}

// ============================ Heater ============================
void writeHeater(float watts) {
  watts = constrain(watts, 0, P_MAX);
  int v = (int)(watts / P_FULL * PWM_MAX + 0.5);
  v = constrain(v, 0, PWM_MAX);
  if (v == 255) v = 256;  // analogWrite treats 255 as "fully on" even in 10-bit mode
  analogWrite(PIN_HEATER, v);
}

// ============================ Faults ============================
const char* faultName(Fault f) {
  switch (f) {
    case F_SENSOR:   return "SENSOR (thermistor open/short)";
    case F_OVERTEMP: return "OVER-TEMPERATURE";
    case F_NORISE:   return "NO TEMPERATURE RISE (thermistor out of water / heater not heating)";
    default:         return "none";
  }
}

void tripFault(Fault f) {
  mode = MODE_FAULT;
  fault = f;
  power = 0;
  integ = 0;
  noriseArmed = false;
  writeHeater(0);
  Serial.print(F("# FAULT: ")); Serial.println(faultName(f));
  Serial.println(F("# Heater off. Fix the cause, then send x or a new command."));
}

// A fault can only be cleared once its cause has gone.
bool faultCleared() {
  if (!sensorOk) return false;            // never heat on a bad reading
  if (mode != MODE_FAULT) return true;
  if (fault == F_OVERTEMP && temp > T_RESUME) return false;
  return true;
}

void noriseCheck(unsigned long now) {
  if (power >= NORISE_POWER * P_MAX) {
    if (!noriseArmed) { noriseArmed = true; noriseStartMs = now; noriseStartT = temp; }
    else if (now - noriseStartMs >= NORISE_S * 1000UL) {
      if (temp - noriseStartT < NORISE_MIN_C) { tripFault(F_NORISE); return; }
      noriseStartMs = now; noriseStartT = temp;   // rising fine: start a new interval
    }
  } else {
    noriseArmed = false;
  }
}

// ============================ Public setters (for ESP32 link later) ============================
bool validSetpoint(float c) { return c >= SP_MIN && c <= SP_MAX; }

void heaterOff() {
  mode = MODE_OFF;
  fault = F_NONE;
  setpoint = integ = power = manualPower = 0;
  bangOn = false;
  noriseArmed = false;
  writeHeater(0);
}

// 0 turns the heater off. Returns false (and changes nothing) if out of range
// or a fault is still active.
bool setHeatSetpoint(float c) {
  if (c == 0) { heaterOff(); return true; }
  if (!validSetpoint(c) || !faultCleared()) return false;
  if (c != setpoint) { spChangeMs = millis(); inBand = false; }
  if (mode != MODE_PI) integ = 0;
  setpoint = c;
  fault = F_NONE;
  mode = MODE_PI;
  return true;
}

bool setHeatBangBang(float c) {
  if (!validSetpoint(c) || !faultCleared()) return false;
  if (c != setpoint) { spChangeMs = millis(); inBand = false; }
  setpoint = c;
  fault = F_NONE;
  mode = MODE_BANG;
  return true;
}

bool setHeatManual(float watts) {
  if (watts < 0 || watts > P_MAX || !faultCleared()) return false;
  manualPower = watts;
  setpoint = 0;
  fault = F_NONE;
  mode = (watts == 0) ? MODE_OFF : MODE_MANUAL;
  return true;
}

float getTemperature()   { return temp; }
float getHeatSetpoint()  { return setpoint; }
float getHeaterPower()   { return power; }
bool  heaterFaulted()    { return mode == MODE_FAULT; }

// ============================ Control step (every 1 s) ============================
void controlStep(unsigned long now) {
  if (mode == MODE_FAULT || !sensorOk) { power = 0; writeHeater(0); return; }

  if (temp > T_CUTOFF) { tripFault(F_OVERTEMP); return; }

  err = (setpoint > 0) ? setpoint - temp : 0;

  switch (mode) {
    case MODE_PI: {
      float pTerm = Kp * err;
      float uTry = pTerm + integ;
      bool satHigh = uTry >= P_MAX;
      bool satLow  = uTry <= 0;
      if (fabs(err) < I_BAND && !(satHigh && err > 0) && !(satLow && err < 0)) {
        integ = constrain(integ + Ki * err * CONTROL_DT, 0, P_MAX);
      }
      power = constrain(pTerm + integ, 0, P_MAX);
      break;
    }
    case MODE_BANG:
      if (temp < setpoint - hyst) bangOn = true;
      else if (temp > setpoint + hyst) bangOn = false;
      power = bangOn ? P_MAX : 0;
      break;
    case MODE_MANUAL:
      power = manualPower;
      break;
    default:
      power = 0;
      break;
  }

  writeHeater(power);
  noriseCheck(now);

  // settling bookkeeping
  bool nowIn = (setpoint > 0) && fabs(setpoint - temp) <= SPEC_BAND;
  if (nowIn && !inBand) inBandSinceMs = now;
  inBand = nowIn;
}

// ============================ Statistics & logging ============================
void updateWindow() {
  if (winCount == 0) { winSum = 0; winPowerSum = 0; winMin = 1e9; winMax = -1e9; winInBand = 0; }
  winSum += temp;
  winPowerSum += power;
  if (temp < winMin) winMin = temp;
  if (temp > winMax) winMax = temp;
  if (inBand) winInBand++;
  if (++winCount >= WINDOW_S) {
    lastWin.mean      = winSum / WINDOW_S;
    lastWin.minV      = winMin;
    lastWin.maxV      = winMax;
    lastWin.powerMean = winPowerSum / WINDOW_S;
    lastWin.inBandPct = 100.0 * winInBand / WINDOW_S;
    winCount = 0;
  }
}

void printCsvHeader() {
  Serial.println(F("t_ms,mode,setpoint,temp,temp_uncal,adc,power_W,duty,err,integ_W"));
}

void logLine() {
  if (logMode == 1) {
    Serial.print(millis());                Serial.print(',');
    Serial.print((int)mode);               Serial.print(',');
    Serial.print(setpoint, 1);             Serial.print(',');
    Serial.print(temp, 2);                 Serial.print(',');
    Serial.print(tempRaw, 2);              Serial.print(',');
    Serial.print(adcAvg, 1);               Serial.print(',');
    Serial.print(power, 2);                Serial.print(',');
    Serial.print(power / P_FULL, 3);       Serial.print(',');
    Serial.print(err, 2);                  Serial.print(',');
    Serial.println(integ, 2);
  } else if (logMode == 2) {
    Serial.print(F("Setpoint:")); Serial.print(setpoint, 1);
    Serial.print(F(",Lower:"));   Serial.print(setpoint - SPEC_BAND, 1);
    Serial.print(F(",Upper:"));   Serial.print(setpoint + SPEC_BAND, 1);
    Serial.print(F(",Temp:"));    Serial.print(temp, 2);
    Serial.print(F(",Power_W:")); Serial.println(power, 1);
  }
}

const char* modeName() {
  switch (mode) {
    case MODE_OFF:    return "OFF";
    case MODE_PI:     return "PI";
    case MODE_BANG:   return "BANG-BANG";
    case MODE_MANUAL: return "MANUAL";
    default:          return "FAULT";
  }
}

void printStatus() {
  unsigned long now = millis();
  Serial.print(F("# mode="));     Serial.print(modeName());
  Serial.print(F(" setpoint="));  Serial.print(setpoint, 1);
  Serial.print(F(" temp="));      Serial.print(temp, 2);
  Serial.print(F(" power="));     Serial.print(power, 1);
  Serial.print(F("W integ="));    Serial.print(integ, 2);
  Serial.print(F("W adc="));      Serial.println(adcAvg, 1);
  if (mode == MODE_FAULT) { Serial.print(F("# fault: ")); Serial.println(faultName(fault)); }
  Serial.print(F("# Kp="));       Serial.print(Kp, 3);
  Serial.print(F(" Ki="));        Serial.print(Ki, 4);
  Serial.print(F(" hyst="));      Serial.print(hyst, 2);
  Serial.print(F(" P_MAX="));     Serial.print(P_MAX, 1);
  Serial.print(F("W CAL_GAIN=")); Serial.print(CAL_GAIN, 4);
  Serial.print(F(" CAL_OFFSET="));Serial.println(CAL_OFFSET, 3);
  if (setpoint > 0) {
    Serial.print(F("# since setpoint change: ")); Serial.print((now - spChangeMs) / 1000);
    Serial.print(F(" s, in +/-0.5 band: "));
    if (inBand) { Serial.print((now - inBandSinceMs) / 1000); Serial.println(F(" s")); }
    else Serial.println(F("no"));
  }
  Serial.print(F("# last 60 s: mean=")); Serial.print(lastWin.mean, 2);
  Serial.print(F(" min="));       Serial.print(lastWin.minV, 2);
  Serial.print(F(" max="));       Serial.print(lastWin.maxV, 2);
  Serial.print(F(" power="));     Serial.print(lastWin.powerMean, 1);
  Serial.print(F("W in+/-0.5=")); Serial.print(lastWin.inBandPct, 0);
  Serial.println('%');
}

void printHelp() {
  Serial.println(F("# Commands: s <C> | b <C> | h <C> | m <W> | kp <v> | ki <v> | cal1 <C> | cal2 <C> | cal0 | l <0|1|2> | x | ?"));
}

// ============================ Calibration ============================
void printCal() {
  Serial.print(F("# CAL_GAIN="));   Serial.print(CAL_GAIN, 4);
  Serial.print(F(" CAL_OFFSET="));  Serial.print(CAL_OFFSET, 3);
  Serial.println(F("  (copy into the sketch defaults)"));
}

void calPoint(byte n, float trueC) {
  if (!sensorOk) { Serial.println(F("# no valid temperature reading")); return; }
  if (n == 1) {
    calRaw1 = tempRaw; calTrue1 = trueC; calHave1 = true;
    CAL_GAIN = 1.0; CAL_OFFSET = trueC - tempRaw;   // offset-only until point 2
    Serial.println(F("# point 1 stored (offset-only correction for now)"));
    printCal();
  } else {
    if (!calHave1) { Serial.println(F("# send cal1 first")); return; }
    float dRaw = tempRaw - calRaw1;
    if (fabs(dRaw) < 3.0) { Serial.println(F("# points too close: need >= 3 C apart")); return; }
    CAL_GAIN = (trueC - calTrue1) / dRaw;
    CAL_OFFSET = calTrue1 - CAL_GAIN * calRaw1;
    Serial.println(F("# 2-point calibration done"));
    printCal();
  }
  temp = CAL_GAIN * tempRaw + CAL_OFFSET;
}

// ============================ Serial commands ============================
void handleCommand(char* line) {
  for (char* p = line; *p; p++) *p = tolower(*p);
  char* cmd = strtok(line, " ,");
  char* a1  = strtok(NULL, " ,");
  if (!cmd) return;

  if (!strcmp(cmd, "s") && a1) {
    if (setHeatSetpoint(atof(a1))) { Serial.print(F("# PI setpoint ")); Serial.println(setpoint, 1); }
    else if (!faultCleared()) Serial.println(F("# rejected: fault still active"));
    else Serial.println(F("# rejected: setpoint must be 0 or 25-35"));
  } else if (!strcmp(cmd, "b") && a1) {
    if (setHeatBangBang(atof(a1))) { Serial.print(F("# bang-bang setpoint ")); Serial.println(setpoint, 1); }
    else if (!faultCleared()) Serial.println(F("# rejected: fault still active"));
    else Serial.println(F("# rejected: setpoint must be 25-35"));
  } else if (!strcmp(cmd, "h") && a1) {
    float h = atof(a1);
    if (h > 0 && h <= 2) { hyst = h; printStatus(); }
    else Serial.println(F("# rejected: hysteresis must be 0-2 C"));
  } else if (!strcmp(cmd, "m") && a1) {
    if (setHeatManual(atof(a1))) { Serial.print(F("# manual power ")); Serial.print(manualPower, 1); Serial.println('W'); }
    else if (!faultCleared()) Serial.println(F("# rejected: fault still active"));
    else { Serial.print(F("# rejected: power must be 0-")); Serial.print(P_MAX, 0); Serial.println('W'); }
  } else if (!strcmp(cmd, "kp") && a1) {
    Kp = max(0.0, atof(a1)); printStatus();
  } else if (!strcmp(cmd, "ki") && a1) {
    Ki = max(0.0, atof(a1)); printStatus();
  } else if (!strcmp(cmd, "cal1") && a1) {
    calPoint(1, atof(a1));
  } else if (!strcmp(cmd, "cal2") && a1) {
    calPoint(2, atof(a1));
  } else if (!strcmp(cmd, "cal0")) {
    CAL_GAIN = 1.0; CAL_OFFSET = 0.0; calHave1 = false; printCal();
  } else if (!strcmp(cmd, "l") && a1) {
    logMode = constrain(atoi(a1), 0, 2);
    if (logMode == 1) printCsvHeader();
  } else if (!strcmp(cmd, "x")) {
    if (mode == MODE_FAULT && !faultCleared()) {
      Serial.println(F("# heater stays off: fault condition still present"));
      power = 0; writeHeater(0);
    } else {
      heaterOff(); Serial.println(F("# heater off"));
    }
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
  pinMode(PIN_HEATER, OUTPUT);
  digitalWrite(PIN_HEATER, LOW);

  // Timer1 (D9, D10): 10-bit phase-correct PWM, no prescaler -> ~7.8 kHz.
  // Same as the stirring sketch, so both can share it when merged.
  TCCR1A = _BV(WGM11) | _BV(WGM10);
  TCCR1B = _BV(CS10);
  writeHeater(0);

  Serial.begin(250000);
  Serial.println(F("# Bioreactor heating controller ready. Heater is OFF."));
  printHelp();

  sensorOk = readTemperature();
  if (!sensorOk) tripFault(F_SENSOR);
  lastSampleMs = lastControlMs = millis();
}

void loop() {
  unsigned long now = millis();

  if (now - lastSampleMs >= SAMPLE_MS) {
    lastSampleMs += SAMPLE_MS;
    sensorOk = readTemperature();
    // A sensor fault stays latched after reconnecting until the user clears it.
    if (!sensorOk && mode != MODE_FAULT) tripFault(F_SENSOR);
  }

  if (now - lastControlMs >= CONTROL_MS) {
    lastControlMs += CONTROL_MS;
    controlStep(now);
    updateWindow();
    logLine();
  }

  pollSerial();
}
