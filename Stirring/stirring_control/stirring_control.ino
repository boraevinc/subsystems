/*
  stirring_control.ino
  Bioreactor - stirring subsystem: closed-loop speed control

  Board : Arduino Nano ESP32 (ESP32-S3, 3.3 V logic). WiFi is NOT used: the
          separate Espressif ESP32 does networking over a wired UART link.
          Arduino IDE: Boards Manager "esp32 by Espressif Systems" 3.x,
          board "Arduino Nano ESP32", Pin Numbering "By Arduino pin (default)".
  Spec  : hold stirring speed at a setpoint in 500-1500 RPM, within +/-20 RPM.
          Setpoint must be adjustable on the fly (here: Serial Monitor commands;
          later: the ESP32 link calls setStirSetpoint()).

  Hardware (from MotorHallSensorInstructions.pptx):
    - 5 V DC motor, 10:1 gearbox, powered from the Nano's VBUS pin (USB 5 V),
      NOT an external supply.
    - Hall sensor, 70 pulses per rev of the output shaft -> D2 (interrupt).
      The Nano ESP32 pins are NOT 5 V tolerant: the sensor output must only
      ever be pulled up to 3.3 V (or go through a 4.7k/10k divider if it
      drives 5 V itself). See the EEE build guide.
    - Logic-level N-MOSFET (IRLB8748) low-side switch, gate on D10,
      1N4001 flyback diode across the motor, 470 uF decoupling on 5 V.

  Controller:
    PWM = feedforward(ref) + Kp*e + Ki*integral(e)
      - feedforward: PWM = ffGain*RPM + ffOffset, fitted from the open-loop sweep
      - PI on the raw measured speed, conditional-integration anti-windup
      - setpoint is ramped (rampRate RPM/s) so the gears are never shock-loaded
        (Moodle: "power should be ramped up steadily ... gears will shear")
      - stall detection: high PWM but no pulses for 1.5 s -> motor off, FAULT

  Speed measurement:
    The ISR only timestamps and counts pulses. Every 10 ms the loop takes an
    atomic snapshot and computes RPM from the number of pulses and the time
    between the first and last of them, i.e. it averages over every pulse in the
    window. (The starter sketch did float maths inside the ISR and read the
    shared float without disabling interrupts, which can give corrupt readings.)

  ---------------------------------------------------------------------------
  Serial Monitor: 115200 baud, line ending "Newline"
    s <rpm>      closed-loop setpoint (0 = stop, else 500-1500)
    m <pwm>      manual open-loop PWM, 0-1023 (PI off)
    w            open-loop sweep: steps PWM 0->1023, prints RPM at each step
                 and a suggested feedforward line
    kp <val>     set proportional gain   (PWM counts per RPM of error)
    ki <val>     set integral gain       (PWM counts per RPM*s)
    ff <g> <o>   set feedforward gain and offset
    r <rpm/s>    set setpoint ramp rate
    l <0|1|2>    logging: 0 off, 1 CSV (10 Hz), 2 Serial Plotter (10 Hz)
    x            stop motor immediately (also clears a fault)
    ?            status, gains and last 1 s window statistics

  Tuning procedure:
    1. Upload, open Serial Monitor, send "w". Wait ~1 min. Copy the suggested
       "ff ..." line and send it. Keep the sweep table for the report
       (open-loop characteristic + the real max RPM at 5 V).
    2. "l 2", open Serial Plotter, "s 800". Step between 500 and 1000.
    3. Set ki 0. Raise kp until the speed starts to oscillate, then halve it.
    4. Raise ki until the steady-state error disappears within ~1 s without
       large overshoot.
    5. Copy final Kp, Ki, ffGain, ffOffset into the defaults below.
    6. Check the reading against the tachometer at a few setpoints.
*/

#if !defined(ARDUINO_ARCH_ESP32)
#error "This sketch targets the Arduino Nano ESP32. Select it under Tools > Board."
#endif

// ============================ Pins ============================
const uint8_t PIN_HALL  = D2;           // Hall sensor (any Nano ESP32 pin can interrupt)
const uint8_t PIN_MOTOR = D10;          // MOSFET gate, LEDC PWM
const uint8_t PIN_LED   = LED_BUILTIN;  // D13, blinks once per output-shaft revolution

// ============================ PWM (LEDC) ============================
const uint32_t MOTOR_PWM_HZ   = 8000;   // 8 kHz, 10-bit like the original starter code
const uint8_t  MOTOR_PWM_BITS = 10;
const uint8_t  MOTOR_PWM_CH   = 0;      // only used by esp32 core 2.x

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

// ============================ Sensor ============================
const float PULSES_PER_REV = 70.0;
const float RPM_SENSOR_MAX = 1800.0;  // faster pulses are rejected as noise
const unsigned long TMIN_US = (unsigned long)(60.0e6 / (RPM_SENSOR_MAX * PULSES_PER_REV)); // ~476 us
const unsigned long ZERO_TIMEOUT_US = 100000UL;  // no pulse for 100 ms -> 0 RPM
const float RPM_K = 60.0e6 / PULSES_PER_REV;     // RPM = RPM_K * pulses / microseconds
const byte LED_TOGGLE_PULSES = 35;               // toggle twice per rev = one blink

// ============================ Control ============================
const unsigned long TS_US = 10000UL;   // 10 ms control period
const float TS = TS_US * 1e-6;
const int   PWM_MAX = 1023;            // 10-bit PWM, 1023 = 5 V
const float SP_MIN = 500.0;            // spec range
const float SP_MAX = 1500.0;

float Kp       = 0.5;    // PWM counts per RPM error        (tune)
float Ki       = 3.0;    // PWM counts per (RPM * s)         (tune)
float ffGain   = 0.9;    // PWM counts per RPM               (from sweep)
float ffOffset = 80.0;   // PWM counts to overcome friction  (from sweep)
float rampRate = 500.0;  // RPM per second

const float I_LIMIT       = 400.0;  // integral term clamp, PWM counts
const int   MANUAL_SLEW   = 10;     // PWM counts per tick in manual/sweep (~1 s 0->1023)
const int   STALL_PWM     = 500;    // above this PWM the shaft must be turning
const int   STALL_TICKS   = 150;    // 1.5 s

// ============================ Logging ============================
const byte LOG_EVERY_TICKS = 10;    // 100 ms
const byte WINDOW_TICKS    = 100;   // 1 s statistics window
const float SPEC_BAND      = 20.0;  // +/- RPM

// ============================ Sweep ============================
const int  SWEEP_STEP       = 64;
const byte SWEEP_POINTS     = 17;   // 0, 64, ... 960, 1023
const int  SWEEP_HOLD_TICKS = 300;  // 3 s per step
const int  SWEEP_AVG_TICKS  = 100;  // average the last 1 s of each step

// ============================ State ============================
enum Mode : byte { MODE_OFF, MODE_AUTO, MODE_MANUAL, MODE_SWEEP, MODE_FAULT };
Mode mode = MODE_OFF;

volatile unsigned long isrCount  = 0;
volatile unsigned long isrLastUs = 0;
portMUX_TYPE isrMux = portMUX_INITIALIZER_UNLOCKED;  // protects isrCount/isrLastUs
unsigned long ledPulses = 0;

unsigned long prevCount = 0, prevPulseUs = 0, nextTickUs = 0;
float rpm = 0, rpmSmooth = 0;
float setpoint = 0, ref = 0, integ = 0, err = 0;
int   pwmOut = 0, manualTarget = 0;
int   stallTicks = 0;
byte  logMode = 0, logTick = 0;

// 1 s window statistics (same shape as the MQTT telemetry: mean/min/max)
struct WindowStats { float mean, minV, maxV, pwmMean, inBandPct; };
WindowStats lastWin = {0, 0, 0, 0, 0};
float winSum, winMin, winMax, winPwmSum;
int   winInBand;
byte  winTick = 0;

// sweep
byte  sweepIdx = 0;
int   sweepTick = 0;
float sweepSum = 0;
float sweepPwm[SWEEP_POINTS], sweepRpm[SWEEP_POINTS];

// serial commands
char cmdBuf[32];
byte cmdLen = 0;

// ============================ ISR ============================
// Kept in IRAM and as short as possible; the LED is driven from the loop.
void IRAM_ATTR hallISR() {
  unsigned long now = micros();
  portENTER_CRITICAL_ISR(&isrMux);
  if (now - isrLastUs >= TMIN_US) {        // reject noise / impossible speeds
    isrLastUs = now;
    isrCount = isrCount + 1;
  }
  portEXIT_CRITICAL_ISR(&isrMux);
}

// ============================ Speed ============================
void measureSpeed() {
  portENTER_CRITICAL(&isrMux);
  unsigned long c = isrCount;
  unsigned long t = isrLastUs;
  portEXIT_CRITICAL(&isrMux);
  unsigned long nowUs = micros();

  unsigned long dc = c - prevCount;
  if (dc > 0) {
    // blink: toggle twice per revolution = one blink per turn
    ledPulses += dc;
    while (ledPulses >= LED_TOGGLE_PULSES) {
      ledPulses -= LED_TOGGLE_PULSES;
      digitalWrite(PIN_LED, !digitalRead(PIN_LED));
    }
    unsigned long dt = t - prevPulseUs;
    // dt > timeout means we are starting from rest: the first pulse has no
    // valid previous edge, so skip this one estimate.
    if (dt > 0 && dt <= ZERO_TIMEOUT_US) rpm = RPM_K * (float)dc / (float)dt;
    prevCount = c;
    prevPulseUs = t;
  } else {
    unsigned long since = nowUs - prevPulseUs;
    if (since > ZERO_TIMEOUT_US) {
      rpm = 0;
    } else if (since > 0) {
      // No pulse yet this window: true speed is at most one pulse per 'since'.
      float bound = RPM_K / (float)since;
      if (rpm > bound) rpm = bound;
    }
  }
  rpmSmooth += 0.1 * (rpm - rpmSmooth);   // display only, control uses rpm
}

// ============================ Actuator ============================
void writeMotor(int v) {
  v = constrain(v, 0, PWM_MAX);
  pwmWrite(PIN_MOTOR, MOTOR_PWM_CH, (uint32_t)v);
}

float feedforward(float r) {
  return (r > 0) ? ffOffset + ffGain * r : 0;
}

int slewToward(int current, int target) {
  if (target > current) return min(current + MANUAL_SLEW, target);
  return max(current - MANUAL_SLEW, target);
}

// ============================ Public setters (for ESP32 link later) ============================
// Returns false and changes nothing if the value is out of range.
bool setStirSetpoint(float rpmSp) {
  if (rpmSp != 0 && (rpmSp < SP_MIN || rpmSp > SP_MAX)) return false;
  if (mode != MODE_AUTO) {
    // Bumpless transfer: start the ramp from the current speed and preload
    // the integrator so the PWM doesn't jump.
    ref = rpm;
    integ = constrain(pwmOut - feedforward(ref), -I_LIMIT, I_LIMIT);
  }
  setpoint = rpmSp;
  stallTicks = 0;
  mode = MODE_AUTO;
  return true;
}

bool setStirManual(int pwm) {
  if (pwm < 0 || pwm > PWM_MAX) return false;
  manualTarget = pwm;
  stallTicks = 0;
  mode = MODE_MANUAL;
  return true;
}

void stopMotor() {
  mode = MODE_OFF;
  setpoint = ref = integ = 0;
  manualTarget = pwmOut = 0;
  stallTicks = 0;
  writeMotor(0);
}

float getStirRPM()      { return rpm; }
float getStirSetpoint() { return setpoint; }
int   getStirPWM()      { return pwmOut; }

// ============================ Control step (every 10 ms) ============================
void controlStep() {
  switch (mode) {
    case MODE_AUTO: {
      float step = rampRate * TS;
      if (ref < setpoint) ref = min(ref + step, setpoint);
      else                ref = max(ref - step, setpoint);

      if (setpoint == 0 && ref == 0) {   // finished ramping down
        stopMotor();
        return;
      }

      err = ref - rpm;
      float ff = feedforward(ref);
      float uTry = ff + Kp * err + integ;
      bool satHigh = uTry >= PWM_MAX;
      bool satLow  = uTry <= 0;
      // Anti-windup: don't integrate further into saturation.
      if (!(satHigh && err > 0) && !(satLow && err < 0)) {
        integ = constrain(integ + Ki * err * TS, -I_LIMIT, I_LIMIT);
      }
      float u = ff + Kp * err + integ;
      pwmOut = constrain((int)(u + 0.5), 0, PWM_MAX);
      break;
    }

    case MODE_MANUAL:
      pwmOut = slewToward(pwmOut, manualTarget);
      err = 0;
      if (manualTarget == 0 && pwmOut == 0) { stopMotor(); return; }
      break;

    case MODE_SWEEP:
      sweepStep();
      err = 0;
      break;

    default:   // OFF, FAULT
      pwmOut = 0;
      err = 0;
      break;
  }

  // Stall protection: lots of drive but the shaft isn't turning.
  if (mode != MODE_OFF && mode != MODE_FAULT && pwmOut >= STALL_PWM && rpm == 0) {
    if (++stallTicks >= STALL_TICKS) {
      stopMotor();
      mode = MODE_FAULT;
      Serial.println(F("# FAULT: stall (PWM high, no pulses). Motor off. Check wiring/shaft, then send s/m/x."));
    }
  } else {
    stallTicks = 0;
  }

  writeMotor(pwmOut);
}

// ============================ Open-loop sweep ============================
void startSweep() {
  stopMotor();
  sweepIdx = 0;
  sweepTick = 0;
  sweepSum = 0;
  mode = MODE_SWEEP;
  Serial.println(F("# SWEEP start: pwm,rpm (3 s per step, mean of last 1 s)"));
}

void sweepStep() {
  int target = min((int)sweepIdx * SWEEP_STEP, PWM_MAX);
  pwmOut = slewToward(pwmOut, target);
  sweepTick++;
  if (sweepTick > SWEEP_HOLD_TICKS - SWEEP_AVG_TICKS) sweepSum += rpm;
  if (sweepTick < SWEEP_HOLD_TICKS) return;

  sweepPwm[sweepIdx] = target;
  sweepRpm[sweepIdx] = sweepSum / SWEEP_AVG_TICKS;
  Serial.print(F("SWEEP,")); Serial.print(target); Serial.print(',');
  Serial.println(sweepRpm[sweepIdx], 1);

  sweepIdx++;
  sweepTick = 0;
  sweepSum = 0;
  if (sweepIdx >= SWEEP_POINTS) finishSweep();
}

void finishSweep() {
  // Least-squares fit PWM = a*RPM + b over the points where the shaft turned.
  float n = 0, sx = 0, sy = 0, sxx = 0, sxy = 0;
  int breakaway = -1;
  for (byte i = 0; i < SWEEP_POINTS; i++) {
    if (sweepRpm[i] < 30) continue;
    if (breakaway < 0) breakaway = (int)sweepPwm[i];
    n++; sx += sweepRpm[i]; sy += sweepPwm[i];
    sxx += sweepRpm[i] * sweepRpm[i]; sxy += sweepRpm[i] * sweepPwm[i];
  }
  Serial.print(F("# SWEEP done. Max RPM at PWM 1023: "));
  Serial.println(sweepRpm[SWEEP_POINTS - 1], 1);
  Serial.print(F("# First PWM step that turned the shaft: "));
  Serial.println(breakaway);
  float den = n * sxx - sx * sx;
  if (n >= 3 && den != 0) {
    float a = (n * sxy - sx * sy) / den;
    float b = (sy - a * sx) / n;
    Serial.print(F("# Suggested feedforward, send:  ff "));
    Serial.print(a, 4); Serial.print(' '); Serial.println(b, 1);
  } else {
    Serial.println(F("# Not enough moving points to fit feedforward."));
  }
  // ramp back down to zero
  mode = MODE_MANUAL;
  manualTarget = 0;
}

// ============================ Statistics & logging ============================
void updateWindow() {
  if (winTick == 0) { winSum = 0; winPwmSum = 0; winMin = 1e9; winMax = -1e9; winInBand = 0; }
  winSum += rpm;
  winPwmSum += pwmOut;
  if (rpm < winMin) winMin = rpm;
  if (rpm > winMax) winMax = rpm;
  if (mode == MODE_AUTO && fabs(setpoint - rpm) <= SPEC_BAND) winInBand++;
  if (++winTick >= WINDOW_TICKS) {
    lastWin.mean      = winSum / WINDOW_TICKS;
    lastWin.minV      = winMin;
    lastWin.maxV      = winMax;
    lastWin.pwmMean   = winPwmSum / WINDOW_TICKS / PWM_MAX;   // duty 0-1
    lastWin.inBandPct = 100.0 * winInBand / WINDOW_TICKS;
    winTick = 0;
  }
}

void printCsvHeader() {
  Serial.println(F("t_ms,mode,setpoint,ref,rpm,rpm_avg,pwm,err"));
}

void logLine() {
  if (logMode == 1) {
    Serial.print(millis());        Serial.print(',');
    Serial.print((int)mode);       Serial.print(',');
    Serial.print(setpoint, 0);     Serial.print(',');
    Serial.print(ref, 1);          Serial.print(',');
    Serial.print(rpm, 1);          Serial.print(',');
    Serial.print(rpmSmooth, 1);    Serial.print(',');
    Serial.print(pwmOut);          Serial.print(',');
    Serial.println(err, 1);
  } else if (logMode == 2) {
    // Arduino IDE 2 Serial Plotter: label:value pairs; +/-20 RPM spec band shown
    Serial.print(F("Setpoint:")); Serial.print(setpoint, 0);
    Serial.print(F(",Lower:"));   Serial.print(setpoint - SPEC_BAND, 0);
    Serial.print(F(",Upper:"));   Serial.print(setpoint + SPEC_BAND, 0);
    Serial.print(F(",RPM:"));     Serial.print(rpm, 1);
    Serial.print(F(",RPM_avg:")); Serial.print(rpmSmooth, 1);
    Serial.print(F(",PWM:"));     Serial.println(pwmOut);
  }
}

const char* modeName() {
  switch (mode) {
    case MODE_OFF:    return "OFF";
    case MODE_AUTO:   return "AUTO";
    case MODE_MANUAL: return "MANUAL";
    case MODE_SWEEP:  return "SWEEP";
    default:          return "FAULT";
  }
}

void printStatus() {
  Serial.print(F("# mode="));      Serial.print(modeName());
  Serial.print(F(" setpoint="));   Serial.print(setpoint, 0);
  Serial.print(F(" ref="));        Serial.print(ref, 1);
  Serial.print(F(" rpm="));        Serial.print(rpm, 1);
  Serial.print(F(" pwm="));        Serial.println(pwmOut);
  Serial.print(F("# Kp="));        Serial.print(Kp, 4);
  Serial.print(F(" Ki="));         Serial.print(Ki, 4);
  Serial.print(F(" ffGain="));     Serial.print(ffGain, 4);
  Serial.print(F(" ffOffset="));   Serial.print(ffOffset, 1);
  Serial.print(F(" ramp="));       Serial.println(rampRate, 0);
  Serial.print(F("# last 1 s: mean=")); Serial.print(lastWin.mean, 1);
  Serial.print(F(" min="));        Serial.print(lastWin.minV, 1);
  Serial.print(F(" max="));        Serial.print(lastWin.maxV, 1);
  Serial.print(F(" duty="));       Serial.print(lastWin.pwmMean, 3);
  Serial.print(F(" in+/-20="));    Serial.print(lastWin.inBandPct, 0);
  Serial.println('%');
}

void printHelp() {
  Serial.println(F("# Commands: s <rpm> | m <pwm> | w | kp <v> | ki <v> | ff <gain> <offset> | r <rpm/s> | l <0|1|2> | x | ?"));
}

// ============================ Serial commands ============================
void handleCommand(char* line) {
  for (char* p = line; *p; p++) *p = tolower(*p);
  char* cmd = strtok(line, " ,");
  char* a1  = strtok(NULL, " ,");
  char* a2  = strtok(NULL, " ,");
  if (!cmd) return;

  if (!strcmp(cmd, "s") && a1) {
    if (setStirSetpoint(atof(a1))) { Serial.print(F("# setpoint ")); Serial.println(setpoint, 0); }
    else Serial.println(F("# rejected: setpoint must be 0 or 500-1500"));
  } else if (!strcmp(cmd, "m") && a1) {
    if (setStirManual(atoi(a1))) { Serial.print(F("# manual pwm ")); Serial.println(manualTarget); }
    else Serial.println(F("# rejected: pwm must be 0-1023"));
  } else if (!strcmp(cmd, "w")) {
    startSweep();
  } else if (!strcmp(cmd, "kp") && a1) {
    Kp = max(0.0, atof(a1)); printStatus();
  } else if (!strcmp(cmd, "ki") && a1) {
    Ki = max(0.0, atof(a1)); printStatus();
  } else if (!strcmp(cmd, "ff") && a1 && a2) {
    ffGain = atof(a1); ffOffset = atof(a2); printStatus();
  } else if (!strcmp(cmd, "r") && a1) {
    float r = atof(a1);
    if (r > 0) { rampRate = r; printStatus(); }
    else Serial.println(F("# rejected: ramp must be > 0"));
  } else if (!strcmp(cmd, "l") && a1) {
    logMode = constrain(atoi(a1), 0, 2);
    if (logMode == 1) printCsvHeader();
  } else if (!strcmp(cmd, "x")) {
    stopMotor(); Serial.println(F("# stopped"));
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
  pinMode(PIN_HALL, INPUT_PULLUP);   // internal pull-up goes to 3.3 V
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_MOTOR, OUTPUT);
  digitalWrite(PIN_MOTOR, LOW);

  // 10-bit PWM at 8 kHz on the motor pin: 0-1023 = 0-100 % of the 5 V supply.
  pwmBegin(PIN_MOTOR, MOTOR_PWM_CH, MOTOR_PWM_HZ, MOTOR_PWM_BITS);
  writeMotor(0);

  attachInterrupt(digitalPinToInterrupt(PIN_HALL), hallISR, RISING);
  Serial.begin(115200);
  Serial.println(F("# Bioreactor stirring controller ready."));
  printHelp();

  nextTickUs = micros() + TS_US;
}

void loop() {
  unsigned long now = micros();
  if ((long)(now - nextTickUs) >= 0) {
    nextTickUs += TS_US;
    if ((long)(now - nextTickUs) >= 0) nextTickUs = now + TS_US;  // fell behind: resync

    measureSpeed();
    controlStep();
    updateWindow();
    if (++logTick >= LOG_EVERY_TICKS) { logTick = 0; logLine(); }
  }
  pollSerial();
}
