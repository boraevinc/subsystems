/*
  WIRING
    D2  <- Hall sensor signal
    D10 -> motor MOSFET gate (100R in series, 10k gate to GND)
    Nano VBUS (5 V) -> motor + and sensor +

  HOW IT WORKS
    Speed: the Hall sensor gives 70 pulses per turn of the output shaft.
      The sketch counts pulses and times them:
      RPM = pulses / 70 / (time in minutes).
    Control (every 50 ms): PI control, as in the motor lecture.
      power = KP*error + (integral of KI*error)
      The setpoint is approached gradually (RAMP_RPM_PER_S) so the gears
      never get a sudden jolt.
    Safety:
      - Motor stops if it's getting lots of power but the shaft isn't turning
        for 2 s (jammed, or Hall sensor disconnected).
      - Warns if the setpoint is faster than this motor can go at 5 V
        (an N20 at 5 V tops out around 1000-1250 RPM).

  SERIAL MONITOR (115200 baud, "Newline")
    s 1000   hold 1000 RPM (allowed 500-1500)
    x        stop   (s 0 does the same)
    p 50     fixed 50% power, no control (testing / finding the top speed)
  It prints "RPM:...,Setpoint:...,Power%:..." 10 times a second (Serial
  Plotter works too).

  GAINS
    KP and KI are calculated from the motor constants KV and T_MOTOR, using
    the PI Motor lecture's design equations. You don't type gains in: you
    measure the motor, and the sketch works out the gains.

  MEASURING KV (once, with the stirrer in the liquid)
    1. Send "p 80" (80% power = 4 V) and wait ~5 s for the speed to settle.
    2. KV = final RPM / 4.0          e.g. 1000 RPM -> KV = 250
    3. Type it below and upload again. The new KP and KI are printed at
       start-up.
  T_MOTOR: leave it at 0.15 s, the lecture's value for this same N20 motor.
    It can't be timed with "p" (that ramps up over ~0.8 s to protect the
    gears, far slower than the motor). If a speed step later overshoots by
    more than ~10 RPM, your motor is slower than the lecture's: set 0.3.

  TUNING
    Too jumpy: lower WN_RATIO (0.5 = smooth, the lecture's "large motor"
    design). Too sluggish: raise it towards 1.
*/

// ---------- Pins ----------
const int PIN_HALL  = D2;
const int PIN_MOTOR = D10;

// ---------- Sensor ----------
const int PULSES_PER_REV = 70;            // Hall pulses per output-shaft turn

// ---------- Motor constants (MEASURE these, see MEASURING below) ----------
const float KV      = 250;                // speed constant, RPM per volt (lecture demo motor)
const float T_MOTOR = 0.15;               // time constant, seconds       (lecture demo motor)

// ---------- Design choices (PI Motor lecture, slides 15-16) ----------
const float ZETA     = 1.0;               // damping: 1 = no overshoot
const float WN_RATIO = 1.0;               // speed of response, wn / wo: 0.5 smooth ... 1 fast

// ---------- PI gains, worked out from the lecture's equation (5) ----------
//   Kp = (2*zeta*wn/wo - 1) / Kv      Ki = wn^2 / (Kv * wo)      with wo = 1/T
// Those are in volts; x VOLTS_TO_PWM turns them into PWM counts (1023 per 5 V).
const float VOLTS_TO_PWM = 1023 / 5.0;
const float WO = 1.0 / T_MOTOR;
const float WN = WN_RATIO * WO;
const float KP = (2 * ZETA * WN / WO - 1) / KV * VOLTS_TO_PWM;   // PWM counts per RPM
const float KI = WN * WN / (KV * WO) * VOLTS_TO_PWM;             // PWM counts per (RPM x s)

const float RAMP_RPM_PER_S = 500;         // how fast the target speed changes
const int   PWM_MAX = 1023;               // 10-bit PWM: 1023 = full 5 V
const float DT = 0.05;                    // control every 50 ms

// ---------- State ----------
volatile unsigned long pulseCount = 0;     // changed by the interrupt
volatile unsigned long lastPulseMicros = 0;
portMUX_TYPE pulseLock = portMUX_INITIALIZER_UNLOCKED;

float setpoint = 0;         // RPM asked for (0 = stopped)
float target = 0;           // ramps towards the setpoint
float integral = 0;         // PWM counts, built up by the I part
int   fixedPwm = 0;         // used by the "p" test command
int   pwm = 0;              // what the motor is getting now
float rpm = 0;
int   stallTime = 0;        // control steps with high power but no rotation
int   maxedTime = 0;        // control steps at full power below the setpoint
unsigned long lastControlTime = 0;
int   printCounter = 0;

// Runs on every Hall pulse. Kept tiny: count it and note the time.
void IRAM_ATTR onPulse() {
  unsigned long now = micros();
  if (now - lastPulseMicros < 300) return;   // faster than ~2800 RPM = electrical noise
  portENTER_CRITICAL_ISR(&pulseLock);
  pulseCount = pulseCount + 1;
  lastPulseMicros = now;
  portEXIT_CRITICAL_ISR(&pulseLock);
}

// Speed from the pulses since last time: (pulses / 70) turns in (time) microseconds.
float measureRPM() {
  static unsigned long prevCount = 0, prevMicros = 0;
  portENTER_CRITICAL(&pulseLock);            // read both values together
  unsigned long count = pulseCount, t = lastPulseMicros;
  portEXIT_CRITICAL(&pulseLock);

  if (micros() - t > 200000) {               // no pulse for 0.2 s: stopped
    prevCount = count; prevMicros = t;
    return 0;
  }
  if (count == prevCount) return rpm;        // no new pulse yet: keep last value
  if (t - prevMicros > 200000) {             // just started again: start timing afresh
    prevCount = count; prevMicros = t;
    return rpm;
  }
  float turns = (count - prevCount) / (float)PULSES_PER_REV;
  float minutes = (t - prevMicros) / 60.0e6;
  prevCount = count; prevMicros = t;
  return turns / minutes;
}

void motorStop(const char* reason) {
  setpoint = 0;
  target = 0;
  integral = 0;
  fixedPwm = 0;
  pwm = 0;
  ledcWrite(PIN_MOTOR, 0);
  Serial.print("# MOTOR STOPPED: ");
  Serial.println(reason);
}

void handleSerial() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  line.toLowerCase();
  float value = line.substring(line.indexOf(' ') + 1).toFloat();

  if (line.startsWith("s ") && value >= 500 && value <= 1500) {
    if (setpoint == 0) target = rpm;        // ramp from the current speed
    setpoint = value;
    fixedPwm = 0;
    maxedTime = 0;
    Serial.print("# setpoint ");
    Serial.println(setpoint);
  } else if (line == "x" || line == "s 0") {
    motorStop("by command");
  } else if (line.startsWith("p ") && value >= 0 && value <= 100) {
    setpoint = 0;
    integral = 0;
    fixedPwm = (int)(value / 100.0 * PWM_MAX);
    Serial.print("# fixed power % ");
    Serial.println(value);
  } else {
    Serial.println("# commands: s <500-1500> | x | p <0-100 %>");
  }
}

void setup() {
  // Motor off first.
  pinMode(PIN_MOTOR, OUTPUT);
  digitalWrite(PIN_MOTOR, LOW);
  ledcAttach(PIN_MOTOR, 20000, 10);    // 20 kHz PWM (above hearing), duty 0-1023
  ledcWrite(PIN_MOTOR, 0);

  pinMode(PIN_HALL, INPUT_PULLUP);     // pull-up goes to 3.3 V
  attachInterrupt(digitalPinToInterrupt(PIN_HALL), onPulse, RISING);

  Serial.begin(115200);
  delay(1500);                         // let the USB serial connect
  Serial.setTimeout(100);
  Serial.println("# Stirring controller (simple). Motor OFF. Send s 1000 to start.");
  Serial.print("# gains from KV and T_MOTOR: KP = ");
  Serial.print(KP, 3);
  Serial.print("  KI = ");
  Serial.println(KI, 3);
}

void loop() {
  handleSerial();

  if (millis() - lastControlTime < 50) return;      // run every 50 ms
  lastControlTime = millis();

  rpm = measureRPM();

  if (setpoint > 0) {
    // Move the target towards the setpoint gradually (protects the gears).
    float step = RAMP_RPM_PER_S * DT;
    if (target < setpoint) target = min(target + step, setpoint);
    else                   target = max(target - step, setpoint);

    // PI control.
    float error = target - rpm;
    integral = constrain(integral + KI * error * DT, 0.0f, (float)PWM_MAX);
    pwm = constrain((int)(KP * error + integral), 0, PWM_MAX);
  } else {
    // Test mode: move gradually towards the fixed power (protects the gears).
    if (pwm < fixedPwm) pwm = min(pwm + 50, fixedPwm);
    else                pwm = max(pwm - 50, fixedPwm);
  }
  ledcWrite(PIN_MOTOR, pwm);

  // ---- Safety: lots of power but not turning for 2 s ----
  if (pwm > PWM_MAX / 2 && rpm == 0) stallTime++; else stallTime = 0;
  if (stallTime >= 40) motorStop("shaft not turning (jammed, or Hall sensor disconnected?)");

  // ---- Warning: full power for 3 s and still too slow ----
  if (setpoint > 0 && pwm == PWM_MAX && rpm < setpoint - 20) maxedTime++; else maxedTime = 0;
  if (maxedTime == 60) {
    Serial.print("# WARNING: setpoint is faster than this motor can go. Top speed here is about ");
    Serial.print(rpm, 0);
    Serial.println(" RPM.");
  }

  // ---- Print 10 times a second ----
  if (++printCounter >= 2) {
    printCounter = 0;
    Serial.print("RPM:");          Serial.print(rpm, 0);
    Serial.print(",Setpoint:");    Serial.print(setpoint, 0);
    Serial.print(",Power%:");      Serial.println(pwm * 100.0 / PWM_MAX, 0);
  }
}
