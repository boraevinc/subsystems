/*
  WIRING
    A0 <- thermistor divider 
    D9 -> heater MOSFET gate
    
  HOW IT WORKS (every second)
    1. Read the temperature.
    2. PI control: power = Kp*error + (integral of Ki*error)
         Kp part: more power the further below the setpoint we are.
         I part : slowly learns the power needed to make up for heat
                  lost to the room, so it settles right on the setpoint.
    3. Heater power is capped at MAX_WATTS, because 12 V across 3 ohm would
       be 48 W and the element is only rated 30 W.
    Safety: the heater turns off if
      - the thermistor reading is impossible (wire off or shorted), or
      - the water goes above MAX_TEMP.

  SERIAL MONITOR (115200 baud, "Newline")
    s 30     hold 30 C (allowed 25-35)
    x        heater off   (s 0 does the same)
    p 10     fixed 10 W, no control (testing; 0 to MAX_WATTS)
  It prints "Temp:...,Setpoint:...,Watts:..." every second (Serial Plotter
  works too).

  CALIBRATION
    Thermistor next to a lab thermometer in stirred water, wait 1 min.
    TEMP_OFFSET = thermometer reading - "Temp" reading. Type it below and
    upload again.

  GAINS
    KP and KI are calculated from the vessel constants (VESSEL_LITRES,
    H_LOSS) and MAX_WATTS, using the Control lecture's design equations.

  MEASURING (once)
    VESSEL_LITRES: measure the water with a measuring jug.
    H_LOSS: send "s 30" and let it hold for ~20 min. Note the steady
      "Watts" and the room temperature. H_LOSS = watts / (30 - room temp).
      e.g. 5 W at 20 C room -> 5 / 10 = 0.5. (KP only changes by this
      amount, so a rough value is fine.)
    Type them below and upload again. The new KP and KI are printed at
    start-up.

  TUNING
    Swings back and forth around the setpoint: lower ZETA (KP follows it).
    Sits slightly off the setpoint for a long time: raise KI_MIN.
    Overshoots on warm-up: usually the thermistor is slow to see the heat.
    Keep the stirrer on and put the thermistor in the moving water, not
    touching the heater or the wall. (Small volumes, about 0.5 L, are the
    hardest: the water warms faster than the sensor reports it.)
*/

// ---------- Pins ----------
const int PIN_THERM  = A0;
const int PIN_HEATER = D9;

// ---------- Thermistor (edit TEMP_OFFSET after calibrating) ----------
const float VCC_MV      = 3300.0;   // 3V3 pin: measure it with a multimeter
const float R_FIXED     = 10000.0;  // top resistor of the divider
const float R25         = 10000.0;  // thermistor resistance at 25 C
const float BETA        = 4220.0;   // from the thermistor datasheet
const float TEMP_OFFSET = 0.0;      // thermometer - sketch reading

// ---------- Heater ----------
const float HEATER_VOLTS = 12.0;
const float HEATER_OHMS  = 3.0;
const float FULL_WATTS   = HEATER_VOLTS * HEATER_VOLTS / HEATER_OHMS;   // 48 W
const float MAX_WATTS    = 28.0;    // stay under the 30 W rating

// ---------- Vessel constants (MEASURE these, see MEASURING above) ----------
const float VESSEL_LITRES = 1.0;    // water in the vessel
const float H_LOSS = 0.5;           // heat loss to the room, watts per C above room temp

// ---------- Design choices (Control lecture, slides 45-48) ----------
const float DESIGN_RISE = 15.0;     // C: typical warm-up, e.g. 15 C inlet -> 30 C
const float ZETA = 8.0;             // damping factor (the slides use 8 for heaters)

// ---------- PI gains, worked out from the lecture's equation (20) ----------
//   wn = P / (m*c*rise)    Ki = m*c*wn^2    Kp = 2*zeta*m*c*wn - h
const float MC = VESSEL_LITRES * 4180.0;                // J per C (1 L of water = 1 kg)
const float WN = MAX_WATTS / (MC * DESIGN_RISE);        // rad/s
const float KP = 2 * ZETA * MC * WN - H_LOSS;           // watts per C of error
// The slides design for an 80 L tank that takes 4 hours to heat. For our ~1 L
// the formula's Ki is tiny (~0.0008), which simulated as ~0.5 C off the setpoint
// for hours, so Ki is never allowed below KI_MIN.
const float KI_MIN = 0.01;
const float KI = max(MC * WN * WN, KI_MIN);             // watts per (C x second)
const float I_BAND = 2.0;           // only build up the integral within 2 C

// ---------- Safety ----------
const float MAX_TEMP = 40.0;                  // C

// ---------- State ----------
float setpoint = 0;          // 0 = not controlling
float fixedWatts = 0;        // used by the "p" test command
float integral = 0;          // watts, built up by the I part
float watts = 0;             // what the heater is getting now
unsigned long lastControlTime = 0;

// Average 32 readings, then convert divider voltage -> resistance -> C.
// Returns -999 if the reading is impossible.
float readTemp() {
  long sum = 0;
  for (int i = 0; i < 32; i++) sum += analogReadMilliVolts(PIN_THERM);
  float mv = sum / 32.0;
  if (mv < 50 || mv > 3000) return -999;              // shorted or disconnected
  float rTherm = R_FIXED * mv / (VCC_MV - mv);        // voltage divider formula
  float kelvin = 1.0 / (1.0 / 298.15 + log(rTherm / R25) / BETA);   // beta equation
  return kelvin - 273.15 + TEMP_OFFSET;
}

void setHeater(float w) {
  watts = constrain(w, 0.0f, MAX_WATTS);
  ledcWrite(PIN_HEATER, (int)(watts / FULL_WATTS * 1023));   // 10-bit PWM
}

void heaterOff(const char* reason) {
  setpoint = 0;
  fixedWatts = 0;
  integral = 0;
  setHeater(0);
  Serial.print("# HEATER OFF: ");
  Serial.println(reason);
}

void handleSerial() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  line.toLowerCase();
  float value = line.substring(line.indexOf(' ') + 1).toFloat();

  if (line.startsWith("s ") && value >= 25 && value <= 35) {
    setpoint = value;
    fixedWatts = 0;
    Serial.print("# setpoint ");
    Serial.println(setpoint);
  } else if (line == "x" || line == "s 0") {
    heaterOff("by command");
  } else if (line.startsWith("p ") && value >= 0 && value <= MAX_WATTS) {
    setpoint = 0;
    integral = 0;
    fixedWatts = value;
    Serial.print("# fixed power ");
    Serial.println(fixedWatts);
  } else {
    Serial.println("# commands: s <25-35> | x | p <0-28 watts>");
  }
}

void setup() {
  // Heater off first.
  pinMode(PIN_HEATER, OUTPUT);
  digitalWrite(PIN_HEATER, LOW);
  ledcAttach(PIN_HEATER, 1000, 10);   // 1 kHz PWM, duty 0-1023
  ledcWrite(PIN_HEATER, 0);

  Serial.begin(115200);
  delay(1500);                        // let the USB serial connect
  Serial.setTimeout(100);
  Serial.println("# Heater controller (simple). Heater OFF. Send s 30 to start.");
  Serial.print("# gains from the vessel constants: KP = ");
  Serial.print(KP, 2);
  Serial.print("  KI = ");
  Serial.println(KI, 4);
}

void loop() {
  handleSerial();

  if (millis() - lastControlTime < 1000) return;     // run once a second
  lastControlTime = millis();

  float temp = readTemp();

  // ---- Safety checks (apply in every mode) ----
  bool heating = (setpoint > 0 || fixedWatts > 0);
  if (heating && temp == -999)   heaterOff("thermistor reading impossible (wire off or shorted?)");
  if (heating && temp > MAX_TEMP) heaterOff("water above MAX_TEMP");

  // ---- Decide the heater power ----
  if (setpoint > 0) {
    float error = setpoint - temp;
    // Build up the integral only near the setpoint (stops it winding up during
    // the long warm-up), and only while it can actually change the output.
    if (fabs(error) < I_BAND) integral = constrain(integral + KI * error, 0.0f, MAX_WATTS);
    setHeater(KP * error + integral);
  } else {
    setHeater(fixedWatts);                             // 0 when off
  }

  // ---- Print ----
  Serial.print("Temp:");         Serial.print(temp, 2);
  Serial.print(",Setpoint:");    Serial.print(setpoint, 1);
  Serial.print(",Watts:");       Serial.println(watts, 1);
}
