// Arduino Nano ESP32 — UART link test
// Sends fake readings once a second on D1 (TX)
// and listens for commands from the ESP32 on D0 (RX).
//
// Wiring: Nano D1 -> ESP32 GPIO26, Nano D0 -> ESP32 GPIO27, GND -> GND
// On the Nano ESP32: Serial = USB, Serial0 = pins D0/D1.

float targetTemp = 30.0;   // °C
float targetPH   = 5.0;
int   targetRPM  = 1000;

unsigned long lastSend = 0;

void setup() {
  Serial.begin(115200);     // USB, for debugging
  Serial0.begin(115200);    // link to the ESP32 (must match the ESP32's speed)
  Serial0.setTimeout(50);   // don't block long waiting for a newline
}

void loop() {
  // 1. Send fake readings once a second (later: real sensor values)
  if (millis() - lastSend >= 1000) {
    lastSend = millis();
    float temp = targetTemp + random(-50, 51) / 100.0;
    float ph   = targetPH   + random(-10, 11) / 100.0;
    int   rpm  = targetRPM  + random(-20, 21);
    Serial0.printf("T:%.2f;PH:%.2f;RPM:%d\n", temp, ph, rpm);
  }

  // 2. Handle commands from the ESP32, e.g. "SET T 32.5"
  if (Serial0.available()) {
    String line = Serial0.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) handleCommand(line);
  }
}

void handleCommand(const String &line) {
  // Expected format: SET <T|PH|RPM> <value>
  if (!line.startsWith("SET ")) { Serial0.println("ERR " + line); return; }

  int space = line.indexOf(' ', 4);
  if (space < 0) { Serial0.println("ERR " + line); return; }

  String key   = line.substring(4, space);
  float  value = line.substring(space + 1).toFloat();

  // Clamp to safe limits — replace these with the ranges from your spec
  if (key == "T") {
    targetTemp = constrain(value, 25.0, 35.0);
    Serial0.printf("ACK T %.2f\n", targetTemp);
  } else if (key == "PH") {
    targetPH = constrain(value, 3.0, 7.0);
    Serial0.printf("ACK PH %.2f\n", targetPH);
  } else if (key == "RPM") {
    targetRPM = constrain((int)value, 500, 1500);
    Serial0.printf("ACK RPM %d\n", targetRPM);
  } else {
    Serial0.println("ERR " + line);
  }
}