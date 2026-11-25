// ESP32 DevKitC (WROVER) — UART link test
// Prints everything the Nano sends to the Serial Monitor,
// and forwards anything you type in the Serial Monitor to the Nano.
//
// Wiring: ESP32 GPIO26 (RX) <- Nano D1, ESP32 GPIO27 (TX) -> Nano D0, GND -> GND
// GPIO16/17 are NOT used: on WROVER modules they're taken by the PSRAM.

#define RXD2 26
#define TXD2 27

void setup() {
  Serial.begin(115200);                            // USB, to your laptop
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);   // link to the Nano
  Serial2.setTimeout(50);
  Serial.setTimeout(50);
  Serial.println("ESP32 ready. Type a command, e.g.  SET T 32.5");
}

void loop() {
  // Nano -> laptop
  if (Serial2.available()) {
    String line = Serial2.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) Serial.println("From Nano: " + line);
  }

  // Laptop -> Nano
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() > 0) {
      Serial2.println(cmd);
      Serial.println("Sent:      " + cmd);
    }
  }
}