# Bioreactor Dashboard

The dashboard (Node-RED) and the ESP32 don't talk directly. Both connect to the MQTT broker (`broker.hivemq.com`, port 1883), which passes messages between them. Each message is sent on a **topic**, a name both sides agree on.

## ESP32 Topic Addresses:

Subscribe to:
- ucl-bioreactor-team28/setpoints/set -> new setpoints from the dashboard, sent when the user presses Apply

Publish to:
- ucl-bioreactor-team28/telemetry -> live readings, every 1 second. Drives the dashboard's gauges, charts and CSV log.
- ucl-bioreactor-team28/setpoints/state -> the setpoints the ESP32 is actually using. Send at startup and after every change. Retained.
- ucl-bioreactor-team28/status -> `online` when connected, `offline` when not. Retained, with `offline` as the Last Will.

**Retained** = the broker keeps the latest message, so the dashboard gets it immediately when it opens.
**Last Will** = a message the broker sends automatically if the ESP32 disconnects unexpectedly (power or WiFi loss).

## Message Formats

**setpoints/set** and **setpoints/state**
```json
{ "temperature_C": 30, "pH": 5, "rpm": 1000 }
```
Clamp to: 25–35 °C, pH 3–7, 500–1500 RPM. The broker is public, so anyone could send a bad value — the ESP32 should never act on one outside these ranges.

**telemetry**
```json
{
  "temperature_C": 29.8,
  "pH": 5.03,
  "rpm": 1004,
  "actuators": { "heater": 0.62, "motor": 0.67, "acid_pump": 0, "base_pump": 0.3 },
  "setpoints": { "temperature_C": 30, "pH": 5, "rpm": 1000 }
}
```
- `temperature_C`, `pH`, `rpm` — sensor readings in real units (converted from raw values on the ESP32).
- `actuators` — how hard each output is working: 0 = off, 1 = fully on (e.g. PWM 128/255 ≈ 0.5).
- `setpoints` — the current targets, so each reading can be compared to its target.

Field names are case-sensitive and must match exactly.

## Testing Without Hardware

The **Fake device** tab in Node-RED sends and receives these exact messages, so it can be used as a reference. Disable it once the real ESP32 is connected.