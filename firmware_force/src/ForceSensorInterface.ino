#include "HX711.h"

HX711 scale;

const int LOAD_CELL_DOUT_PIN = 6;
const int LOAD_CELL_SCK_PIN = 7;
const float LOAD_CELL_SCALE = 1349.5144; // Calibration scale factor
const long LOAD_CELL_OFFSET = -105680;   // Calibration offset

void setup() {
  Serial.begin(115200);
  delay(1000);  // Allow time for Serial to initialize
  
  scale.begin(LOAD_CELL_DOUT_PIN, LOAD_CELL_SCK_PIN);
  scale.set_offset(LOAD_CELL_OFFSET);
  scale.set_scale(LOAD_CELL_SCALE);
  scale.tare();  // Zero the scale
  
  Serial.println("<STATUS,FORCE_SENSOR_READY>");
}

void loop() {
  // Continuous streaming mode: send every sample as soon as it's ready.
  static bool streaming = true;
  static unsigned long last_error_time = 0;
  const unsigned long error_suppress_ms = 200; // suppress repeated ERROR messages

  // Handle incoming control commands without blocking streaming.
  while (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toUpperCase();
    if (cmd == "TARE") {
      scale.tare();
      Serial.println("<STATUS,TARE_COMPLETE>");
    } else if (cmd == "STREAM_OFF") {
      streaming = false;
      Serial.println("<STATUS,STREAM_OFF>");
    } else if (cmd == "STREAM_ON") {
      streaming = true;
      Serial.println("<STATUS,STREAM_ON>");
    } else if (cmd.length() > 0) {
      // ignore other commands in streaming mode
      Serial.print("<ERROR,UNKNOWN_COMMAND:");
      Serial.print(cmd);
      Serial.println(">");
    }
  }

  if (streaming) {
    if (scale.is_ready()) {
      // minimal averaging for lowest latency; change to get_units(1) if desired
      float force = scale.get_units(1);
      Serial.print("<FORCE,");
      Serial.print(force, 3);
      Serial.println(">");
      last_error_time = 0;
    } else {
      if (last_error_time == 0 || (millis() - last_error_time) > error_suppress_ms) {
        // Serial.println("<ERROR,SENSOR_NOT_READY>");
        last_error_time = millis();
      }
    }
  }

  // No explicit delay: loop runs as fast as the MCU allows; HX711 readiness throttles output.
}