#include "HX711.h"
#include <EEPROM.h>

HX711 scale;

const int LOAD_CELL_DOUT_PIN = 6;
const int LOAD_CELL_SCK_PIN = 7;
// Defaults; will be overridden by EEPROM if saved
float LOAD_CELL_SCALE = 1349.5144; // Calibration scale factor
long LOAD_CELL_OFFSET = -105680;   // Calibration offset

// EEPROM addresses for persisted calibration (8 bytes for float, 4 for long)
const int EEPROM_MAGIC_ADDR = 0; // store a magic value to detect saved cal
const int EEPROM_MAGIC = 0xCA1B;
const int EEPROM_OFFSET_ADDR = 4; // long (4 bytes)
const int EEPROM_SCALE_ADDR = 8;  // float (4 bytes)

// Calibration averaging window
const int CAL_AVG = 20;

// calibration state
bool calibrating = false;

void setup() {
  Serial.begin(115200);
  delay(1000);  // Allow time for Serial to initialize
  
  scale.begin(LOAD_CELL_DOUT_PIN, LOAD_CELL_SCK_PIN);
  // load persisted calibration if present
  int magic = 0;
  EEPROM.get(EEPROM_MAGIC_ADDR, magic);
  if (magic == EEPROM_MAGIC) {
    EEPROM.get(EEPROM_OFFSET_ADDR, LOAD_CELL_OFFSET);
    EEPROM.get(EEPROM_SCALE_ADDR, LOAD_CELL_SCALE);
  }
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
    } else if (cmd == "CALIBRATE" || cmd == "CAL") {
      // enter calibration mode
      calibrating = true;
      Serial.println("<STATUS,CALIBRATE_START>");
      // instruct host to remove load and confirm
      Serial.println("<STATUS,REMOVE_LOAD>");
    } else if (cmd == "CONFIRM_ZERO") {
      if (calibrating) {
        scale.tare(CAL_AVG);
        long offset = scale.get_offset();
        LOAD_CELL_OFFSET = offset;
        scale.set_offset(LOAD_CELL_OFFSET);
        Serial.print("<STATUS,ZERO_DONE,");
        Serial.print(LOAD_CELL_OFFSET);
        Serial.println(">");
        Serial.println("<STATUS,APPLY_WEIGHT>");
      } else {
        Serial.println("<ERROR,NOT_CALIBRATING>");
      }
    } else if (cmd.startsWith("SET_WEIGHT")) {
      if (calibrating) {
        // expected format: SET_WEIGHT,<grams>
        int c = cmd.indexOf(',');
        if (c > 0) {
          String val = cmd.substring(c+1);
          float grams = val.toFloat();
          if (grams > 0.0) {
            scale.calibrate_scale((uint32_t)grams, CAL_AVG);
            float s = scale.get_scale();
            LOAD_CELL_SCALE = s;
            scale.set_scale(LOAD_CELL_SCALE);
            Serial.print("<STATUS,CAL_COMPLETE,");
            Serial.print(LOAD_CELL_OFFSET);
            Serial.print(",");
            Serial.print(LOAD_CELL_SCALE, 6);
            Serial.println(">");
          } else {
            Serial.println("<ERROR,BAD_WEIGHT>");
          }
        } else {
          Serial.println("<ERROR,BAD_COMMAND>");
        }
      } else {
        Serial.println("<ERROR,NOT_CALIBRATING>");
      }
    } else if (cmd == "SAVE_CAL") {
      // persist offset and scale to EEPROM
      int magicv = EEPROM_MAGIC;
      EEPROM.put(EEPROM_MAGIC_ADDR, magicv);
      EEPROM.put(EEPROM_OFFSET_ADDR, LOAD_CELL_OFFSET);
      EEPROM.put(EEPROM_SCALE_ADDR, LOAD_CELL_SCALE);
      Serial.println("<STATUS,CAL_SAVED>");
      calibrating = false;
    } else if (cmd == "GET_CAL") {
      Serial.print("<STATUS,CAL,");
      Serial.print(LOAD_CELL_OFFSET);
      Serial.print(",");
      Serial.print(LOAD_CELL_SCALE, 6);
      Serial.println(">");
    } else if (cmd == "RESET_CAL") {
      // clear EEPROM magic so defaults apply
      int magicv = 0;
      EEPROM.put(EEPROM_MAGIC_ADDR, magicv);
      Serial.println("<STATUS,CAL_RESET>");
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