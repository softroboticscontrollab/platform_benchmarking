#include "test_pressure.h"

void PressureSensor::setup(uint32_t baud) {
  Serial.begin(baud);
  delay(50);
  Wire.begin();                 

  Serial.println(F("MPRLS Simple Test"));
  if (!mpr.begin()) {
    Serial.println(F("Failed to communicate with MPRLS (0x18). Check wiring/power."));
    // don't hard-hang; allow retry attempts later
  } else {
    Serial.println(F("Found MPRLS sensor"));
  }
}

void PressureSensor::loop(uint32_t period_ms) {
  if (millis() - last_ms < period_ms) return;
  last_ms = millis();

  float pressure_hPa = mpr.readPressure();     // NAN on failure
  Serial.print(F("Pressure (hPa): "));
  Serial.println(pressure_hPa);

  Serial.print(F("Pressure (PSI): "));
  Serial.println(pressure_hPa / 68.947572932f);

  if (isnan(pressure_hPa)) {
    // Try to re-init if read failed
    if (mpr.begin()) {
      Serial.println(F("Reinit OK"));
    } else {
      Serial.println(F("Reinit FAIL"));
    }
  }
}
