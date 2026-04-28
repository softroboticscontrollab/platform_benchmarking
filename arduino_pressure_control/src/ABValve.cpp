#include "ABValve.h"

ABValve::ABValve(uint8_t v1Pin, uint8_t v2Pin,
                 uint8_t v3Pin, uint8_t v4Pin) {
  _pins[0] = v1Pin;
  _pins[1] = v2Pin;
  _pins[2] = v3Pin;
  _pins[3] = v4Pin;
}

void ABValve::setup() {
    for (uint8_t i = 0; i < NUM_VALVES; i++) {
        pinMode(_pins[i], OUTPUT);
        digitalWrite(_pins[i], LOW); // Ensure all valves are off at start
    }
}

void ABValve::valveOn(uint8_t Pump_idx) {
    uint8_t idx = Pump_idx - 1; // Convert to 0-based index
    if (idx < NUM_VALVES) {
        digitalWrite(_pins[idx], HIGH);
        // Debugging info
        // Serial.print("Valve "); Serial.print(Pump_idx); Serial.println(" ON");
    }
}

void ABValve::valveOff(uint8_t Pump_idx) {
    uint8_t idx = Pump_idx - 1; // Convert to 0-based index
    if (idx < NUM_VALVES) {
        digitalWrite(_pins[idx], LOW);
        // Debugging info
        // Serial.print("Valve "); Serial.print(Pump_idx); Serial.println(" OFF");
    }
}

void ABValve::allOff() {
    for (uint8_t i = 0; i < NUM_VALVES; i++) {
        digitalWrite(_pins[i], LOW);
    }
}

uint8_t ABValve::getPin(uint8_t Pump_idx) const {
    if (Pump_idx > 0 && Pump_idx <= NUM_VALVES) {
        return _pins[Pump_idx - 1]; // Convert to 0-based index
    }
    return 255; // Invalid index
}