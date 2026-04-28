#pragma once
#include <Arduino.h>

class ABValve {
public:
  static const uint8_t NUM_VALVES = 4;

  // constructor
  // Input pin numbers for the four valves
  ABValve(uint8_t v1Pin, uint8_t v2Pin,
          uint8_t v3Pin, uint8_t v4Pin);

  // Initialize the valve pins
  void setup();

  // trun on the (i+1)th pump's valve (idx: 1, 2, 3, 4)
  void valveOn(uint8_t Pump_idx);

  // turn off the (i+1)th pump's valve (idx: 1, 2, 3, 4)
  void valveOff(uint8_t Pump_idx);

  // turn off all valves
  void allOff();

  // get the pin number of the pump (idx: 1, 2, 3, 4)
  uint8_t getPin(uint8_t Pump_idx) const;

private:
  uint8_t _pins[NUM_VALVES];
};
