#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPRLS.h>

class PressureSensor {
public:
  void setup(uint32_t baud = 9600);
  void loop(uint32_t period_ms = 1000);

private:
  Adafruit_MPRLS mpr{-1, -1};  // no RST/EOC pins
  uint32_t last_ms = 0;
};
