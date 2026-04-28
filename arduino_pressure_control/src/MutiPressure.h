#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_MPRLS.h"

class PressureMux {
public:   
  // Class Constructor
  explicit PressureMux(uint8_t tcaAddr = 0x70, uint8_t bottle_CH = 4, 
    uint8_t pump1_CH = 0, uint8_t pump2_CH = 1, uint8_t pump3_CH = 2, uint8_t pump4_CH = 3);

  // Initialize Serial, Wire, and do sensor presence checks
  void setup();

  // Read both sensors and print; delayMs controls pacing
  void loop(int delayMs = 500);

  // Check if everything is correctly setup. 
  // TODO: auto-check for TCA
  void checkSetup();

  // Get Pressure of the given pump/bottle channel
  float getPressure(uint8_t pump_CH);

  // Assign xx_hPa and xx_PSI (type Float) for all pump/bottle.
  void getAllPressure();

  // Call by: object.xx_hPa. Float 
  float bottle_hPa, pump1_hPa, pump2_hPa, pump3_hPa, pump4_hPa;
  // Call by: object.xx_PSI. Float 
  float bottle_PSI, pump1_PSI, pump2_PSI, pump3_PSI, pump4_PSI;
  
  // print all channle's pressure: hPa & PSI
  // TODO: with pump/bottle name
  void printAllPressure();


private:
  
  // Select the channel
  void tcaSelect(uint8_t channel);  

  // Check if the sensor on the channel is correctly setup
  bool checkSensorOnChannel(uint8_t ch);  
  
  // Check if the TCA is correctly setup
  // TODO: auto-check
  void checkTCA();

  uint8_t _tcaAddr;

  // All pressure List, every 2 consecutive pairs are {xx_hPa, xx_PSI}
  float pressureList[10];
  
  // All Channel Index for pump, in order of {bottle, pump_1, pump_2, pump_3, pump_4}
  uint8_t pumpChanList[5];


  // MPRLS uses -1, -1 when RST/EOC are unused
  Adafruit_MPRLS _mpr;
  const float hPa_to_PSI = 1.0 / 68.947572932;
  
  // For debug
  bool isSensorWorking = 1;

};
