#include "Arduino.h" 
#include "MutiPressure.h"
#include "PumpController.h"

/**
 * @file Unit1_mutiPressure&PumpTest.cpp
 * @brief Test program for the PressureMux and PumpController classes.  
 * 
 * This is a unit test for the PressureMux and PumpController classes. 
 * 
 * This program initializes the PressureMux to read from multiple pressure sensors 
 * and uses the PumpController to maintain a desired pressure in the reservoir/bottle. 
 * The program periodically reads the current bottle pressure and updates the pump control output 
 * to achieve a specified target pressure. 
 * Debug information about the current bottle pressure 
 * and whether the target pressure has been achieved is printed to Serial.
 * 
 */

const unsigned long Ts_mux_ms = 50; // sensor sampling period
unsigned long last_mux_ms = 0;
bool achieve = 0; // whether desired pressure is achieved
float kp_test = 13.0f; // P gain for pump controller test
float ki_test = 1.0f; // I gain for pump controller test

PressureMux mutiSensor_test;
PumpController pump1_control(5, Ts_mux_ms, kp_test, ki_test); // pump connected to pin 5


void setup(){
  mutiSensor_test.setup();
  pump1_control.setup();
};

void loop(){
 
  unsigned long now = millis();
  if (now - last_mux_ms >= Ts_mux_ms) {
    last_mux_ms = now;
    mutiSensor_test.getAllPressure();   // take readings from all sensors
  }
  float curr_bottlePress = mutiSensor_test.bottle_hPa;
  float desiredPressure = 1060.0f; // desired pressure in hPa

  pump1_control.update(desiredPressure, curr_bottlePress);


  /* DEBUGS AND OUTPUT*/
  if (curr_bottlePress < desiredPressure) {
    achieve = 0;
  }
  else if(curr_bottlePress >= desiredPressure){
    pump1_control.resetIntegral(); // reset integral term
    achieve = 1;
  }
  
  Serial.println("The bottle pressure is: " + String(curr_bottlePress) + " hPa");
  if (achieve) {
    Serial.println("Desired pressure achieved.");
  }


};

