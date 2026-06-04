#include "Arduino.h" 
#include "MutiPressure.h"
#include "PumpController.h"
#include "ABValve.h"

/***************** 
 * @file Unit2_ABValve.cpp
 * @brief Test program for the ABValve class.  
 * 
 * This is a unit test for the ABValve class. 
 * 
 * This program initializes the ABValve to control 4 valves connected to specified pins. 
 * The program listens for Serial commands to turn individual valves on or off. 
 * The expected Serial command format is:
 *   V<idx><state>
 * where:
 *   - 'V' is the command prefix for valve control
 *   - <idx> is the valve index (1-4)
 *   - <state> is the desired state (0 for off, 1 for on)
 *
 * Example command to turn valve 2 on:
 *   V21
 *
 * Example command to turn valve 3 off:
 *   V30
 *
 * The program also allows printing current pressure readings from all sensors by sending the command 'R'.
 */


const unsigned long Ts_mux_ms = 100; // sensor sampling period
unsigned long last_mux_ms = 0;
bool achieve = 0; // whether desired pressure is achieved

PressureMux mutiSensor_test;
PumpController pump1_control(5, Ts_mux_ms); // pump connected to pin 5
ABValve abValve1(9, 10, 11, 12); // valves connected to pins 


void setup(){
  mutiSensor_test.setup();
  pump1_control.setup();
  abValve1.setup();
};

void loop(){
  
  /* Pump Control*/
  unsigned long now = millis();
  if (now - last_mux_ms >= Ts_mux_ms) {
    last_mux_ms = now;
    mutiSensor_test.getAllPressure();   // take readings from all sensors
  }
  float curr_bottlePress = mutiSensor_test.bottle_hPa;
  float desiredPressure = 1400.0f; // desired pressure in hPa

  pump1_control.update(desiredPressure, curr_bottlePress);

  /* Control Valve*/

  if (Serial.available()) {
    char cmd = Serial.read();   

    if (cmd == 'V') {           
      int idx;
      int state;

      
      while(!Serial.available());
      idx = Serial.read() - '0';   // Pump index (1-4) 

      while(!Serial.available());
      state = Serial.read() - '0';  // Valve state (0: off, 1: on)

      // Control the valve
      if (state == 1) abValve1.valveOn(idx);
      else if (state == 0) abValve1.valveOff(idx);

      Serial.print("The valve ");
      Serial.print(idx);
      Serial.print(" is turned ");;
      Serial.println(state == 1 ? "ON" : "OFF");
    }

    if (cmd == 'R'){
      mutiSensor_test.getAllPressure();
      mutiSensor_test.printAllPressure();
    }
  }


};

