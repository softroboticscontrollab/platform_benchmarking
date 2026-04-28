#include "Arduino.h" 
#include "MutiPressure.h"
#include "PumpController.h"
#include "ABValve.h"


PressureMux mutiSensor_test;
PumpController pump1_control(8); // pump connected to pin 8
ABValve abValve1(9, 10, 11, 12); // valves connected to pins 

const unsigned long Ts_mux_ms = 50; // sensor sampling period
unsigned long last_mux_ms = 0;
bool achieve = 0; // whether desired pressure is achieved


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

