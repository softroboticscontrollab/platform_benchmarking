#include "Arduino.h" 
#include "MutiPressure.h"
#include "PumpController.h"


PressureMux mutiSensor_test;
PumpController pump1_control(8); // pump connected to pin 8
const unsigned long Ts_mux_ms = 50; // sensor sampling period
unsigned long last_mux_ms = 0;
bool achieve = 0; // whether desired pressure is achieved


void setup(){
  mutiSensor_test.setup();
  pump1_control.setup();
};

void loop(){
  //mutiSensor_test.loop(1000); // read and print every 1 second
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

