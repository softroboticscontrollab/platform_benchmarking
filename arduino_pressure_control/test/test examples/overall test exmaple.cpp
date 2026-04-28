#include "Arduino.h" 
#include "MutiPressure.h"
#include "PumpController.h"
#include "ABValve.h"
#include "ProportionalValveController.h"

// TODO: in the setup(), make the bottle achieve the target pressure first
// TODO: in proportional class, examine if the target pressure is beyond the bottle pressure. If so, warn and set to max.
// TODO: make sure we receive the same data input from the python. Think about using Juan's Parse Function.


// Global parameters
float Ts_ms = 100.0f; // controller sampling time in milliseconds
float desiredPressure = 1400.0f; // desired pressure in hPa in bottle
float achievablePressure = 1300.0f; // achievable bottle pressure in hPa
float ATM_PRESSURE_HPA = 1014.25f; // atmospheric pressure in hPa
float diffP12 = 0.0f; // desired pressure difference between limb 1 and limb 2 in hPa
float diffP34 = 0.0f; // desired pressure difference between limb 3 and limb 4 in hPa


// Global objects
PressureMux mutiSensor_test;
PumpController pump1_control(8,Ts_ms); // pump connected to pin 8
ABValve abValve1(9, 10, 11, 12); // valves connected to pins 
ProportionalValveController propValve_control(abValve1, desiredPressure, Ts_ms); // bottle pressure 2000 hPa

// Global variables and functions
unsigned long previousMillis;
unsigned long currentMillis;
float targetPressure [4];
void calculateTargetPressures(float currP1, float currP2, float currP3, float currP4, float diffP12, float diffP34);
void serialCommandInterface();  

void setup(){

  Serial.begin(9600);
  Wire.begin();

  // Initialize all modules
  mutiSensor_test.setup();
  pump1_control.setup();
  abValve1.setup();
  propValve_control.setup();

  // Set the bottle pressure to an initial value
  Serial.println();
  Serial.println("Setting initial bottle pressure...");
  unsigned long start = millis();
  while (1)
  {
    mutiSensor_test.getAllPressure();
    float curr_bottlePress = mutiSensor_test.bottle_hPa;
    pump1_control.update(desiredPressure, curr_bottlePress);
    if (abs(curr_bottlePress - desiredPressure) < 20.0f) {
      Serial.println("Initial bottle pressure achieved.");
      break;
    }
    if (millis() - start > 10000) { 
        Serial.println("Bottle pressurization timeout — proceeding anyway.");
        break;
    }
    delay(100);
  }
  Serial.println("Setup complete.");

  // record time and delay before starting main loop
  previousMillis = millis();
  delay(1000);  
 
};

void loop(){
  currentMillis = millis();
  if (currentMillis - previousMillis >= Ts_ms) {
    previousMillis = currentMillis;

    // Read all pressures
    mutiSensor_test.getAllPressure(); 

    // Update pump controller for bottle pressure
    float curr_bottlePress = mutiSensor_test.bottle_hPa;
    pump1_control.update(desiredPressure, curr_bottlePress);

    // Calculate desired pressures for four limbs and update proportional valve controller
    float currP1 = mutiSensor_test.pump1_hPa;
    float currP2 = mutiSensor_test.pump2_hPa;
    float currP3 = mutiSensor_test.pump3_hPa;
    float currP4 = mutiSensor_test.pump4_hPa;
    calculateTargetPressures(currP1, currP2, currP3, currP4, diffP12, diffP34);
    propValve_control.update(targetPressure[0], currP1,
                             targetPressure[1], currP2,
                             targetPressure[2], currP3,
                             targetPressure[3], currP4);  
    
  
    }
    serialCommandInterface();
};

/*
  Utility functions:
*/

// Serial Command Interface for testing valve and pressure readings
// Commands:
// - 'R' - Read and print all pressures
// - 'V__' - Valve control command, where __ are two digits:
//         First digit: valve index (1-4)
//         Second digit: valve state (0: off, 1: on)
//    Example: 'V11' turns valve 1 ON
//            'V20' turns valve 2 OFF
void serialCommandInterface() {
  if (Serial.available()) {
    char cmd = Serial.read();   

    // read and print all pressures
    if (cmd == 'R'){
      mutiSensor_test.getAllPressure();
      mutiSensor_test.printAllPressure();
    }

    // compare the target pressure and the sensor reading
    if (cmd == 'D'){
      Serial.println("Comparing target pressures and current pressures:");
      Serial.println("==============================");
      Serial.println("Target Pressures vs. Current Pressures:");
      Serial.print("Limb 1: ");
      Serial.print(targetPressure[0]);
      Serial.print(" hPa vs. ");
      Serial.print(mutiSensor_test.pump1_hPa);
      Serial.println(" hPa");

      Serial.print("Limb 2: ");
      Serial.print(targetPressure[1]);
      Serial.print(" hPa vs. ");
      Serial.print(mutiSensor_test.pump2_hPa);
      Serial.println(" hPa");

      Serial.print("Limb 3: ");
      Serial.print(targetPressure[2]);
      Serial.print(" hPa vs. ");
      Serial.print(mutiSensor_test.pump3_hPa);
      Serial.println(" hPa");

      Serial.print("Limb 4: ");
      Serial.print(targetPressure[3]);
      Serial.print(" hPa vs. ");
      Serial.print(mutiSensor_test.pump4_hPa);
      Serial.println(" hPa");

      Serial.println("==============================");
    }

    // get the DAC outputs
    if (cmd == 'P'){
      Serial.println("Proportional Valve DAC Outputs:");
      Serial.println("==============================");
      Serial.print("Valve 1 DAC Output: ");
      Serial.println(propValve_control.dacOutputs1);
      Serial.print("Valve 2 DAC Output: ");
      Serial.println(propValve_control.dacOutputs2);
      Serial.print("Valve 3 DAC Output: ");
      Serial.println(propValve_control.dacOutputs3);
      Serial.print("Valve 4 DAC Output: ");
      Serial.println(propValve_control.dacOutputs4);  
      Serial.println("==============================");
    }

    // Change the diffP12 and diffP34 values, command format: 'Cxxxyyy', where xxx is diffP12, yyy is diffP34
    // Example: 'C060-30' sets diffP12 = 60, diffP34 = -30
    if (cmd == 'C'){
      String paramStr = Serial.readStringUntil('\n'); 
      if (paramStr.length() >= 6) {
        String diffP12Str = paramStr.substring(0, 3);
        String diffP34Str = paramStr.substring(3, 6);
        diffP12 = diffP12Str.toFloat();
        diffP34 = diffP34Str.toFloat();
        Serial.print("Updated diffP12 to ");
        Serial.print(diffP12);
        Serial.print(", diffP34 to ");
        Serial.println(diffP34);
      } else {
        Serial.println("Invalid parameters for command C. Format: Cxxxyyy");
      }
    }

    // Reset everything: resetIntegral for all PI controllers, set the diffP12 and diffP34 to zero
    if (cmd == 'E'){
      propValve_control.resetIntegral();
      pump1_control.resetIntegral();
      diffP12 = 0.0f;
      diffP34 = 0.0f;
      Serial.println("Reset all PI integrals and set diffP12 and diffP34 to zero.");
    }





  }   
}

void calculateTargetPressures(float currP1, float currP2, float currP3, float currP4, float diffP12, float diffP34) 
{
  float targetP1, targetP2, targetP3, targetP4;

  // For limbs 1 and 2
  if (diffP12 >= 0) {   // pressure of limb 1 > pressure of limb 2, set targetP2 = ATM
      targetP2 = ATM_PRESSURE_HPA;
      targetP1 = ATM_PRESSURE_HPA + abs(diffP12);
  } else {  // pressure of limb 2 > pressure of limb 1, set targetP1 = ATM
      targetP1 = ATM_PRESSURE_HPA;
      targetP2 = ATM_PRESSURE_HPA + abs(diffP12);
  }
  // For limbs 3 and 4
  if (diffP34 >= 0) { // pressure of limb 3 > pressure of limb 4, set targetP4 = ATM
      targetP4 = ATM_PRESSURE_HPA;
      targetP3 = ATM_PRESSURE_HPA + abs(diffP34);
  } else {  // pressure of limb 4 > pressure of limb 3, set targetP3 = ATM
      targetP3 = ATM_PRESSURE_HPA;
      targetP4 = ATM_PRESSURE_HPA + abs(diffP34);
  }
  targetPressure[0] = targetP1;
  targetPressure[1] = targetP2;
  targetPressure[2] = targetP3;
  targetPressure[3] = targetP4; 
};

