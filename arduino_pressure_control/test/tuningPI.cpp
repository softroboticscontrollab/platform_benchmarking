#include "Arduino.h" 
#include "Wire.h"
#include "MutiPressure.h"
#include "PumpController.h"
#include "ABValve.h"
#include "ProportionalValveController.h"

/**
 * @file tuningPI.cpp
 * @brief PI controller tuning utility for the pneumatic limb hardware.
 *
 * This file is used to tune the proportional valve PI controller by applying
 * repeated step inputs to one selected chamber and visualizing the pressure
 * response over Serial.
 *
 * The tuning protocol cycles through three phases:
 *
 * 1. Waiting phase:
 *    The chamber remains near the initial atmospheric pressure.
 *
 * 2. Holding phase:
 *    The selected chamber target pressure is set to `setpoint`.
 *
 * 3. Reset phase:
 *    The selected chamber target pressure is reset to the initial pressure.
 *
 * The selected chamber is configured by:
 *
 *   tunning_idx
 *
 * where:
 * - 1 tunes chamber 1
 * - 2 tunes chamber 2
 * - 3 tunes chamber 3
 * - 4 tunes chamber 4
 *
 * The PI gains are configured manually in:
 *
 *   Kp[4]
 *   Ki[4]
 *
 * Serial output is formatted for Teleplot-style visualization 
 *  (Teleplot is an Extension for real-time plotting, installed via VSCode Extensions):
 *
 *   >pressure:<measured_pressure>
 *   >target:<target_pressure>
 *
 * Recommended workflow:
 *
 * 1. Select the chamber to tune using `tunning_idx`.
 * 2. Set a target pressure step using `setpoint`.
 * 3. Start with Ki = 0 and tune Kp.
 * 4. Once Kp is acceptable, tune Ki.
 * 5. Re-upload the program after modifying gains.
 *
 * Note:
 * PlatformIO only builds the executable entry point from `src/main.cpp`.
 * To run this tuning utility, copy this file into `src/main.cpp`
 * after saving a backup of the original main controller.
 */

// Global parameters
float Ts_ms = 100.0f; // controller sampling time in milliseconds
float desiredBottlePressure = 1400.0f; // desired pressure in hPa in bottle
float achievablePressure = 1300.0f; // achievable bottle pressure in hPa
float initialLimbPressure = 1000.0f; // general atmospheric pressure in hPa
float Kp[4] = {30.0f, 32.0f, 34.0f, 32.0f}; // Proportional gain for each valve
float Ki[4] = {12.0f, 12.0f, 14.0f, 12.0f}; // Integral gain for each valve 9


// Global objects
PressureMux mutiSensor;
PumpController pump1_control(5,Ts_ms); // pump connected to pin 5
ABValve abValve1(46, 24, 47, 25); // valves connected to pins 
ProportionalValveController propValve_control(abValve1, desiredBottlePressure, Ts_ms, Kp, Ki); // bottle pressure 2000 hPa

// Global variables and functions
unsigned long previousMillis;
unsigned long currentMillis;
unsigned long lastSampleTime = 0;   // for filter timing
float initialPressure = initialLimbPressure;    // Detect today's atm to set baseline for the target pressures 
float targetPressure [4] = {initialLimbPressure, initialLimbPressure, initialLimbPressure, initialLimbPressure}; 
float previousPressure [4] = {initialLimbPressure, initialLimbPressure, initialLimbPressure, initialLimbPressure};

// Helper functions
void setPumpInitialPressure(float targetPressure_hPa);  // Set the initial bottle pressure in the setup phase
void mappingTragetPressure(); // Map the received target pressures based on atmspheric pressure, use when controlling two limb segments

// **Flags**
// Flag indicating whether the four chambers are treated as two opposing chamber pairs.
// When true:
//   - Chambers [1,2] and [3,4] are treated as two bidirectional limb segments.
//   - Target pressures are remapped around the measured atmospheric pressure.
//   - The lower-pressure chamber in each pair is treated as the released side.
//
// When false:
//   - All four chambers are controlled independently using direct target pressures.
bool ischamberPairLimbs = true; 

// Settings for PI tunning
unsigned long tunningPIMillis;  // record time for PI tunning
int tunning_idx = 1; // which propotional valve to tune: 1, 2, 3, or 4
float setpoint = 1050.0f; // desired pressure for the selected limb segment
int waiting_ms = 3000; // waiting time before setting the setpoint
int holding_ms = 8000; // holding time at the setpoint 
int reset_ms = 5000; // time to reset to initial pressure


void setup(){

  Serial.begin(115200);
  Wire.begin();

  // Initialize all modules
  mutiSensor.setup();
  pump1_control.setup();
  abValve1.setup();
  propValve_control.setup();

  // If connected to the 2 segement hardware, collect today's atm pressure 
  if (ischamberPairLimbs)
  {
    mutiSensor.getAllPressure();
    initialPressure = 0.25f*(mutiSensor.pump1_hPa +
                           mutiSensor.pump2_hPa +
                           mutiSensor.pump3_hPa +
                           mutiSensor.pump4_hPa);
    propValve_control.setNominalZeroPressure(initialPressure + 1.0f); // set nominal zero pressure slightly above atm
    Serial.println("Today's atmospheric pressure detected: " + String(initialPressure) + " hPa");
  }


  // Set initial pressure
  setPumpInitialPressure(desiredBottlePressure);

  // record time and delay before starting main loop
  previousMillis = millis();
  tunningPIMillis = millis();
  delay(1000);  
 
}

void loop(){
  currentMillis = millis();
  
  // Main control loop
  if (currentMillis - previousMillis >= Ts_ms) {
    previousMillis = currentMillis;

    //1. Read all pressures and send over Serial
    float currP1 = 0, currP2 = 0, currP3 = 0, currP4 = 0;
    mutiSensor.getAllPressure(); 
    currP1 = mutiSensor.pump1_hPa;
    currP2 = mutiSensor.pump2_hPa;
    currP3 = mutiSensor.pump3_hPa;
    currP4 = mutiSensor.pump4_hPa;

    //2. Update pump controller for bottle pressure
    float curr_bottlePress = mutiSensor.bottle_hPa;
    pump1_control.update(desiredBottlePressure, curr_bottlePress);

    //3. Calculate target pressures for four limbs and update proportional valve controller
    if (ischamberPairLimbs){
      // if controlling two limb segments only, map targetPressure based on atm
      mappingTragetPressure();
    }
    propValve_control.update(targetPressure[0], currP1,
                             targetPressure[1], currP2,
                             targetPressure[2], currP3,
                             targetPressure[3], currP4);  

    }
    
    // 4. **PI tunning protocol**
    static uint8_t lastPhase = 0;  // 0=waiting, 1=holding, 2=reset
    uint8_t currentPhase;

    unsigned long elapsed = millis() - tunningPIMillis;
    if (elapsed < waiting_ms) currentPhase = 0;
    else if (elapsed < waiting_ms + holding_ms) currentPhase = 1;
    else if (elapsed < waiting_ms + holding_ms + reset_ms) currentPhase = 2;
    else {
      tunningPIMillis = millis();
      currentPhase = 0;
    }

    // only act when phase changes
    if (currentPhase != lastPhase) {
      if (currentPhase == 1) {  // holding now
        targetPressure[tunning_idx - 1] = setpoint;
        }
      else if (currentPhase == 2) {  // reset now
        targetPressure[tunning_idx - 1] = initialPressure;
        }
      propValve_control.resetIntegral();  // reset integral term whenever setpoint changes
      lastPhase = currentPhase;
      }

    // Send teleplot data
    float displayPressure;
    mutiSensor.getAllPressure();
    float currPressures [4] = {mutiSensor.pump1_hPa,
                          mutiSensor.pump2_hPa,
                          mutiSensor.pump3_hPa,
                          mutiSensor.pump4_hPa};
    displayPressure = currPressures[tunning_idx - 1];
    Serial.print(">pressure:");
    Serial.println(displayPressure);
    Serial.print(">target:");
    Serial.println(targetPressure[tunning_idx - 1]);
};

/*
  Utility functions:
*/

void setPumpInitialPressure(float targetPressure_hPa) {
  Serial.println("Setting initial bottle pressure...");
  unsigned long start = millis();
  while (1)
  {
    mutiSensor.getAllPressure();
    float curr_bottlePress = mutiSensor.bottle_hPa;
    pump1_control.update(targetPressure_hPa, curr_bottlePress);
    if (abs(curr_bottlePress - targetPressure_hPa) < 20.0f) {
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
}

// The received pressure from python are 2 pairs <p1, p2> and <p3, p4> 
// For each pair: take the smaller one as today's atm, and the other one +abs(p_a-p_b) to maintain the proper pressure difference
void mappingTragetPressure() {
  float diff12 = abs(targetPressure[0] - targetPressure[1]);
  float diff34 = abs(targetPressure[2] - targetPressure[3]);
  // First pair
  if (targetPressure[0] <= targetPressure[1]) {
    targetPressure[0] = initialPressure;  // will be regared as 0 in propValve_control.update() and fully released
    targetPressure[1] = initialPressure + diff12;   // maintain the pressure difference
  }
  else {
    targetPressure[1] = initialPressure;
    targetPressure[0] = initialPressure + diff12;
  }

  // Second pair
  if (targetPressure[2] <= targetPressure[3]) {
    targetPressure[2] = initialPressure;
    targetPressure[3] = initialPressure + diff34;
  }
  else {
    targetPressure[3] = initialPressure;
    targetPressure[2] = initialPressure + diff34;
  }
}

