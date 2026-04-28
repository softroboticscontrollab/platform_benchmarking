#include "Arduino.h" 
#include "Wire.h"
#include "MutiPressure.h"
#include "PumpController.h"
#include "ABValve.h"
#include "ProportionalValveController.h"

/*
NOTE: 
1. Setting Kp (Ki) manually: 
a. Do Kp first
  - Set idx and corresponding Kp in Kp[4]. Set setManual = true and setpoint.
  - The program will wait waiting_ms, then set the setpoint for holding_ms, then reset to initial pressure for reset_ms, and repeat.
  - Read the teleplot data to see the response. Two variable ">:pressure", ">:target" 
  - stop and re-upload the code to try new Kp.
b. Do Ki next
  - After Kp is set, set Ki in Ki[4]. Everything else is the same as Kp setting.
c. For filtered system, tune Kp and Ki in the setup()

2. Setting Kp (Ki) automatically searching: 
  - Set Kp_var and Kp_step and Kp in Kp[4] and idx. SetAuto = true and setpoint.
  - The program will try each Kp from Kp_min = Kp-Kp_var to Kp_max+Kp_var with step Kp_step and gives a score. After all Kp tried, it will repeat again.
  - The score relates to the settling_time and steady_state_error
  - Read the teleplot data to see the response for each Kp. 3 variables: ">:Kp", ">:pressure", ">:score".
*/

// TODO: setting  automatically searching function (unfinished because I am tired :< )


// Global parameters
float Ts_ms = 100.0f; // controller sampling time in milliseconds
float desiredBottlePressure = 1400.0f; // desired pressure in hPa in bottle
float achievablePressure = 1300.0f; // achievable bottle pressure in hPa
float initialLimbPressure = 1000.0f; // general atmospheric pressure in hPa
float Kp[4] = {30.0f, 32.0f, 34.0f, 32.0f}; // Proportional gain for each valve
float Ki[4] = {12.0f, 12.0f, 14.0f, 12.0f}; // Integral gain for each valve 9
const int BUFFER_FSIZE = 10; // buffer size for low-pass filter
const float sample_interval_ms = 10.0f;  // for filter, 10ms = 100Hz

// **Butterworth Coefficients (2nd order, fs=100Hz, fc=10Hz)**
const float b0_hf = 0.0674;
const float b1_hf = 0.1349;
const float b2_hf = 0.0674;
const float a1_hf = -1.1430;
const float a2_hf = 0.4128;
struct BiquadState {  // Butterworth filter states (2ed order)
    float x1 = 0, x2 = 0;  // input history
    float y1 = 0, y2 = 0;  // output history
};
BiquadState filter_P1, filter_P2, filter_P3, filter_P4;


// Global objects
PressureMux mutiSensor_test;
PumpController pump1_control(5,Ts_ms); // pump connected to pin 8
ABValve abValve1(46, 24, 47, 25); // valves connected to pins 
ProportionalValveController propValve_control(abValve1, desiredBottlePressure, Ts_ms, Kp, Ki); // bottle pressure 2000 hPa

// Global variables and functions
unsigned long previousMillis;
unsigned long currentMillis;
unsigned long lastSampleTime = 0;   // for filter timing
float initialPressure;    // Detect today's atm to set baseline for the target pressures 
float targetPressure [4] = {initialLimbPressure, initialLimbPressure, initialLimbPressure, initialLimbPressure}; 
float previousPressure [4] = {initialLimbPressure, initialLimbPressure, initialLimbPressure, initialLimbPressure};
float P1_buffer[BUFFER_FSIZE] = {0};  // buffers for low-pass filter
float P2_buffer[BUFFER_FSIZE] = {0};
float P3_buffer[BUFFER_FSIZE] = {0};
float P4_buffer[BUFFER_FSIZE] = {0};
int buffer_index = 0;

// Helper functions
void setPumpInitialPressure(float targetPressure_hPa);  // Set the initial bottle pressure in the setup phase
void mappingTragetPressure(); // Map the received target pressures based on atmspheric pressure, use when controlling two limb segments
void serialCommandInterface();  
void computeScore(float currPressure, float& score, unsigned long& settlingTime, bool& isSettled, float setpoint);
float butterworth2Filter(float x, BiquadState &state); // 2ed order Butterworth filter

// **Flags**
bool ischamberPairLimbs = true; // true: input <p1, p2> and <p3, p4> are two limb segments. Will do target presssure mapping based on today's atm. false: input <p1, p2, p3, p4> are four independent limb segments.
bool setManual = true; // true: manual Kp setting; false: automatic Kp searching
bool setAuto = false; // true: automatic Kp searching; false: manual Kp setting
bool isFilter = false; // true: enable low-pass filter. false: disable the filter.

// Settings for PI tunning
unsigned long tunningPIMillis;  // record time for PI tunning
int tunning_idx = 4; // which propotional valve to tune: 1, 2, 3, or 4
float setpoint = 1110.0f; // desired pressure for the selected limb segment
int waiting_ms = 3000; // waiting time before setting the setpoint
int holding_ms = 8000; // holding time at the setpoint 
int reset_ms = 5000; // time to reset to initial pressure
int Kp_var = 3; // Kp variation range for automatic searching
int Kp_step = 1; // Kp step for automatic searching

float Kp_current; // current Kp being tested
float score_current; // current score for the Kp being tested
float Kp_min = Kp[tunning_idx-1] - Kp_var, Kp_max = Kp[tunning_idx-1] + Kp_var; // min and max Kp for automatic searching


void setup(){

  Serial.begin(115200);
  Wire.begin();

  // Initialize all modules
  mutiSensor_test.setup();
  pump1_control.setup();
  abValve1.setup();
  propValve_control.setup();

  // If connected to the 2 segement hardware, collect today's atm pressure 
  if (ischamberPairLimbs)
  {
    mutiSensor_test.getAllPressure();
    initialPressure = 0.25f*(mutiSensor_test.pump1_hPa +
                           mutiSensor_test.pump2_hPa +
                           mutiSensor_test.pump3_hPa +
                           mutiSensor_test.pump4_hPa);
    propValve_control.setNominalZeroPressure(initialPressure + 1.0f); // set nominal zero pressure slightly above atm
    Serial.println("Today's atmospheric pressure detected: " + String(initialPressure) + " hPa");
  }

  // if isFiler, modify the Kp and Ki to compensate for the filter delay
  if (isFilter){ 
    Kp[0] = 17.0f;  
    Kp[1] = 18.0f;
    Kp[2] = 25.0f;
    Kp[3] = 25.0f;
    Ki[0] = 4.5f;
    Ki[1] = 8.0f;
    Ki[2] = 10.0f;
    Ki[3] = 10.0f;
    propValve_control.setGains(Kp, Ki);
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

  // Filter sampling loop
  if (isFilter && (currentMillis - lastSampleTime >= sample_interval_ms)) {
    lastSampleTime = currentMillis;
    // Read all pressures
    mutiSensor_test.getAllPressure(); 
    // Apply Butterworth filter
    P1_buffer[buffer_index] = butterworth2Filter(mutiSensor_test.pump1_hPa, filter_P1);
    P2_buffer[buffer_index] = butterworth2Filter(mutiSensor_test.pump2_hPa, filter_P2);
    P3_buffer[buffer_index] = butterworth2Filter(mutiSensor_test.pump3_hPa, filter_P3);
    P4_buffer[buffer_index] = butterworth2Filter(mutiSensor_test.pump4_hPa, filter_P4);
    buffer_index = (buffer_index + 1) % BUFFER_FSIZE;
  }

  // Main control loop
  if (currentMillis - previousMillis >= Ts_ms) {
    previousMillis = currentMillis;

    //1. Read all pressures and send over Serial
    float currP1 = 0, currP2 = 0, currP3 = 0, currP4 = 0;
    if (isFilter){  // take the newest filtered value
      int latest_index = (buffer_index - 1 + BUFFER_FSIZE) % BUFFER_FSIZE;
      currP1 = P1_buffer[latest_index];
      currP2 = P2_buffer[latest_index];
      currP3 = P3_buffer[latest_index];
      currP4 = P4_buffer[latest_index];
    }
    else {
      mutiSensor_test.getAllPressure(); 
      currP1 = mutiSensor_test.pump1_hPa;
      currP2 = mutiSensor_test.pump2_hPa;
      currP3 = mutiSensor_test.pump3_hPa;
      currP4 = mutiSensor_test.pump4_hPa;
    }

    //2. Update pump controller for bottle pressure
    float curr_bottlePress = mutiSensor_test.bottle_hPa;
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
    
    // PI tunning protocol
    //serialCommandInterface();

    if (setManual) {
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
      if (isFilter) {
        int latest_index = (buffer_index - 1 + BUFFER_FSIZE) % BUFFER_FSIZE;
        if (tunning_idx-1 == 0) displayPressure = P1_buffer[latest_index];
        else if (tunning_idx-1 == 1) displayPressure = P2_buffer[latest_index];
        else if (tunning_idx-1 == 2) displayPressure = P3_buffer[latest_index];
        else displayPressure = P4_buffer[latest_index];
        }
      else {
        mutiSensor_test.getAllPressure();
        float currPressures [4] = {mutiSensor_test.pump1_hPa,
                              mutiSensor_test.pump2_hPa,
                              mutiSensor_test.pump3_hPa,
                              mutiSensor_test.pump4_hPa};
        displayPressure = currPressures[tunning_idx - 1];
      }
      Serial.print(">pressure:");
      Serial.println(displayPressure);
      Serial.print(">target:");
      Serial.println(targetPressure[tunning_idx - 1]);
    }
    

    

};

/*
  Utility functions:
*/

// Serial Command Interface for testing valve and pressure readings
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

    // reset internal integral terms
    if (cmd == 'Z'){
      propValve_control.resetIntegral();
      Serial.println("Integral terms reset.");
    }

    // Change new target pressures for each limb
    // Example: C 1020 1030 1040 1050
    if (cmd == 'C'){
      for (int i = 0; i < 4; i++) {
        float inputPressure = Serial.parseFloat();
        if (inputPressure > achievablePressure){
          Serial.println("Input pressure exceeds achievable pressure. Setting to max achievable pressure.");
          inputPressure = achievablePressure;
        }
        targetPressure[i] = inputPressure;
      }
      Serial.println("New target pressures set.");
    }

    // tunning Kp and Ki gains
    // Example: K 1.0 2.0 3.0 4.0  change Kp gains for each valve
    // Example: I 1.0 2.0 3.0 4.0  change Ki gains for each valve
    if (cmd == 'K'){
      for (int i = 0; i < 4; i++) {
        Kp[i] = Serial.parseFloat();
      }
      propValve_control.setGains(Kp, Ki);
      propValve_control.resetIntegral();
      Serial.println("New Kp gains set.");
      // print the set Kp gains
      Serial.println("Kp Gains:");
      for (int i = 0; i < 4; i++) {
        Serial.print("Valve ");
        Serial.print(i+1);
        Serial.print(": ");
        Serial.println(Kp[i]);
      }
    }
    else if (cmd == 'I'){
      for (int i = 0; i < 4; i++) {
        Ki[i] = Serial.parseFloat();
      }
      propValve_control.setGains(Kp, Ki);
      propValve_control.resetIntegral();
      Serial.println("New Ki gains set.");
      // print the set Ki gains
      Serial.println("Ki Gains:");
      for (int i = 0; i < 4; i++) {
        Serial.print("Valve ");
        Serial.print(i+1);
        Serial.print(": ");
        Serial.println(Ki[i]);  
      }

    
      
    }
    
  }   
}


void setPumpInitialPressure(float targetPressure_hPa) {
  Serial.println("Setting initial bottle pressure...");
  unsigned long start = millis();
  while (1)
  {
    mutiSensor_test.getAllPressure();
    float curr_bottlePress = mutiSensor_test.bottle_hPa;
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

// 2ed order Butterworth filter implementation
float butterworth2Filter(float x, BiquadState &state) {
    float y = b0_hf * x + b1_hf * state.x1 + b2_hf * state.x2
                      - a1_hf * state.y1 - a2_hf * state.y2;
    // Update states
    state.x2 = state.x1;
    state.x1 = x;
    state.y2 = state.y1;
    state.y1 = y;
    return y;
}


