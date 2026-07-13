#include "Arduino.h" 
#include "Wire.h"
#include "MutiPressure.h"
#include "PumpController.h"
#include "ABValve.h"
#include "ProportionalValveController.h"

// USE THIS FILE WITHOUT STIFFENING, IF DOING STIFFENING, THEN COPY STIFF.cpp TO HERE
// FILE FOUND IN >test
// MAKE SURE TO CANGE THIS FILES NAME TO MAIN.CPP FOR CODE TO RUN

/**
 * @file main.cpp
 * @brief Main hardware control program for a 2-segment bidirectional pneumatic limb.
 *
 * This program runs on the microcontroller using the Arduino framework through
 * PlatformIO. It controls the pneumatic hardware by:
 *
 * 1. Reading pressure values from multiple pressure sensors.
 * 2. Maintaining the reservoir/bottle pressure using a pump controller.
 * 3. Receiving target chamber pressures from the host computer through Serial.
 * 4. Mapping target pressures for paired bidirectional limb chambers.
 * 5. Updating proportional valve commands using PI control.
 * 6. Streaming measured pressures and valve aperture values back over Serial.
 *
 * Serial input format:
 *
 *   <p1,p2,p3,p4>
 *
 * where p1-p4 are target pressures in hPa. The message must:
 * - start with '<'
 * - end with '>'
 * - use commas as separators
 * - contain no spaces
 *
 * Example:
 *
 *   <1010,1100,1010,1080>
 *
 * Serial output format:
 *
 *   [MPR]: p1 p2 p3 p4
 *   [RPR]: p_reservoir
 *   [VAL]: v1 v2 v3 v4
 *
 * where:
 * - MPR contains measured chamber pressures in hPa
 * - RPR contains measured reservoir pressure in hPa
 * - VAL contains normalized valve aperture commands in the range [0, 1]
 *
 * Notes:
 * - This file is intended as the main runtime controller.
 * - PI controller tuning utilities are provided separately under test/.
 * - PlatformIO expects the executable entry point to be named src/main.cpp.
 */


// **Global Parameters Define**
const float Ts_ms = 50.0f; // controller sampling time in milliseconds
float desiredBottlePressure = 1400.0f; // desired pressure in hPa in bottle (reservoir).
float achievablePressure = 1250.0f; // achievable chamber pressure in hPa in the limbs, to avoid target pressure beyond the capability of the system. This is used for input validation.
float initialLimbPressure = 1010.0f; // dummy initial pressure for each limb segment in hPa
String sprefix1 = "MPR"; // for pressure sensor in chambers
String sprefix2 = "RPR"; // for pressure sensor in reservoir
String sprefix3 = "VAL"; // for valve apertures
const int BUFSIZE = 200;  // buffer size for serial communication
float Kp[4] = {30.0f, 31.0f, 32.0f, 31.0f}; // Proportional gain for each valve
float Ki[4] = {12.0f, 12.0f, 14.0f, 12.0f}; // Integral gain for each valve 
float val_apertures[4] = {0.0f,0.0f,0.0f,0.0f}; // For collection of valve aperture values

// **Global Objects Define**
PressureMux mutiSensor; 
PumpController pump1_control(5,Ts_ms); // pump connected to pin 5
ABValve abValve1(46, 24, 47, 25); // valves connected to pins 
ProportionalValveController propValve_control(abValve1, desiredBottlePressure, Ts_ms, Kp, Ki);


// **Global variables**
unsigned long previousMillis;
unsigned long currentMillis;
float initialPressure;    // Detect today's atm to set baseline for the target pressures 
float targetPressure [4] = {initialLimbPressure, initialLimbPressure, initialLimbPressure, initialLimbPressure}; 
float previousPressure [4] = {initialLimbPressure, initialLimbPressure, initialLimbPressure, initialLimbPressure};
char receivedChars[BUFSIZE];
char tempChars[BUFSIZE]; // temporary array for use when parsing

// **Helper functions**
void setPumpInitialPressure(float targetPressure_hPa);  // Set the initial bottle pressure in the setup phase
void mappingTragetPressure(); // Map the received target pressures based on atmspheric pressure, use when controlling two limb segments
void datatx(String data, String prefix);  
bool recvWithStartEndMarkers();
void parseData();

// **Flags**

// Flag indicating whether chambers form bidirectional limb pairs.
// In paired mode (true):
//   - Each limb consists of two opposing chambers
//   - One chamber inflates while the other deflates
//   - Pressure commands are interpreted as differential offsets from atmospheric pressure
//
// In unpaired mode (false):
//   - All chambers are controlled independently with direct pressure targets
bool isPairLimbs = true; 

void setup(){

  Serial.begin(115200);
  Wire.begin();

  // (1). Initialize all hardware modules
  mutiSensor.setup();
  pump1_control.setup();
  abValve1.setup();
  propValve_control.setup();

  // (2). Flag related setups
  // (2.1) If connected to the 2 segement hardware, collect today's atm pressure 
  if (isPairLimbs) {
    Serial.println("Warming up pressure sensors...");
    // read 10 times to let sensors stabilize
    for (int i = 0; i < 10; i++) {
        mutiSensor.getAllPressure();
        delay(50);  // 50ms delay between readings
    }
    // take the last average as today's atmospheric pressure
    initialPressure = 0.25f * (mutiSensor.pump1_hPa +
                               mutiSensor.pump2_hPa +
                               mutiSensor.pump3_hPa +
                               mutiSensor.pump4_hPa);
    propValve_control.setNominalZeroPressure(initialPressure + 1.0f);
    Serial.println("Today's atmospheric pressure detected: " + String(initialPressure) + " hPa");
  }

  // (3). Set initial pressure
  setPumpInitialPressure(desiredBottlePressure);

  // (4). Record time and delay before starting main loop
  previousMillis = millis();
  delay(1000);  
 
}

void loop(){
  currentMillis = millis();

  // Loop 1: Controller loop
  if (currentMillis - previousMillis >= Ts_ms) {
    previousMillis = currentMillis;

    // (1). Read all pressures and send over Serial
    float currP1 = 0, currP2 = 0, currP3 = 0, currP4 = 0, currP5 = 0;
    mutiSensor.getAllPressure(); 
    currP1 = mutiSensor.pump1_hPa;
    currP2 = mutiSensor.pump2_hPa;
    currP3 = mutiSensor.pump3_hPa;
    currP4 = mutiSensor.pump4_hPa;
    currP5 = mutiSensor.bottle_hPa;
    datatx(String(currP1) + " " +
            String(currP2) + " " +
            String(currP3) + " " +
            String(currP4), sprefix1);
    
    datatx(String(currP5), sprefix2);        

    // (2). Update pump controller for bottle pressure
    mutiSensor.getAllPressure();
    float curr_bottlePress = mutiSensor.bottle_hPa;
    pump1_control.update(desiredBottlePressure, curr_bottlePress);

    // (3). Calculate target pressures for four limbs and update proportional valve controller
    if (isPairLimbs){
      // if controlling two limb segments only, map targetPressure based on atm
      mappingTragetPressure();
    }
    propValve_control.update(targetPressure[0], currP1,
                             targetPressure[1], currP2,
                             targetPressure[2], currP3,
                             targetPressure[3], currP4);  
    val_apertures[0] = (float)propValve_control.dacOutputs1/4095.0f;
    val_apertures[1] = (float)propValve_control.dacOutputs2/4095.0f;
    val_apertures[2] = (float)propValve_control.dacOutputs3/4095.0f;
    val_apertures[3] = (float)propValve_control.dacOutputs4/4095.0f;    

    datatx(String(val_apertures[0]) + " " +
          String(val_apertures[1]) + " " +
          String(val_apertures[2]) + " " +
          String(val_apertures[3]), sprefix3);    
    }

    // Loop 2: Read the Serial input for new target pressures
    if (recvWithStartEndMarkers()) {
    strcpy(tempChars, receivedChars); // temporary copy because strtok() used in parseData() replaces the commas with \0
    parseData();
    }
}


// Utility functions

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

// Send data over Serial in the format [prefix]: <f> <f> <f> ... 
// Separate data with spaces
void datatx(String data, String prefix){
  Serial.println("[" + prefix + "]: " + data);
}

// Check if the Serial input has new target pressures that start with '<' and end with '>'
bool recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;

    while (Serial.available() > 0) {

        rc = Serial.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
              if (ndx > BUFSIZE - 2) {
                Serial.print("*******************WARNING**********************");
                Serial.println("");
                Serial.print("Number of characters in input exceed capacity!!!");
                ndx = 0; 
                recvInProgress = false; 
              }
              else{
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= BUFSIZE) {
                    ndx = BUFSIZE - 1;
                }
              }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                return 1;
            }
        }

        else if (rc == startMarker) {
            // ndx = 0; 
            recvInProgress = true;
        }
    }
    // returns without running if there is no rx data waiting over serial.
    return 0;
}

// parse the received string into target pressures for four limbs
// the processed data should be in the format: p1,p2,p3,p4
void parseData() {      

    char * strtokIndx; // this is used by strtok() as an index
    static int i = 0;
    bool isWrong = 0;

    for (i = 0; i < 4; i++) {
      if (i == 0){
        strtokIndx = strtok(tempChars, ",");
        targetPressure[i] = atof(strtokIndx);     // convert this part to a float
        if (targetPressure[i] > achievablePressure) { // if the new pressure is beyond achievable, the data we get is wrong
          isWrong = 1;
          break;
        }
      }
      else {
        strtokIndx = strtok(NULL, ",");
        targetPressure[i] = atof(strtokIndx);     // convert this part to a float
        if (targetPressure[i] > achievablePressure) {
          isWrong = 1;
          break;}

      }
    }

    if (isWrong){
      // Take the previous target pressures instead
      targetPressure[0] = previousPressure[0];
      targetPressure[1] = previousPressure[1];
      targetPressure[2] = previousPressure[2];
      targetPressure[3] = previousPressure[3];
    }
    else {
      // Update previousPressure
      previousPressure[0] = targetPressure[0];
      previousPressure[1] = targetPressure[1];
      previousPressure[2] = targetPressure[2];
      previousPressure[3] = targetPressure[3];
    }
}
