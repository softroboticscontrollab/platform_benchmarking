#include "Arduino.h" 
#include "Wire.h"
#include "MutiPressure.h"
#include "PumpController.h"
#include "ABValve.h"
#include "ProportionalValveController.h"
#include "FilterBank.h"

/*
Note:
1. SAFETY WARNING: the pipe should be disconnected to the robot to prevent explosion.
2. Purpose: The code is to test each sensor and find the cutting off frequency.
3. Intro:
    - Select one channel, the code will do: fully inflate -> fully deflate, and repeat.
    - The sensor reading will be sent through serial port for teleplot for observing the noise.
    - Manully adjust the Fc in **Global Parameter** section to find the best one.
4. Arguments:
    - isFilter: false: just output the raw sensor reading; true: output the filtered reading.
    - channelToTest: which channel to test: 1, 2, 3, or 4
    - HoldingTime_ms: time to hold for inflation/deflation in milliseconds
5. Output:
    - use datatx() to send two types of data:
        - [FIL] <p> /n
        - [RAW] <p> /n
*/

// **Global Parameters Define**
const float Ts_ms = 100.0f; // controller sampling time in milliseconds
const float sample_interval_ms = 10.0f;  // for filter, 10ms = 100Hz
const float sampling_freq_Hz = 1000.0f / sample_interval_ms;  // sampling frequency in Hz
const float cutoff_freq_Hz = 3.0f;  // cutoff frequency in Hz for Butterworth filter
float desiredBottlePressure = 1250.0f; // desired pressure in hPa in bottle
float initialLimbPressure = 1010.0f; // dummy initial pressure for each limb segment in hPa
/*THE KP AND KI WILL NOT BE USED. JUST FOR COMPATIBILITY*/
float Kp[4] = {21.0f, 38.0f, 33.0f, 32.0f}; // Proportional gain for each valve
float Ki[4] = {6.0f, 11.0f, 9.0f, 10.0f}; // Integral gain for each valve
String sprefix1 = "FIL"; // for pressure sensor in chambers
String sprefix2 = "RAW"; // for raw pressure sensor in chambers

// **Global Objects Define**
PressureMux mutiSensor_test;
PumpController pump1_control(8,Ts_ms); // pump connected to pin 8
ABValve abValve1(9, 10, 11, 12); // valves connected to pins 
ProportionalValveController propValve_control(abValve1, desiredBottlePressure, Ts_ms, Kp, Ki);
FilterBank filters[4] = {
    FilterBank(sampling_freq_Hz, cutoff_freq_Hz),  // P1
    FilterBank(sampling_freq_Hz, cutoff_freq_Hz),  // P2
    FilterBank(sampling_freq_Hz, cutoff_freq_Hz),  // P3
    FilterBank(sampling_freq_Hz, cutoff_freq_Hz)   // P4
}; 

// **Global variables**
unsigned long previousMillis;
unsigned long currentMillis;
unsigned long lastSampleTime = 0;   // for filter timing
float filteredPressure [4] = {initialLimbPressure, initialLimbPressure, initialLimbPressure, initialLimbPressure};
float rawPressure [4] = {initialLimbPressure, initialLimbPressure, initialLimbPressure, initialLimbPressure};

// **Helper functions**
void setPumpInitialPressure(float targetPressure_hPa);  // Set the initial bottle pressure in the setup phase
void datatx(String data, String prefix);  

// For Fc tuning
int channelToTest = 4; // which channel to test: 1, 2, 3, or 4
const float HoldingTime_ms = 5000.0f; // time to hold for inflation/deflation in milliseconds

void setup(){

  Serial.begin(115200);
  Wire.begin();

  // (1). Initialize all hardware modules
  mutiSensor_test.setup();
  pump1_control.setup();
  abValve1.setup();
  propValve_control.setup();

  // (2). Set initial pressure
  setPumpInitialPressure(desiredBottlePressure);

  // (3). Record time and delay before starting main loop
  previousMillis = millis();
  delay(1000);  
}

void loop(){
    currentMillis = millis();
    
    // Loop 1: the pump control loop (10Hz) to make the bottle pressure stable for infalation
    if (currentMillis - previousMillis >= Ts_ms) {
        previousMillis = currentMillis;
        mutiSensor_test.getAllPressure();
        float curr_bottlePress = mutiSensor_test.bottle_hPa;
        pump1_control.update(desiredBottlePressure, curr_bottlePress);
    }

    // Loop 2: state machine for valve control (every HoldinigTime_ms)
    static unsigned long lastStateChangeMillis = 0;
    static bool isInflating = true; // start with deflating
    if (currentMillis - lastStateChangeMillis >= HoldingTime_ms) {
        lastStateChangeMillis = currentMillis;
        if (isInflating) {
            // Switch to deflating
            isInflating = false;
            propValve_control.setChannelFullyInflate(channelToTest);
            }
        else {
            // Switch to inflating
            isInflating = true;
            propValve_control.setChannelFullyDeflate(channelToTest);
        } 
      }
    

    // Loop 3: the sensor reading loop (100Hz) and output
    if (currentMillis - lastSampleTime >= sample_interval_ms) {
        lastSampleTime = currentMillis;
        mutiSensor_test.getAllPressure();
        
        switch (channelToTest)
        {
        case 1:
            filteredPressure[0] = filters[0].filter1st(mutiSensor_test.pump1_hPa);
            rawPressure[0] = mutiSensor_test.pump1_hPa;
            datatx(String(filteredPressure[0]), sprefix1);
            datatx(String(rawPressure[0]), sprefix2);
            break;
        case 2:
            filteredPressure[1] = filters[1].filter1st(mutiSensor_test.pump2_hPa);
            rawPressure[1] = mutiSensor_test.pump2_hPa;
            datatx(String(filteredPressure[1]), sprefix1);
            datatx(String(rawPressure[1]), sprefix2);
            break;
        case 3:
            filteredPressure[2] = filters[2].filter1st(mutiSensor_test.pump3_hPa);
            rawPressure[2] = mutiSensor_test.pump3_hPa;
            datatx(String(filteredPressure[2]), sprefix1);
            datatx(String(rawPressure[2]), sprefix2);
            break;
        case 4:
            filteredPressure[3] = filters[3].filter1st(mutiSensor_test.pump4_hPa);
            rawPressure[3] = mutiSensor_test.pump4_hPa;
            datatx(String(filteredPressure[3]), sprefix1);
            datatx(String(rawPressure[3]), sprefix2);
            break;
        default:
            break;
        }
        
     
    }

    // Loop 4: for debug purpose only
    // print DAC outputs
    // switch (channelToTest)
    // {
    // case 1:
    //   Serial.print(">DAC1: ");
    //   Serial.println(propValve_control.dacOutputs1);
    //   break;
    // case 2:
    //   Serial.print(">DAC2: ");
    //   Serial.println(propValve_control.dacOutputs2);
    //   break;
    // case 3:
    //   Serial.print(">DAC3: ");
    //   Serial.println(propValve_control.dacOutputs3);
    //   break;
    // case 4:
    //   Serial.print(">DAC4: ");
    //   Serial.println(propValve_control.dacOutputs4);
    //   break;
    // default:
    //   break;
    // }
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

// Send data over Serial in the format [prefix]: <f> <f> <f> ... 
// Separate data with spaces
void datatx(String data, String prefix){
  Serial.println("[" + prefix + "]: " + data);
}