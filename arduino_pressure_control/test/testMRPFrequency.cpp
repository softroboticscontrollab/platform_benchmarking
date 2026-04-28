// This code will detect the maximum reading frequency of the MRP pressure sensors
// Read the sensor by getAllPressure() for 1s and output and count. Then rest for 1s. Repeat.

// Result:
// - Single sensor reading (by using getPressure()) frequency: ~153Hz
// - All 5 sensors reading (by using getAllPressure()) frequency: ~32Hz, becasue the reading is sequential.

// Solution:
// - Revise getAllPressure() as parallel reading by triggering all sensors first, then reading all results.
// - The frequency is improved to 71Hz for all 5 sensors.


#include "Arduino.h" 
#include "Wire.h"
#include "MutiPressure.h"

PressureMux mutiSensor_test;

unsigned long previousMillis = 0;
const long interval = 1000; // 1 second interval for counting
int readCount = 0;
bool readingPhase = true; // true: reading phase, false: resting phase

void setup() {
  Serial.begin(115200);
  Wire.begin();

  mutiSensor_test.setup();
  Serial.println("Starting MPR Frequency Test...");
}

void loop() {
  unsigned long currentMillis = millis();

  if (readingPhase) {
    // Reading phase
    // float pressure = mutiSensor_test.getPressure(1); // Read pump 1
    // float pressure_2 = mutiSensor_test.getPressure(2); // Read pump 2
    mutiSensor_test.getAllPressure(); // Read all pumps
    // Serial.print(readCount);
    // Serial.print(": ");
    // Serial.println(mutiSensor_test.bottle_hPa); // Print the pressure value
    readCount++;

    if (currentMillis - previousMillis >= interval) {
      // End of reading phase
      Serial.print("Read Count in 1s interval: ");
      Serial.println(readCount);
      readCount = 0;
      previousMillis = currentMillis;
      readingPhase = false; // Switch to resting phase
    }
  } 
  else {
    // Resting phase
    if (currentMillis - previousMillis >= interval) {
      // End of resting phase
      previousMillis = currentMillis;
      readingPhase = true; // Switch to reading phase
    }
  }
}