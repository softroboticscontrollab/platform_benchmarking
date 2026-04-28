#include "MutiPressure.h"

PressureMux::PressureMux(uint8_t tcaAddr, uint8_t bottle_CH, 
    uint8_t pump1_CH, uint8_t pump2_CH, uint8_t pump3_CH, uint8_t pump4_CH)
    : _tcaAddr(tcaAddr), pumpChanList{bottle_CH, pump1_CH, pump2_CH, pump3_CH, pump4_CH},
    _mpr(-1, -1) {
}

void PressureMux::setup()
{
    Serial.println();
    Serial.println("=== MPRLS x5 via TCA9548A ===");
    
    checkSetup();

    Serial.println("Sensor Setup done.");
}

void PressureMux::loop(int delayMs)
{
    if (isSensorWorking)
    {
        getAllPressure();

        printAllPressure();

        delay(delayMs);
    }
    else
    {
        checkSetup();
    }
}


void PressureMux::checkSetup(){
    // Check for TCA:
    Serial.println("\nTCAScanner Ready!");
    checkTCA();

    // Check each channel:
    Serial.println("\nChannel Pressure Sensor Scanner Ready!");
    for (uint8_t i = 0; i < 5; i++)
    {
        uint8_t currentCH = pumpChanList[i];
        checkSensorOnChannel(currentCH);

        if (!isSensorWorking)
        {
            Serial.println("Channel" + String(currentCH) + "sensor check failed! Exit and check wiring.");
            while (true)
            {
                // Pause here forever
            }
            
        }
    }

}

void PressureMux::getAllPressure(){
    /*
    
    // sequential reading (slow)
    for (int i = 0; i < 10; i++)
    {
        pressureList[i] = 0;
    }

    // Read pressure one-by-one
    for (int i = 0; i < 5; i++)
    {
        uint8_t currentCH = pumpChanList[i];
        pressureList[2*i] = getPressure(currentCH);
        pressureList[2*i+1] = pressureList[2*i] * hPa_to_PSI;        
    }
    
    // Assign value to the variables:
    bottle_hPa = pressureList[0]; bottle_PSI = pressureList[1];
    pump1_hPa = pressureList[2]; pump1_PSI = pressureList[3]; 
    pump2_hPa = pressureList[4]; pump2_PSI = pressureList[5];
    pump3_hPa = pressureList[6]; pump3_PSI = pressureList[7];
    pump4_hPa = pressureList[8]; pump4_PSI = pressureList[9];
    */

    // parallel reading (fast)
    // ===== stage 1: Trigger all sensors to start conversion =====
    for (int i = 0; i < 5; i++) {
        tcaSelect(pumpChanList[i]);
        
        // use Wire to send conversion command
        Wire.beginTransmission(0x18);  // MPRLS default I2C address
        Wire.write(0xAA);
        Wire.write(0x00);
        Wire.write(0x00);
        Wire.endTransmission();
        
    }

    // ===== stage 2: Wait for conversion to complete =====
    delay(6);  // wait 6ms, all sensors are converting simultaneously

    // ===== stage 3: Read all results =====
    for (int i = 0; i < 5; i++) {
        uint8_t currentCH = pumpChanList[i];
        tcaSelect(currentCH);
        
        // read 4 bytes from MPRLS: status + 3 data bytes
        Wire.requestFrom(0x18, 4);
        uint8_t status = Wire.read();
        
        // assemble raw 24-bit pressure value
        uint32_t raw = 0;
        raw |= ((uint32_t)Wire.read()) << 16;
        raw |= ((uint32_t)Wire.read()) << 8;
        raw |= ((uint32_t)Wire.read());
        
        // convert raw value to pressure in hPa and PSI
        const uint32_t OUTPUT_MIN = 1677722;  // 10% of 2^24
        const uint32_t OUTPUT_MAX = 15099494; // 90% of 2^24
        const float PSI_MIN = 0.0;
        const float PSI_MAX = 25.0;
        const float PSI_to_hPa = 68.947572932;
        
        float psi = (raw - OUTPUT_MIN) * (PSI_MAX - PSI_MIN);
        psi /= (float)(OUTPUT_MAX - OUTPUT_MIN);
        psi += PSI_MIN;
        
        pressureList[2*i] = psi * PSI_to_hPa;  // hPa
        pressureList[2*i+1] = psi;             // PSI
    }
    
    // assign value to the variables:
    bottle_hPa = pressureList[0]; bottle_PSI = pressureList[1];
    pump1_hPa = pressureList[2]; pump1_PSI = pressureList[3]; 
    pump2_hPa = pressureList[4]; pump2_PSI = pressureList[5];
    pump3_hPa = pressureList[6]; pump3_PSI = pressureList[7];
    pump4_hPa = pressureList[8]; pump4_PSI = pressureList[9];
}

float PressureMux::getPressure(uint8_t pump_CH){
    tcaSelect(pump_CH);
    float currentPress = _mpr.readPressure();
    return currentPress;
}

void PressureMux::printAllPressure(){
    for (int i = 0; i < 5; i++)
    {
        uint8_t currentCH = pumpChanList[i];
        Serial.print("[CH");
        Serial.print(currentCH);
        Serial.print("] ");
        Serial.print("Pressure = ");
        Serial.print(pressureList[2*i]);
        Serial.print("hPa, ");
        Serial.print(pressureList[2*i+1]);
        Serial.println(" PSI");
    }
    
    Serial.println("------------------------------");
}

void PressureMux::tcaSelect(uint8_t channel)
{
    if (channel > 7)
        return;
    Wire.beginTransmission(_tcaAddr);
    Wire.write(1 << channel); // select channel bit
    Wire.endTransmission();
    delayMicroseconds(100); 
    // delay(2); // settle the mux/bus
}

bool PressureMux::checkSensorOnChannel(uint8_t ch)
{
    tcaSelect(ch);
    bool ok = _mpr.begin(); // MPRLS default I2C addr = 0x18
    if (!ok)    // the _mpr is not started correctly 
    {
        Serial.print("CH: ");
        Serial.print(ch);
        Serial.println(" cannot find MPRLS, check wire");
        if (isSensorWorking)
        {
            isSensorWorking = 0;
        }
        
    }
    else
    {
        Serial.print("CH ");
        Serial.print(ch);
        Serial.println(" find MPRLS");
    }
    return ok;
}

void PressureMux::checkTCA(){
  for (uint8_t t=0; t<8; t++) {
      tcaSelect(t);
      Serial.print("TCA Port #"); Serial.println(t);

      for (uint8_t addr = 0; addr<=127; addr++) {
        if (addr == _tcaAddr) continue;

        Wire.beginTransmission(addr);
        if (!Wire.endTransmission()) {
          Serial.print("Found I2C 0x");  Serial.println(addr,HEX);
        }
      }
    }
    Serial.println("\n TCA Checking Done");
}