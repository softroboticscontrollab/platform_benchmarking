#include "ProportionalValveController.h"

ProportionalValveController::ProportionalValveController(ABValve& valve,
                                                         float bottlePressure_hPa,
                                                         float Ts_ms,
                                                         float Kp[4],
                                                         float Ki[4],
                                                         uint16_t dacMax,
                                                         uint16_t dacMin,
                                                         float nominalZero_hPa)
    : _valve(valve),
      _bottlePressure_hPa(bottlePressure_hPa),
      _Ts_ms(Ts_ms),
      _dacMax(dacMax),
      _dacMin(dacMin),
      _nominalZero_hPa(nominalZero_hPa),
      _dac() 
      {_Kp[0] = Kp[0]; _Kp[1] = Kp[1]; _Kp[2] = Kp[2]; _Kp[3] = Kp[3];
        _Ki[0] = Ki[0]; _Ki[1] = Ki[1]; _Ki[2] = Ki[2]; _Ki[3] = Ki[3];
}

void ProportionalValveController::setup() {
    Serial.println();
    Serial.println("=== Proportional Valve Controller Setup ===");

    // Initialize the MCP4728 DAC
    if (!_dac.begin()) {
        Serial.println("Failed to find MCP4728 chip");
        while (1) {
            delay(10);
        }
    }

    // check if the bottle pressure is reasonable and compute the mapping factor
    if (_bottlePressure_hPa <= ATM_PRESSURE_HPA) {
        Serial.println("Error: Bottle pressure must be greater than atmospheric pressure.");
        while (1) {
            delay(10);
        }
    }
    _calculateMapFactor();

    // Set all proportional valves to fully open 
    _dac.setChannelValue(MCP4728_CHANNEL_A, _dacMax);
    _dac.setChannelValue(MCP4728_CHANNEL_B, _dacMax);
    _dac.setChannelValue(MCP4728_CHANNEL_C, _dacMax);
    _dac.setChannelValue(MCP4728_CHANNEL_D, _dacMax);
    _dacOutputs[0] = _dacMax;
    _dacOutputs[1] = _dacMax;
    _dacOutputs[2] = _dacMax;
    _dacOutputs[3] = _dacMax;   

    Serial.println("MCP4728 initialized successfully.");
}

void ProportionalValveController::update(float desP1_hPa, float currP1_hPa,
                                        float desP2_hPa, float currP2_hPa,
                                        float desP3_hPa, float currP3_hPa,
                                        float desP4_hPa, float currP4_hPa) {

    float desiredPressures[4] = {desP1_hPa, desP2_hPa, desP3_hPa, desP4_hPa};
    float currentPressures[4] = {currP1_hPa, currP2_hPa, currP3_hPa, currP4_hPa};
    float errStorage[4];

    // FOR ANTI-WINDUP
    float Kaw[4] = { 2.0f*_Ki[0], 2.0f*_Ki[1], 2.0f*_Ki[2], 2.0f*_Ki[3] };   // anti-windup gain

    for (int i = 0; i < 4; i++) {

        // if desired pressure is below nominal zero, consider it to be fully released
        // open the dac fully and skip PI control, reset integral term
        if (desiredPressures[i] < _nominalZero_hPa) {
            _err[i] = 0.0f;
            _integ[i] = 0.0f;

            // deflate by closing the AB valve
            _valve.valveOff(i + 1);

            // Fully open proportional valve to quickly release pressure
            _dacOutputs[i] = _dacMax;
            continue;
        }

        // Compute the pressure error
        _err[i] = desiredPressures[i] - currentPressures[i];
        errStorage[i] = _err[i];
        // Update the integral term and set bounds
        _integ[i] += _err[i] * (_Ts_ms / 1000.0f);
        if (_integ[i] < -100.0f) {
            _integ[i] = -100.0f;
        }
        if (_integ[i] > 100.0f) {
            _integ[i] = 100.0f;
        }

        // Compute the raw DAC command
        float dacCommand_raw = _Kp[i] * _err[i] * _mapFactor + _Ki[i] * _integ[i] * _mapFactor + _dacMin;
        // float kp_print =  _Kp[i] * _err[i] * _mapFactor;
        // float ki_print = _Ki[i] * _integ[i] * _mapFactor;
        // Serial.print("Kp is: ");
        // Serial.print(kp_print);
        // Serial.print(" Ki is: ");
        // Serial.print(ki_print);
        // Serial.print(" dac Raw is: ");
        // Serial.print(dacCommand_raw);
        // Serial.println("");

        // set AB valve based on error sign
        if (_err[i] >= 0) {     // haven't achieved desired pressure (need inflation)
            _valve.valveOn(i + 1);  // valveOn expects 1-based index
        } else {    // achieved desired pressure or overpressure (need deflation)
            _valve.valveOff(i + 1);
        } 

        // check if abs(dacCommand_raw) is beyond bounds, if so, only use the Kp*error term
        // Prevent integral windup from causing large overshoot
        // Take absolute value because this is the aperture command, not the flow direction
        float dacCommand;
        if (fabs(dacCommand_raw) > _dacMax) {   
            dacCommand = _Kp[i] * _err[i] * _mapFactor + _dacMin;
            _integ[i] = 0.0f;  // we don't use I becasue this case is either Kp too large or windup
        } else {
            dacCommand = dacCommand_raw;
        }
        dacCommand = fabs(dacCommand);

        // float dacCommand = fabs(dacCommand_raw);      

        // Saturate DAC command to [dacMin, dacMax]
        if (dacCommand < _dacMin) {
            dacCommand = _dacMin;
        }
        if (dacCommand > _dacMax) {
            dacCommand = _dacMax;
        }
        
        // Back-calculation for anti-windup
        // float aw_term = Kaw[i] * (dacCommand - fabs(dacCommand_raw));
        // _integ[i] += aw_term * (_Ts_ms / 1000.0f);

        // floor the DAC command to an integer and store in _dacOutputs
        _dacOutputs[i] = floor(dacCommand);
    }

    // send DAC commands to MCP4728
    _dac.setChannelValue(MCP4728_CHANNEL_A, _dacOutputs[0]);
    _dac.setChannelValue(MCP4728_CHANNEL_B, _dacOutputs[1]);
    _dac.setChannelValue(MCP4728_CHANNEL_C, _dacOutputs[2]);
    _dac.setChannelValue(MCP4728_CHANNEL_D, _dacOutputs[3]);

    // for (int i = 0; i < 4; i++) {
    //     // set AB valve based on error sign
    //     if (errStorage[i] >= 0) {     // haven't achieved desired pressure (need inflation)
    //         _valve.valveOn(i + 1);  // valveOn expects 1-based index
    //     } else {    // achieved desired pressure or overpressure (need deflation)
    //         _valve.valveOff(i + 1);
    //     }  
    // }   

    // assign to public members for external access
    dacOutputs1 = _dacOutputs[0];
    dacOutputs2 = _dacOutputs[1];
    dacOutputs3 = _dacOutputs[2];
    dacOutputs4 = _dacOutputs[3];
}

void ProportionalValveController::setGains(float kp[4], float ki[4]) {
    for (int i = 0; i < 4; i++) {
        _Kp[i] = kp[i];
        _Ki[i] = ki[i];
    }
}

void ProportionalValveController::resetIntegral() {
    for (int i = 0; i < 4; i++) {
        _integ[i] = 0.0f;
    }
}

void ProportionalValveController::setNominalZeroPressure(float nominalZero_hPa) {
    _nominalZero_hPa = nominalZero_hPa;
}

void ProportionalValveController::_calculateMapFactor() {
    _minPressureError_hPa = 0.0f;
    _maxPressureError_hPa = _bottlePressure_hPa - ATM_PRESSURE_HPA;

    _mapFactor = (_dacMax - _dacMin) / (_maxPressureError_hPa - _minPressureError_hPa);
}

void ProportionalValveController::setChannelFullyInflate(uint8_t channel_idx) {
    if (channel_idx < 1 || channel_idx > 4) {
        Serial.println("Error: channel_idx must be between 1 and 4.");
        return;
    }
    // use switch to set the corresponding channel
    switch (channel_idx) {
        case 1:
            _valve.valveOn(1);
            _dac.setChannelValue(MCP4728_CHANNEL_A, _dacMax);
            _dacOutputs[0] = _dacMax;
            break;
        case 2:
            _valve.valveOn(2);
            _dac.setChannelValue(MCP4728_CHANNEL_B, _dacMax);
            _dacOutputs[1] = _dacMax;
            break;
        case 3:
            _valve.valveOn(3);
            _dac.setChannelValue(MCP4728_CHANNEL_C, _dacMax);
            _dacOutputs[2] = _dacMax;
            break;
        case 4:
            _valve.valveOn(4);
            _dac.setChannelValue(MCP4728_CHANNEL_D, _dacMax);
            _dacOutputs[3] = _dacMax;
            break;  
        }
    // assign to public members for external access
    dacOutputs1 = _dacOutputs[0];
    dacOutputs2 = _dacOutputs[1];
    dacOutputs3 = _dacOutputs[2];
    dacOutputs4 = _dacOutputs[3];
}

void ProportionalValveController::setChannelFullyDeflate(uint8_t channel_idx) {
    if (channel_idx < 1 || channel_idx > 4) {
        Serial.println("Error: channel_idx must be between 1 and 4.");
        return;
    }
    // use switch to set the corresponding channel
    switch (channel_idx) {
        case 1:
            _valve.valveOff(1);
            _dac.setChannelValue(MCP4728_CHANNEL_A, _dacMax);
            _dacOutputs[0] = _dacMax;
            break;
        case 2:
            _valve.valveOff(2);
            _dac.setChannelValue(MCP4728_CHANNEL_B, _dacMax);
            _dacOutputs[1] = _dacMax;
            break;
        case 3:
            _valve.valveOff(3);
            _dac.setChannelValue(MCP4728_CHANNEL_C, _dacMax);
            _dacOutputs[2] = _dacMax;
            break;
        case 4:
            _valve.valveOff(4);
            _dac.setChannelValue(MCP4728_CHANNEL_D, _dacMax);
            _dacOutputs[3] = _dacMax;
            break;  
        }
    // assign to public members for external access
    dacOutputs1 = _dacOutputs[0];
    dacOutputs2 = _dacOutputs[1];
    dacOutputs3 = _dacOutputs[2];
    dacOutputs4 = _dacOutputs[3];
}