#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP4728.h>
#include "ABValve.h"

// Low-level controller for four proportional valves + their associated AB valves.
// For each chamber i:
//   - The sign of the pressure error controls the AB valve direction (inflate / deflate).
//   - The magnitude of the error is mapped to a DAC output in [dacMin, dacMax],
//     which sets the opening of the proportional valve.
// The mapping uses the bottle pressure as a nominal maximum pressure.
class ProportionalValveController {
public:
    // Constructor
    // - valve: reference to the ABValve instance controlling the four on/off valves.
    // - bottlePressure_hPa: nominal reservoir pressure (in hPa), used to define
    //   the maximum expected pressure error (bottlePressure - atmosphericPressure).
    // - Ts_ms: controller sampling time [ms]. This should match the calling period
    //   of update() in the main loop.
    // - Kp, Ki: PI gains shared by all four chambers.
    // - dacMax, dacMin: DAC output bounds (typically 4095 and 0 for a 12-bit DAC).
    // - nominalZero_hPa: targetPressure < nominalZero_hPa is considered 0 and the valve should be open.
    explicit ProportionalValveController(ABValve& valve,
                                         float bottlePressure_hPa,
                                         float Ts_ms,
                                         float Kp[4],
                                         float Ki[4],
                                         uint16_t dacMax = 4095,
                                         uint16_t dacMin = 0,
                                         float nominalZero_hPa = 1010.0f);

    // Initialize the proportional valve controller:
    //  - Initialize the MCP4728 DAC (begin()).
    //  - On success, set all proportional valves to fully open (aperture = 1.0).
    //  - On failure, print an error message over Serial and block 
    void setup();

    // Update the four PI controllers and write the corresponding DAC outputs.
    // Inputs:
    //  - desPi_hPa: desired chamber pressures [hPa]
    //  - currPi_hPa: measured chamber pressures [hPa]
    //
    // Assumptions:
    //  - Pressure measurements have been updated before calling this function.
    //  - update() is called with a fixed period Ts_ms (passed in the constructor).
    void update(float desP1_hPa, float currP1_hPa,
                float desP2_hPa, float currP2_hPa,
                float desP3_hPa, float currP3_hPa,
                float desP4_hPa, float currP4_hPa);

    // Set the PI gains at runtime (applied to all four chambers).
    void setGains(float kp[4], float ki[4]);

    // Reset the integral terms for all chambers.
    void resetIntegral();

    // Any target pressure below nominalZero_hPa is considered fully released.
    void setNominalZeroPressure(float nominalZero_hPa);

    // Set the ith (i = 1, 2, 3, 4) DAC to maximum output (fully open) and AB valve on to inflate
    void setChannelFullyInflate(uint8_t channel_idx);

    // Set the ith (i = 1, 2, 3, 4) DAC to maximum output (fully open) and AB valve off to deflate
    void setChannelFullyDeflate(uint8_t channel_idx);
    

    // get the last DAC outputs for the four channels by object.dacOutputsi, i = 1,2,3,4
    uint16_t dacOutputs1, dacOutputs2, dacOutputs3, dacOutputs4;


private:

    // Recompute the mapping factor based on the current bottle pressure
    // and DAC bounds:
    //
    //   maxError = bottlePressure - atmosphericPressure
    //   mapFactor = (dacMax - dacMin) / (maxError - minError)
    //
    // The DAC command is then computed as:
    //   DAC = Kp * error * mapFactor + dacMin
    void _calculateMapFactor();

    // Hardware interfaces
    ABValve&        _valve;
    Adafruit_MCP4728 _dac;

    // PI gains (shared across the four channels)
    float _Kp[4];
    float _Ki[4];

    // Sampling time [ms] as used for the integral term:
    // integral += error * (Ts_ms / 1000.0f)
    float _Ts_ms;

    // Nominal bottle pressure [hPa]
    float _bottlePressure_hPa;

    // Atmospheric pressure [hPa]
    static constexpr float ATM_PRESSURE_HPA = 1021.25f;


    // Pressure error bounds [hPa] used for the linear mapping.
    float _minPressureError_hPa;  // usually 0
    float _maxPressureError_hPa;  // bottlePressure_hPa - ATM_PRESSURE_HPA

    // Mapping factor from pressure error to DAC units.
    // Used in:
    //   DAC = Kp * error * _mapFactor + dacMin
    float _mapFactor;

    // Controller states for the four chambers
    float _err[4]   = {0.0f, 0.0f, 0.0f, 0.0f};
    float _integ[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    // Last DAC outputs for the four channels, between 0 and 4095
    uint16_t _dacOutputs[4] = {0, 0, 0, 0};

    // DAC bounds
    uint16_t _dacMax;
    uint16_t _dacMin;

    // Nominal zero pressure threshold [hPa]
    float _nominalZero_hPa;


};