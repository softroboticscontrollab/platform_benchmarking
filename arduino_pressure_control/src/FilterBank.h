#pragma once

#include "Arduino.h"

// 1st order filter state
struct FirstOrderState {
    float x1 = 0;  // input history
    float y1 = 0;  // output history
};

// 2nd order filter state
struct BiquadState {
    float x1 = 0, x2 = 0;  // input history
    float y1 = 0, y2 = 0;  // output history
};

class FilterBank {
public:
    // constructor
    FilterBank(float samplingFreq, float cutoffFreq);
    
    // 1st order Butterworth filter
    float filter1st(float rawValue);
    
    // 2nd order Butterworth filter
    float filter2nd(float rawValue);
    
    // Reset filter states
    void reset();
    
    // Update cutoff frequency (for dynamic adjustment)
    void setCutoffFrequency(float newFc);

private:
    // Filter states
    FirstOrderState state_1st;
    BiquadState state_2nd;
    
    // 1st Butterworth coefficients
    float b0_1st, b1_1st, a1_1st;
    
    // 2nd Butterworth coefficients
    float b0_2nd, b1_2nd, b2_2nd, a1_2nd, a2_2nd;

    // Sampling parameters
    float _fs;  // sampling frequency (Hz)
    float _fc;  // cutoff frequency (Hz)

    // Internal functions to calculate coefficients
    void calculateCoefficients1st();
    void calculateCoefficients2nd();
};
