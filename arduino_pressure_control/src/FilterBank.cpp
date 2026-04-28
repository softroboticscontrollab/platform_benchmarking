#include "FilterBank.h"
#include <math.h>

// constructor
FilterBank::FilterBank(float samplingFreq, float cutoffFreq) {
    _fs = samplingFreq;
    _fc = cutoffFreq;
    
    // initialize coefficients
    calculateCoefficients1st();
    calculateCoefficients2nd();
}

// calculate first-order Butterworth coefficients
// transfer function: H(z) = (b0 + b1*z^-1) / (1 + a1*z^-1)
void FilterBank::calculateCoefficients1st() {
    float wc = 2.0f * PI * _fc;  
    float T = 1.0f / _fs;        // sampling period

    // pre-warping
    float K = wc * T / 2.0f;
    float denom = 1.0f + K;
    
    // calculate coefficients
    b0_1st = K / denom;
    b1_1st = K / denom;
    a1_1st = (K - 1.0f) / denom;
}

// calculate second-order Butterworth coefficients
// transfer function: H(z) = (b0 + b1*z^-1 + b2*z^-2) / (1 + a1*z^-1 + a2*z^-2)
void FilterBank::calculateCoefficients2nd() {
    float wc = 2.0f * PI * _fc;
    float T = 1.0f / _fs;
    
    // pre-warping
    float K = tan(wc * T / 2.0f);
    float K2 = K * K;
    float sqrt2 = sqrt(2.0f);
    float denom = K2 + sqrt2 * K + 1.0f;
    
    // calculate coefficients
    b0_2nd = K2 / denom;
    b1_2nd = 2.0f * K2 / denom;
    b2_2nd = K2 / denom;
    a1_2nd = (2.0f * (K2 - 1.0f)) / denom;
    a2_2nd = (K2 - sqrt2 * K + 1.0f) / denom;
}

// first-order filter implementation
float FilterBank::filter1st(float rawValue) {
    float y = b0_1st * rawValue + b1_1st * state_1st.x1 - a1_1st * state_1st.y1;
    
    // update state
    state_1st.x1 = rawValue;
    state_1st.y1 = y;
    
    return y;
}

// 2ed order filter implementation
float FilterBank::filter2nd(float rawValue) {
    float y = b0_2nd * rawValue 
            + b1_2nd * state_2nd.x1 
            + b2_2nd * state_2nd.x2
            - a1_2nd * state_2nd.y1 
            - a2_2nd * state_2nd.y2;
    
    // update states
    state_2nd.x2 = state_2nd.x1;
    state_2nd.x1 = rawValue;
    state_2nd.y2 = state_2nd.y1;
    state_2nd.y1 = y;
    
    return y;
}

// reset filter states
void FilterBank::reset() {
    state_1st.x1 = 0;
    state_1st.y1 = 0;
    
    state_2nd.x1 = 0;
    state_2nd.x2 = 0;
    state_2nd.y1 = 0;
    state_2nd.y2 = 0;
}

// update cutoff frequency
void FilterBank::setCutoffFrequency(float newFc) {
    _fc = newFc;
    calculateCoefficients1st();
    calculateCoefficients2nd();
    
    reset();    // reset states when changing frequency
}