#pragma once
#include <Arduino.h>
#include <Wire.h>

class PumpController{
    public:
    // Ts_ms is the controller sampling time in milliseconds
    // offset is the hysteresis offset in hPa (makes the P term w.r.t. desired pressure + offset)
    explicit PumpController(uint8_t pwmPin, float Ts_ms,
                 float Kp=13.0f, float Ki=0.05f,
                 uint16_t pwmMax=255, uint16_t pwmMin=0,
                 float offset=10.0f);

    // Initilizie the pump controller
    void setup();

    // update the pump gains
    // make sure the sensor reading is updated before calling this
    void update(float des_hPa, float curr_hPa);

    // set the PID gains
    void setGains(float kp, float ki);

    // check if the controller is in fault state
    bool isFault() const { return _fault; }

    // reset the integral term (call whenever the setpoint changes significantly)
    void resetIntegral() { _integ = 0; }

    private:
    uint8_t _pwmPin;
    
    float _Kp, _Ki;
    uint16_t _uMax, _uMin;
    float _Ts;
    float _offset;

    // states
    float _err=0, _integ=0, _u=0;
    unsigned long _lastMs=0;

    // protection
    bool _fault=false;
    unsigned long _lastSwitchMs=0;
    const unsigned long _minOnOffMs=300;  // minimum on/off time for pump
};