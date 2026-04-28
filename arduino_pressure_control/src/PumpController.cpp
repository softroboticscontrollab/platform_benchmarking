#include "PumpController.h";

PumpController::PumpController(uint8_t pwmPin, float Ts_ms,
                 float Kp, float Ki,
                 uint16_t pwmMax, uint16_t pwmMin,
                  float offset)
    : _pwmPin(pwmPin), _Kp(Kp), _Ki(Ki), _uMax(pwmMax), _uMin(pwmMin), _Ts(Ts_ms), _offset(offset)
{
}

void PumpController::setup(){
    pinMode(_pwmPin, OUTPUT);
    analogWrite(_pwmPin, 0);
}

void PumpController::setGains(float kp, float ki){
    _Kp = kp;
    _Ki = ki;
}

void PumpController::update(float des_hPa, float curr_hPa) {
  unsigned long now = millis();

  // 1) Take the pressure from the sensor 
  if (!isfinite(curr_hPa) || curr_hPa < 0 || curr_hPa > 20000) {
    _fault = true;
    analogWrite(_pwmPin, 0);
    return;
  }
  _fault = false;

  // 2) PI controller
  _err = des_hPa + _offset - curr_hPa;    // P term

  _integ += _err * (_Ts/1000.0f);   // I term
  if (_integ < 0)     _integ = 0;      // single side integral limit
  if (_integ > 5000)  _integ = 5000;   // upper limit

  float u = _Kp*_err + _Ki*_integ;    

  // 3) Clip control signal to PWM range
  if (u < _uMin) u = _uMin;
  if (u > _uMax) u = _uMax;

  // 4) If already achieved desired pressure, reset integral term
    if (curr_hPa >= des_hPa) {
        _integ = 0;
    }       // Actually not necessary here since KI is small

  // 5) Minimum on/off time protection (to prevent rapid switching)
    if ((_u == 0 && u > 0) || (_u > 0 && u == 0)) { // switching on/off
        if (now - _lastSwitchMs < _minOnOffMs) {
        u = _u;   // keep previous state
        } else {
        _lastSwitchMs = now;
        }
    }

  _u = u; // save control signal
  analogWrite(_pwmPin, (int)_u);
}