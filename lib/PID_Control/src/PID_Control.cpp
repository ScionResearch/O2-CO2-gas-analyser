#include "PID_Control.h"

PID_Control::PID_Control(int out_pin, bool polarity) {
    _out_pin = out_pin;
    _polarity = polarity;
    _enabled = false;
    _output = 0.0;
    _Kp = 0.0;
    _Ki = 0.0;
    _Kd = 0.0;
    _setpoint = 0.0;
    
    // Internal state variables
    _integral = 0.0;
    _prev_error = 0.0;
    _prev_input = 0.0;
    _last_error = 0.0;
    _last_time = 0;
    
    // PID components for debugging
    _P_term = 0.0;
    _I_term = 0.0;
    _D_term = 0.0;
    
    // Initialize safety features
    _staleDataEnabled = false;
    _minRateOfChange = 0.0;
    _maxStaleTimeMs = 5000;  // Default 5 seconds
    _lastGoodTime = 0;
    _lastGoodValue = 0.0;
    
    _safeValueEnabled = false;
    _safeMinValue = 0.0;
    _safeMaxValue = 100.0;
    
    _errorState = false;
    
    // Configuration
    _output_min = 0.0;
    _output_max = 255.0;
    _integral_min = -1000.0;
    _integral_max = 1000.0;
    _sample_time = 100; // milliseconds
    
    if (_out_pin >= 0) {
        pinMode(_out_pin, OUTPUT);
        analogWrite(_out_pin, 0);
    }
}

void PID_Control::begin(float Kp, float Ki, float Kd, float setpoint) {
    _Kp = Kp;
    _Ki = Ki;
    _Kd = Kd;
    _setpoint = setpoint;
    
    // Reset internal state
    _integral = 0.0;
    _prev_error = 0.0;
    _prev_input = 0.0;
    _last_time = millis();
    _output = 0.0;
    
    enable();
}

void PID_Control::setpoint(float setpoint) {
    _setpoint = setpoint;
}

void PID_Control::update(float input) {
    if (!_enabled) {
        _output = 0.0;
        _P_term = 0.0;
        _I_term = 0.0;
        _D_term = 0.0;
        _last_error = 0.0;
        if (_out_pin >= 0) {
            analogWrite(_out_pin, 0);
        }
        return;
    }
    
    // Safety checks
    bool errorDetected = false;
    
    // Check for NaN
    if (isnan(input)) {
        errorDetected = true;
    }
    
    // Check safe value limits if enabled
    if (_safeValueEnabled && !errorDetected) {
        if (input < _safeMinValue || input > _safeMaxValue) {
            errorDetected = true;
        }
    }
    
    // Check for stale data if enabled (only when away from setpoint)
    if (_staleDataEnabled && !errorDetected) {
        float error = _setpoint - input;
        if (abs(error) > 0.1) {  // Only check when not at setpoint
            if (_lastGoodTime == 0) {
                _lastGoodTime = millis();
                _lastGoodValue = input;
            } else {
                unsigned long timeDiff = millis() - _lastGoodTime;
                float rateOfChange = abs(input - _lastGoodValue) / (timeDiff / 1000.0);
                
                if (rateOfChange < _minRateOfChange && timeDiff > _maxStaleTimeMs) {
                    errorDetected = true;
                } else if (rateOfChange >= _minRateOfChange) {
                    _lastGoodTime = millis();
                    _lastGoodValue = input;
                }
            }
        } else {
            // At setpoint, reset stale data timer
            _lastGoodTime = millis();
            _lastGoodValue = input;
        }
    }
    
    // Handle error state
    if (errorDetected) {
        _errorState = true;
        disable();  // Disable controller on error
        _output = 0.0;
        _P_term = 0.0;
        _I_term = 0.0;
        _D_term = 0.0;
        if (_out_pin >= 0) {
            analogWrite(_out_pin, 0);
        }
        return;
    }
    
    unsigned long now = millis();
    unsigned long time_change = now - _last_time;
    
    // Only update if sample time has passed
    if (time_change >= _sample_time) {
        // Calculate error
        float error = _setpoint - input;
        _last_error = error;
        
        // Proportional term
        _P_term = _Kp * error;
        
        // Integral term with windup protection
        _integral += _Ki * error * (time_change / 1000.0);
        
        // Clamp integral to prevent windup
        if (_integral > _integral_max) {
            _integral = _integral_max;
        } else if (_integral < _integral_min) {
            _integral = _integral_min;
        }
        
        _I_term = _integral;
        
        // Derivative term on measurement (to avoid derivative kick on setpoint change)
        _D_term = 0.0;
        if (time_change > 0) {
            _D_term = _Kd * (input - _prev_input) / (time_change / 1000.0);
        }
        
        // Calculate total output
        _output = _P_term + _I_term - _D_term; // Note: D is subtracted because we use derivative on measurement
        
        // Apply polarity
        if (!_polarity) {
            _output = -_output;
            _P_term = -_P_term;
            _I_term = -_I_term;
            _D_term = -_D_term;
        }
        
        // Clamp output
        if (_output > _output_max) {
            _output = _output_max;
        } else if (_output < _output_min) {
            _output = _output_min;
        }
        
        // Update state variables
        _prev_error = error;
        _prev_input = input;
        _last_time = now;
        
        // Write to output pin
        if (_out_pin >= 0) {
            analogWrite(_out_pin, (int)_output);
        }
    }
}

void PID_Control::enable() {
    _enabled = true;
    _errorState = false;  // Clear error state when enabling
    _lastGoodTime = 0;    // Reset stale data timer
    _last_time = millis();
}

bool PID_Control::isEnabled() {
    return _enabled;
}

void PID_Control::disable() {
    _enabled = false;
    _output = 0.0;
    if (_out_pin >= 0) {
        analogWrite(_out_pin, 0);
    }
}

void PID_Control::setPID(float Kp, float Ki, float Kd) {
    _Kp = Kp;
    _Ki = Ki;
    _Kd = Kd;
    
    // Reset integral when changing parameters
    _integral = 0.0;
}

float PID_Control::getKp() {
    return _Kp;
}

float PID_Control::getKi() {
    return _Ki;
}

float PID_Control::getKd() {
    return _Kd;
}

float PID_Control::getSetpoint() {
    return _setpoint;
}

// Safety feature implementations
void PID_Control::setStaleDataDetection(float minRateOfChange, unsigned long maxTimeMs) {
    _minRateOfChange = minRateOfChange;
    _maxStaleTimeMs = maxTimeMs;
    _lastGoodTime = 0;  // Reset timer
}

void PID_Control::enableStaleDataDetection() {
    _staleDataEnabled = true;
    _lastGoodTime = 0;  // Reset timer
}

void PID_Control::disableStaleDataDetection() {
    _staleDataEnabled = false;
}

void PID_Control::setSafeValueLimits(float minValue, float maxValue) {
    _safeMinValue = minValue;
    _safeMaxValue = maxValue;
}

void PID_Control::enableSafeValueLimits() {
    _safeValueEnabled = true;
}

void PID_Control::disableSafeValueLimits() {
    _safeValueEnabled = false;
}

bool PID_Control::isInErrorState() {
    return _errorState;
}

void PID_Control::clearErrorState() {
    _errorState = false;
    _lastGoodTime = 0;  // Reset stale data timer
}

float PID_Control::getOutput() {
    return _output;
}

// Additional helper methods
void PID_Control::setOutputLimits(float min, float max) {
    if (min >= max) return;
    _output_min = min;
    _output_max = max;
    
    // Clamp current output
    if (_output > _output_max) _output = _output_max;
    else if (_output < _output_min) _output = _output_min;
}

void PID_Control::setIntegralLimits(float min, float max) {
    if (min >= max) return;
    _integral_min = min;
    _integral_max = max;
    
    // Clamp current integral
    if (_integral > _integral_max) _integral = _integral_max;
    else if (_integral < _integral_min) _integral = _integral_min;
}

void PID_Control::setSampleTime(unsigned long sample_time) {
    if (sample_time > 0) {
        _sample_time = sample_time;
    }
}

void PID_Control::setIterm(float Iterm) {
    if (Iterm <= _integral_max && Iterm >= _integral_min) {
        _integral = Iterm;
    }
}

void PID_Control::reset() {
    _integral = 0.0;
    _prev_error = 0.0;
    _prev_input = 0.0;
    _last_time = millis();
    _output = 0.0;
    _last_error = 0.0;
    _P_term = 0.0;
    _I_term = 0.0;
    _D_term = 0.0;
}

// PID component access methods
float PID_Control::getProportional() {
    return _P_term;
}

float PID_Control::getIntegral() {
    return _I_term;
}

float PID_Control::getDerivative() {
    return _D_term;
}

float PID_Control::getError() {
    return _last_error;
}

