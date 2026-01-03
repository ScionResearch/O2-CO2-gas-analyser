#pragma once

#include <Arduino.h>

class PID_Control {
    public: 
        PID_Control(int out_pin, bool polarity);
        void begin(float Kp, float Ki, float Kd, float setpoint);
        void setpoint(float setpoint);
        void update(float input);
        void enable();
        bool isEnabled();
        void disable();
        void setPID(float Kp, float Ki, float Kd);
        float getKp();
        float getKi();
        float getKd();
        float getSetpoint();
        float getOutput();
        
        // PID component access methods
        float getProportional();
        float getIntegral();
        float getDerivative();
        float getError();
        
        // Safety features
        void setStaleDataDetection(float minRateOfChange, unsigned long maxTimeMs);
        void enableStaleDataDetection();
        void disableStaleDataDetection();
        void setSafeValueLimits(float minValue, float maxValue);
        void enableSafeValueLimits();
        void disableSafeValueLimits();
        bool isInErrorState();
        void clearErrorState();
        
        // Additional helper methods
        void setOutputLimits(float min, float max);
        void setIntegralLimits(float min, float max);
        void setSampleTime(unsigned long sample_time);
        void setIterm(float Iterm);
        void resetIterm();
        void reset();

    private:
        int _out_pin;
        float _Kp;
        float _Ki;
        float _Kd;
        float _setpoint;
        bool _polarity;
        bool _enabled;
        float _output;
        
        // Internal state variables
        float _integral;
        float _prev_error;
        float _prev_input;
        float _last_error;  // Store for proportional term access
        unsigned long _last_time;
        
        // PID components for debugging
        float _P_term;
        float _I_term;
        float _D_term;
        
        // Safety feature variables
        bool _staleDataEnabled;
        float _minRateOfChange;
        unsigned long _maxStaleTimeMs;
        unsigned long _lastGoodTime;
        float _lastGoodValue;
        
        bool _safeValueEnabled;
        float _safeMinValue;
        float _safeMaxValue;
        
        bool _errorState;
        
        // Configuration
        float _output_min;
        float _output_max;
        float _integral_min;
        float _integral_max;
        unsigned long _sample_time;
};