#pragma once
#include <Arduino.h>

// NTC Thermistor class - to read temperature from NTC thermistor voltage divider
// For lowside and highside configurations
// Lowside configuration: NTC connected between ADC input and ground, reference resistor between Vref and ADC input
// 
//  +Vref
//    |
// [R_Ref]
//    |─── ADC Input Pin
//  [NTC]
//   _|_
//   GND
//
// Highside configuration: NTC connected between Vref and ADC input, reference resistor between ADC input and ground
//
//  +Vref
//    |
//  [NTC]
//    |─── ADC Input Pin
// [R_Ref]
//   _|_
//   GND

class NTC_Therm {
public:
    NTC_Therm(int pin, float R_Ref, float R_NTC_25, float Beta, bool isHighSide, uint16_t adcRes, float adcVoltageRef);
    void begin();
    float temperature();
    bool isConnected();
    void setOffset(float offsetC);
    void setScale(float scale);
    float getOffset();
    float getScale();

private:
    int _pin;
    float _R_Ref;
    float _R_NTC_25;
    float _Beta;
    bool _isHighSide;
    uint16_t _adcRes;
    uint32_t _adcMax;
    float _adcVoltageRef;
    float _offsetC = 0.0;
    float _scale = 1.0;
};