#pragma once
#include <Arduino.h>

// NTC Thermistor class - to read temperature from NTC thermistor voltage divider
// For lowside and highside configurations
// Lowside configuration: NTC connected between ADC input and ground, reference resistor between Vref and ADC input
//
//  +Vref
//    |
// [R_Ref]
//    |--- ADC Input Pin
//  [NTC]
//   _|_
//   GND
//
// Highside configuration: NTC connected between Vref and ADC input, reference resistor between ADC input and ground
//
//  +Vref
//    |
//  [NTC]
//    |--- ADC Input Pin
// [R_Ref]
//   _|_
//   GND

#define NTC_MAX_MOVING_AVERAGE_SAMPLES 32

class NTC_Therm {
public:
    NTC_Therm(int pin, float R_Ref, float R_NTC_25, float Beta, bool isHighSide, uint16_t adcRes, float adcVoltageRef);
    void begin();
    // Non-blocking single-ADC-read step; returns true when a filtered sample updates.
    bool sampleStep();
    float temperature();
    bool isConnected();
    void setOffset(float offsetC);
    void setScale(float scale);
    float getOffset();
    float getScale();
    void enableMovingAverage(uint16_t samples);
    void disableMovingAverage();
    void enableOversampling(uint16_t samples);
    void disableOversampling();

private:
    bool _isRawConnected(float adcRaw) const;
    float _computeTempFromRaw(float adcRaw) const;
    void _resetFilters();

    int _pin;
    float _R_Ref;
    float _R_NTC_25;
    float _Beta;
    bool _isHighSide;
    uint16_t _adcRes;
    uint32_t _adcMax;
    float _adcVoltageRef;
    float _offsetC = 0.0f;
    float _scale = 1.0f;

    // Non-blocking oversampling accumulator
    uint16_t _oversamplingSamples = 1;
    uint32_t _osSum = 0;
    uint16_t _osCount = 0;

    // Fixed-size moving average circular buffer
    uint16_t _movingAverageWindow = 1;
    bool _movingAverageEnabled = false;
    float _maBuffer[NTC_MAX_MOVING_AVERAGE_SAMPLES] = {0.0f};
    uint16_t _maHead = 0;
    uint16_t _maCount = 0;

    bool _hasValidTemp = false;
    bool _lastConnected = false;
    float _latestTempC = NAN;
};

