#include "NTC_Therm.h"

NTC_Therm::NTC_Therm(int pin, float R_Ref, float R_NTC_25, float Beta, bool isHighSide, uint16_t adcRes, float adcVoltageRef)
    : _pin(pin), _R_Ref(R_Ref), _R_NTC_25(R_NTC_25), _Beta(Beta), _isHighSide(isHighSide), _adcRes(adcRes), _adcMax((1 << adcRes) - 1), _adcVoltageRef(adcVoltageRef) {
}

void NTC_Therm::begin() {
    pinMode(_pin, INPUT);
    analogRead(_pin);
}

float NTC_Therm::temperature() {
    if (!isConnected()) {
        return NAN;
    }
    
    uint16_t adcRaw = analogRead(_pin);

    float R_ntc = 0;

    if (!_isHighSide) {
        // Low-side NTC, high-side fixed resistor
        R_ntc = _R_Ref * ((float)adcRaw / (float)(_adcMax - adcRaw));
    } else {
        // High-side NTC, low-side fixed resistor
        R_ntc = _R_Ref * ((float)(_adcMax - adcRaw) / (float)adcRaw);
        
    }
    
    const float T0 = 298.15f; // 25°C in Kelvin
    float invT = (1.0f / T0) + (1.0f / _Beta) * log(R_ntc / _R_NTC_25);
    return ((1.0f / invT) - 273.15f - _offsetC) * _scale;
}

bool NTC_Therm::isConnected() {
    uint16_t adcRaw = analogRead(_pin);
    return (adcRaw > (_adcMax / 20) && adcRaw < (_adcMax - (_adcMax / 20)));
}

void NTC_Therm::setOffset(float offsetC) {
    _offsetC = offsetC;
}

void NTC_Therm::setScale(float scale) {
    _scale = scale;
}

float NTC_Therm::getOffset() {
    return _offsetC;
}

float NTC_Therm::getScale() {
    return _scale;
}