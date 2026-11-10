#include "NTC_Therm.h"
#include <math.h>

NTC_Therm::NTC_Therm(int pin, float R_Ref, float R_NTC_25, float Beta, bool isHighSide, uint16_t adcRes, float adcVoltageRef)
    : _pin(pin),
      _R_Ref(R_Ref),
      _R_NTC_25(R_NTC_25),
      _Beta(Beta),
      _isHighSide(isHighSide),
      _adcRes(adcRes),
      _adcMax((1UL << adcRes) - 1UL),
      _adcVoltageRef(adcVoltageRef) {
}

void NTC_Therm::begin() {
    pinMode(_pin, INPUT);
    (void)analogRead(_pin);
    _resetFilters();
}

void NTC_Therm::_resetFilters() {
    _osSum = 0;
    _osCount = 0;
    _maHead = 0;
    _maCount = 0;
    _hasValidTemp = false;
    _latestTempC = NAN;
}

bool NTC_Therm::_isRawConnected(float adcRaw) const {
    float minThresh = (float)_adcMax / 20.0f;
    float maxThresh = (float)_adcMax - minThresh;
    return (adcRaw > minThresh && adcRaw < maxThresh);
}

float NTC_Therm::_computeTempFromRaw(float adcRaw) const {
    if (adcRaw <= 0.5f || adcRaw >= (float)_adcMax - 0.5f) {
        return NAN;
    }

    float R_ntc = 0.0f;
    if (!_isHighSide) {
        // Low-side NTC, high-side fixed resistor
        R_ntc = _R_Ref * (adcRaw / ((float)_adcMax - adcRaw));
    } else {
        // High-side NTC, low-side fixed resistor
        R_ntc = _R_Ref * (((float)_adcMax - adcRaw) / adcRaw);
    }

    if (R_ntc <= 0.0f || _R_NTC_25 <= 0.0f || _Beta <= 0.0f) {
        return NAN;
    }

    const float T0 = 298.15f; // 25 deg C in Kelvin
    float invT = (1.0f / T0) + (1.0f / _Beta) * logf(R_ntc / _R_NTC_25);
    if (invT <= 0.0f) {
        return NAN;
    }
    return ((1.0f / invT) - 273.15f - _offsetC) * _scale;
}

bool NTC_Therm::sampleStep() {
    uint32_t adcRaw = (uint32_t)analogRead(_pin);
    if (!_isRawConnected((float)adcRaw)) {
        _lastConnected = false;
        _resetFilters();
        return true;
    }
    _lastConnected = true;

    _osSum += adcRaw;
    _osCount++;

    uint16_t targetOs = (_oversamplingSamples > 0) ? _oversamplingSamples : 1;
    if (_osCount < targetOs) {
        return false;
    }

    float avgRaw = (float)_osSum / (float)_osCount;
    _osSum = 0;
    _osCount = 0;

    float tempC = _computeTempFromRaw(avgRaw);
    if (isnan(tempC)) {
        _resetFilters();
        return true;
    }

    if (_movingAverageEnabled && _movingAverageWindow > 1) {
        _maBuffer[_maHead] = tempC;
        _maHead = (_maHead + 1) % _movingAverageWindow;
        if (_maCount < _movingAverageWindow) {
            _maCount++;
        }
        float sum = 0.0f;
        for (uint16_t i = 0; i < _maCount; i++) {
            sum += _maBuffer[i];
        }
        _latestTempC = sum / (float)_maCount;
    } else {
        _latestTempC = tempC;
    }

    _hasValidTemp = true;
    return true;
}

float NTC_Therm::temperature() {
    if (!_hasValidTemp) {
        uint32_t adcRaw = (uint32_t)analogRead(_pin);
        if (!_isRawConnected((float)adcRaw)) {
            _lastConnected = false;
            return NAN;
        }
        _lastConnected = true;
        float tempC = _computeTempFromRaw((float)adcRaw);
        if (!isnan(tempC)) {
            _latestTempC = tempC;
            _hasValidTemp = true;
        }
        return tempC;
    }
    return _lastConnected ? _latestTempC : NAN;
}

bool NTC_Therm::isConnected() {
    uint16_t adcRaw = (uint16_t)analogRead(_pin);
    _lastConnected = _isRawConnected((float)adcRaw);
    return _lastConnected;
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

void NTC_Therm::enableMovingAverage(uint16_t samples) {
    if (samples <= 1) {
        disableMovingAverage();
        return;
    }
    if (samples > NTC_MAX_MOVING_AVERAGE_SAMPLES) {
        samples = NTC_MAX_MOVING_AVERAGE_SAMPLES;
    }
    _movingAverageWindow = samples;
    _movingAverageEnabled = true;
    _resetFilters();
}

void NTC_Therm::disableMovingAverage() {
    _movingAverageEnabled = false;
    _movingAverageWindow = 1;
    _maHead = 0;
    _maCount = 0;
}

void NTC_Therm::enableOversampling(uint16_t samples) {
    _oversamplingSamples = (samples > 0) ? samples : 1;
    _osSum = 0;
    _osCount = 0;
}

void NTC_Therm::disableOversampling() {
    _oversamplingSamples = 1;
    _osSum = 0;
    _osCount = 0;
}

