#include "SGX_4OX.h"

SGX40X::SGX40X(int pin_sensor, int analog_read_resolution, float adc_ref_voltage, float ref_resistance, float voltage_gain)
    : _pin_sensor(pin_sensor),
      _analog_full_scale((1 << analog_read_resolution) - 1),
      _adc_ref_voltage(adc_ref_voltage),
      _ref_resistance(ref_resistance),
      _voltage_gain(voltage_gain),
      _offset_current_uA(SGX40X_DEFAULT_CURRENT_ZERO_uA),
      _span_coefficient(SGX40X_DEFAULT_SPAN_COEFFICIENT),
      _is_calibrating(false) {
}

// Debug only - print configuration to Serial
void SGX40X::begin() { 
    pinMode(_pin_sensor, INPUT);
    analogRead(_pin_sensor); // Dummy read to set up ADC on some platforms
    if (!Serial) return;
    if (SGX40X_DEBUG) Serial.printf("SGX40X sensor pin: %d, ADC full scale: %d, ADC ref voltage: %.2fV, Ref resistance: %.2f Ohm, Voltage gain: %.2f\n",
                  _pin_sensor, _analog_full_scale, _adc_ref_voltage, _ref_resistance, _voltage_gain);
}

float SGX40X::getMicroAmps(int adc_raw) {
    // Convert ADC reading to voltage
    float voltage = (adc_raw * _adc_ref_voltage) / _analog_full_scale;
    // Apply voltage gain
    voltage /= _voltage_gain;
    // Calculate current in microAmps
    return (voltage / _ref_resistance) * 1e6; // Convert to microAmps
}

float SGX40X::getTempCompensationFactor(float temperature_c) {
    int index;
    for (index = 0; index < 9; index++) {
        if (temperature_c < SGX40X_TEMP_COEFF[index + 1][0]) break;
    }
    float temp_span = SGX40X_TEMP_COEFF[index + 1][0] - SGX40X_TEMP_COEFF[index][0];
    float coeff_span = SGX40X_TEMP_COEFF[index + 1][1] - SGX40X_TEMP_COEFF[index][1];
    float temp_offset = temperature_c - SGX40X_TEMP_COEFF[index][0];
    float position = temp_offset/temp_span;
    float compensation_factor = SGX40X_TEMP_COEFF[index][1] + (coeff_span * position);

    return compensation_factor;
}

float SGX40X::readO2() {
    float current_uA = 0;
    for (int i = 0; i < 10; i++) current_uA += getMicroAmps(analogRead(_pin_sensor));
    current_uA /= 10.0; // Average over 10 readings
    return (current_uA - _offset_current_uA) * _span_coefficient;
}

float SGX40X::readO2(float temperature_c) {
    return readO2() / getTempCompensationFactor(temperature_c);
}

bool SGX40X::startZeroCalibration(float temperature_c) {
    if (_is_calibrating) return false; // Already calibrating
    if (temperature_c < SGX40X_MIN_TEMP_C || temperature_c > SGX40X_MAX_TEMP_C) return false; // Temp out of range
    _calibration_start_time = millis();
    _calibration_last_read_time = millis();
    _calibration_last_read_current = getMicroAmps(analogRead(_pin_sensor));
    _calibration_point = 0.0;
    _calibration_temperature = temperature_c;
    _is_calibrating = true;
    return true;
}

bool SGX40X::startSpanCalibration(float o2_percent, float temperature_c) {
    if (_is_calibrating) return false; // Already calibrating
    if (temperature_c < SGX40X_MIN_TEMP_C || temperature_c > SGX40X_MAX_TEMP_C) return false; // Temp out of range
    _calibration_start_time = millis();
    _calibration_last_read_time = millis();
    _calibration_last_read_current = getMicroAmps(analogRead(_pin_sensor));
    _calibration_point = o2_percent;
    _calibration_temperature = temperature_c;
    _is_calibrating = true;
    return true;
}

bool SGX40X::isCalibrating() {
    return _is_calibrating;
}

bool SGX40X::manageCalibration() {
    // Check if calibrating
    if (!_is_calibrating) return false; // Not calibrating

    // Check for timeout
    if (millis() - _calibration_start_time > SGX40X_MAX_CAL_TIME_ms) {
        _calibration_error = 1; // Did not stabilize
        _calibration_stable_count = 0;
        _is_calibrating = false;
        return false;
    }

    // Check if it's time to read again
    if (millis() - _calibration_last_read_time > SGX40X_CAL_READ_PERIOD_ms) {
        _calibration_last_read_time = millis();

        // Debug print
        if (SGX40X_DEBUG) Serial.printf("Calibration read at %d ms\n", millis() - _calibration_start_time);

        float current_uA = getMicroAmps(analogRead(_pin_sensor));
        // Debug print
        if (SGX40X_DEBUG) Serial.printf("Current reading: %.2f µA\n", current_uA);

        // Check current value is within limits
        if (current_uA < SGX40X_MIN_CURRENT_AIR_uA && _calibration_point >= 20.0) {
            _calibration_error = 2; // Current below min
            _is_calibrating = false;
            // Debug print
            if (SGX40X_DEBUG) Serial.printf("Current below minimum limit: %.2f µA\n", current_uA);
            return false;
        }
        if (current_uA > SGX40X_MAX_CURRENT_uA) {
            _calibration_error = 3; // Current above max
            _is_calibrating = false;
            // Debug print
            if (SGX40X_DEBUG) Serial.printf("Current above maximum limit: %.2f µA\n", current_uA);
            return false;
        }

        // Check if current value is stable
        if (fabs(current_uA - _calibration_last_read_current) <= SGX40X_CAL_MAX_DEVIATION_uA) {
            _calibration_stable_count++;
            // Debug print
            if (SGX40X_DEBUG) Serial.printf("Current stable count %d: %.2f µA (last: %.2f µA)\n", _calibration_stable_count, current_uA, _calibration_last_read_current);
            
            if (_calibration_stable_count >= SGX40X_CAL_STABLE_POINTS) {
                _calibration_stable_count = 0;
                // Debug print
                if (SGX40X_DEBUG) Serial.printf("Calibration successful after %d readings.\n", _calibration_stable_count);
                
                // Calibration successful - get temp compensation factor
                float temp_comp_factor = getTempCompensationFactor(_calibration_temperature);
                if (_calibration_point == 0.0) {    // Zero calibration
                    _offset_current_uA = current_uA * temp_comp_factor; // temperature compensation for zero point my be unnecessary, but it's effect is very small regardless
                    // Debug print
                    if (SGX40X_DEBUG) Serial.printf("Zero calibration set: Offset current = %.2f µA\n", _offset_current_uA);
                } else {                            // Span calibration
                    float compensated_current = current_uA - _offset_current_uA;
                    compensated_current /= temp_comp_factor;
                    _span_coefficient = _calibration_point / compensated_current;
                    // Debug print
                    if (SGX40X_DEBUG) Serial.printf("Span calibration set: Span coefficient = %.6f\n", _span_coefficient);
                }
                _is_calibrating = false;
                _calibration_error = 0; // Success
                return true;
            }
        } else {    // Not stable, reset stable count
            _calibration_stable_count = 0; // Reset stable count
            // Debug print
            if (SGX40X_DEBUG) Serial.printf("Current not stable: %.2f µA (last: %.2f µA)\n", current_uA, _calibration_last_read_current);
        }
        _calibration_last_read_current = current_uA; // Update last read current
    }
    return false;   // Calibration still in progress
}

uint8_t SGX40X::calibrationError() {
    return _calibration_error; // Return the error code
}

float SGX40X::getZeroCalibration() {
    return _offset_current_uA;
}

float SGX40X::getSpanCalibration() {
    return _span_coefficient;
}

bool SGX40X::setZeroCalibration(float zero_current_uA) {
    if (zero_current_uA < 0.0 || zero_current_uA > SGX40X_MAX_CURRENT_uA) {
        return false; // Invalid value
    }
    _offset_current_uA = zero_current_uA;
    return true;
}

bool SGX40X::setSpanCalibration(float span_coefficient) {
    if (span_coefficient <= 0.0) {
        return false; // Invalid value
    }
    _span_coefficient = span_coefficient;
    return true;
}