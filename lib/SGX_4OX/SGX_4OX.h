#pragma once
#include <Arduino.h>

#define SGX40X_DEBUG    1

// Temperature coefficient array
const float SGX40X_TEMP_COEFF[9][2] = {
    { -30.0, 0.868 },
    { -20.0, 0.895 },
    { -10.0, 0.946 },
    { 0.0,   0.960 },
    { 10.0,  0.976 },
    { 20.0,  1.000 },
    { 30.0,  1.025 },
    { 40.0,  1.036 },
    { 50.0,  1.042 }
};

// Limits
#define SGX40X_MIN_TEMP_C               -30.0
#define SGX40X_MAX_TEMP_C               50.0
#define SGX40X_MAX_CURRENT_uA           150.0
#define SGX40X_MIN_CURRENT_AIR_uA       70.0
#define SGX40X_MAX_CURRENT_AIR_uA       130.0
#define SGX40X_DEFAULT_SPAN_COEFFICIENT 0.193877551 // 20.9% / 107.7 µA
#define SGX40X_DEFAULT_CURRENT_ZERO_uA  0.06
#define SGX40X_MAX_CAL_TIME_ms          60000UL
#define SGX40X_CAL_READ_PERIOD_ms       1000UL
#define SGX40X_CAL_STABLE_POINTS        5
#define SGX40X_CAL_MAX_DEVIATION_uA     0.1

class SGX40X {
public:
    SGX40X(int pin_sensor, int analog_read_resolution, float adc_ref_voltage, float ref_resistance, float voltage_gain);           // Constructor
    void begin();                                                               // Initialize sensor (for debug only)
    float getMicroAmps(int adc_raw);                                            // Read sensor current in microAmps
    float getTempCompensationFactor(float temperature_c);                       // Get temperature compensation factor
    float readO2();                                                             // Read O2 concentration at reference temperature (20°C)
    float readO2(float temperature_c);                                          // Read O2 concentration with temperature compensation
    bool startZeroCalibration(float temperature_c);                             // Start 0% O2 calibration process
    bool startSpanCalibration(float o2_percent, float temperature_c);           // Start span calibration process
    bool isCalibrating();                                                       // Check if calibration is in progress
    bool manageCalibration();                                                   // Manage calibration steps, to be called periodically while calibrating
    uint8_t calibrationError();                                                 // Get result of last calibration -> 0=success, 1=did not stabilize, 2=current below min, 3=current above max
    float getZeroCalibration();                                                 // Get 0% O2 calibration value for NVM storage
    float getSpanCalibration();                                                 // Get span calibration value for NVM storage
    bool setZeroCalibration(float zero_current_uA);                             // Set 0% O2 calibration value from NVM
    bool setSpanCalibration(float span_coefficient);                            // Set span calibration value from NVM

private:
    // Initial parameters
    int _pin_sensor;
    int _analog_full_scale;
    float _adc_ref_voltage;
    float _ref_resistance;
    float _voltage_gain;
    // Calibration parameters
    float _offset_current_uA;
    float _span_coefficient;
    // Calibration process variables
    bool _is_calibrating;
    uint32_t _calibration_start_time;
    uint32_t _calibration_last_read_time;
    float _calibration_last_read_current;
    float _calibration_point;
    float _calibration_temperature;
    uint8_t _calibration_error;
    uint8_t _calibration_stable_count;
};