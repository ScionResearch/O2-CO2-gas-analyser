#pragma once

#include <Arduino.h>

#define CD100_START_STRING "0000005b"

class CD100 {
public:
    CD100(HardwareSerial &serial, uint32_t baudrate = 38400);
    void begin();
    bool manage();
    uint32_t CO2();
    float temperature();
    uint32_t faultStatus();
    bool setCalibrationOffset(int32_t offset);
    bool setCalibrationScale(float scale);
    int32_t getCalibrationOffset();
    float getCalibrationScale();

private:
    uint32_t CRC(uint32_t *data, uint8_t length);

    // Member variables
    HardwareSerial &_serial;
    uint32_t _baudrate;
    uint32_t _lastCO2ppm = 0;
    float _lastTemperatureC = 0.0;
    uint32_t _lastFaultStatus = 0;
    uint32_t _calibrationOffset = 0;
    float _calibrationScale = 1.0;
};