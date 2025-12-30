#include "SGX_INIR2_CD100.h"

CD100::CD100(HardwareSerial &serial, uint32_t baudrate)
    : _serial(serial), _baudrate(baudrate) {
}

void CD100::begin() {
    _serial.begin(_baudrate, SERIAL_8N2);
}

bool CD100::manage() {
    if (_serial.available() > 100) {
        // Clear buffer if too much data
        while (_serial.available()) {
            _serial.read();
        }
        return false;
    }
    if (_serial.available() >= 70) {
        int num_bytes= _serial.available(); // Get number of available bytes
        uint8_t data[100];
        _serial.readBytes(data, min(num_bytes, 100)); // Read available bytes into buffer
        // Get start of frame
        int firstCharIndex = -1;
        for (int i = 0; i < 10; i++) {
            if (data[i+8] == 0x0A) {
                firstCharIndex = i;
                break;
            }
        }
        if (firstCharIndex == -1 || firstCharIndex + 70 > num_bytes) {
            return false; // Not enough data for a full frame
        }
        // Iterate through frames
        for (int i = 0 + firstCharIndex; i < num_bytes; i+=10) {            
            if (memcmp(&data[i], CD100_START_STRING, 8) == 0) {
                if (i + 70 > num_bytes) {
                    return false; // Not enough data for a full frame
                }
                // Pack ASCII hex into uint32_t array for CRC calculation
                uint32_t converted[4];
                char buf[9];
                converted[0] = 0x0000005b; // Start value
                for (uint8_t j = 1; j < 4; j++) {
                    memcpy(buf, &data[i+(j*10)], 8);
                    buf[8] = '\0';
                    converted[j] = strtoul(buf, NULL, 16);
                }
                // Calculate CRC
                uint32_t crc = CRC(converted, 4);
                // Get transmitted CRC
                memcpy(buf, &data[i+40], 8);
                buf[8] = '\0';
                uint32_t crcTransmitted = strtoul(buf, NULL, 16);
                // Validate CRC
                if (crc != crcTransmitted) {
                    return false;
                }
                // Validate 1's complement
                uint32_t crcOnesComplement = ~crc;
                memcpy(buf, &data[i+50], 8);
                buf[8] = '\0';
                uint32_t crcOnesComplementTransmitted = strtoul(buf, NULL, 16);
                if (crcOnesComplement != crcOnesComplementTransmitted) {
                    return false;
                }

                // Extract data
                _lastCO2ppm = (converted[1] + _calibrationOffset) * _calibrationScale;
                _lastTemperatureC = (converted[3] * 0.1f) - 273.2f;
                _lastFaultStatus = converted[2];
                
                return true;
            }
        }
    }
    return false;
}

uint32_t CD100::CO2() {
    return _lastCO2ppm;
}

float CD100::temperature() {
    return _lastTemperatureC;
}

uint32_t CD100::faultStatus() {
    return _lastFaultStatus;
}

bool CD100::setCalibrationOffset(int32_t offset) {
    if (offset < -5000 || offset > 5000) {
        return false;
    }
    _calibrationOffset = offset;
    return true;
}

bool CD100::setCalibrationScale(float scale) {
    if (scale < 0.8f || scale > 1.2f) {
        return false;
    }
    _calibrationScale = scale;
    return true;
}

int32_t CD100::getCalibrationOffset() {
    return _calibrationOffset;
}

float CD100::getCalibrationScale() {
    return _calibrationScale;
}

// Private functions
uint32_t CD100::CRC(uint32_t *data, uint8_t length) {
    uint32_t crc = 0;
    for (uint8_t i = 0; i < length; i++) {
        for (uint8_t j = 0; j < 4; j++) {
            uint8_t byte = (data[i] >> (j * 8) & 0xFF);
            crc += byte;
        }
    }
    return crc;
}