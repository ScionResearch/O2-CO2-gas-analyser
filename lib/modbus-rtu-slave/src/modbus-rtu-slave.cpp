#include "modbus-rtu-slave.h"
#include <string.h>

ModbusRTUSlave::ModbusRTUSlave(HardwareSerial& serial, uint8_t dePin) {
  _hardwareSerial = &serial;
  #ifdef __AVR__
  _softwareSerial = 0;
  #endif
  #ifdef HAVE_CDCSERIAL
  _usbSerial = 0;
  #endif
  _serial = &serial;
  _coils = 0;
  _discreteInputs = 0;
  _holdingRegisters = 0;
  _inputRegisters = 0;
  _dePin = dePin;
}

#ifdef __AVR__
ModbusRTUSlave::ModbusRTUSlave(SoftwareSerial& serial, uint8_t dePin) {
  _hardwareSerial = 0;
  _softwareSerial = &serial;
  #ifdef HAVE_CDCSERIAL
  _usbSerial = 0;
  #endif
  _serial = &serial;
  _coils = 0;
  _discreteInputs = 0;
  _holdingRegisters = 0;
  _inputRegisters = 0;
  _dePin = dePin;
}
#endif

#ifdef HAVE_CDCSERIAL
ModbusRTUSlave::ModbusRTUSlave(Serial_& serial, uint8_t dePin) {
  _hardwareSerial = 0;
  #ifdef __AVR__
  _softwareSerial = 0;
  #endif
  _usbSerial = &serial;
  _serial = &serial;
  _coils = 0;
  _discreteInputs = 0;
  _holdingRegisters = 0;
  _inputRegisters = 0;
  _dePin = dePin;
}
#endif

void ModbusRTUSlave::configureCoils(bool coils[], uint16_t numCoils) {
  _coils = coils;
  _numCoils = numCoils;
}

void ModbusRTUSlave::configureDiscreteInputs(bool discreteInputs[], uint16_t numDiscreteInputs) {
  _discreteInputs = discreteInputs;
  _numDiscreteInputs = numDiscreteInputs;
}

void ModbusRTUSlave::configureHoldingRegisters(uint16_t holdingRegisters[], uint16_t numHoldingRegisters) {
  _holdingRegisters = holdingRegisters;
  _numHoldingRegisters = numHoldingRegisters;
}

void ModbusRTUSlave::configureInputRegisters(uint16_t inputRegisters[], uint16_t numInputRegisters) {
  _inputRegisters = inputRegisters;
  _numInputRegisters = numInputRegisters;
}

void ModbusRTUSlave::begin(uint8_t id, uint32_t baud, uint16_t config) {
  if (id >= 1 && id <= 247) _id = id;
  else _id = NO_ID;
  _baud = (baud > 0) ? baud : 9600;
  _rxLen = 0;

  if (_hardwareSerial) {
    _calculateTimeouts(_baud, config);
    _hardwareSerial->begin(_baud, config);
  }
  #ifdef __AVR__
  else if (_softwareSerial) {
    _calculateTimeouts(_baud, SERIAL_8N1);
    _softwareSerial->begin(_baud);
  }
  #endif
  #ifdef HAVE_CDCSERIAL
  else if (_usbSerial) {
    _calculateTimeouts(_baud, config);
    _usbSerial->begin(_baud, config);
    while (!_usbSerial);
  }
  #endif
  if (_dePin != NO_DE_PIN) {
    pinMode(_dePin, OUTPUT);
    digitalWrite(_dePin, LOW);
  }
  uint32_t startTime = micros();
  do {
    if (_serial->available()) {
      startTime = micros();
      _serial->read();
    }
  } while ((uint32_t)(micros() - startTime) < _frameTimeout);
  _lastRxTime = micros();
}

int ModbusRTUSlave::poll() {
  int ret_val = 0;
  if (_serial->available() || _rxLen > 0) {
    if (_readRequest()) {
      switch (_buf[1]) {
        case 1:
          _processReadCoils();
          ret_val = 1;
          break;
        case 2:
          _processReadDiscreteInputs();
          ret_val = 2;
          break;
        case 3:
          _processReadHoldingRegisters();
          ret_val = 3;
          break;
        case 4:
          _processReadInputRegisters();
          ret_val = 4;
          break;
        case 5:
          _processWriteSingleCoil();
          ret_val = 5;
          break;
        case 6:
          _processWriteSingleHoldingRegister();
          ret_val = 6;
          break;
        case 15:
          _processWriteMultipleCoils();
          ret_val = 15;
          break;
        case 16:
          _processWriteMultipleHoldingRegisters();
          ret_val = 16;
          break;
        default:
          _exceptionResponse(1);
          break;
      }
    }
  }
  return ret_val;
}

void ModbusRTUSlave::_processReadCoils() {
  uint16_t startAddress = _bytesToWord(_buf[2], _buf[3]);
  uint16_t quantity = _bytesToWord(_buf[4], _buf[5]);
  if (!_coils || _numCoils == 0) _exceptionResponse(1);
  else if (quantity == 0 || quantity > 2000) _exceptionResponse(3);
  else if (quantity > _numCoils || startAddress > (_numCoils - quantity)) _exceptionResponse(2);
  else {
    _buf[2] = _div8RndUp(quantity);
    memset(&_buf[3], 0, _buf[2]);
    for (uint16_t i = 0; i < quantity; i++) {
      bitWrite(_buf[3 + (i >> 3)], i & 7, _coils[startAddress + i]);
    }
    _writeResponse(3 + _buf[2]);
  }
}

void ModbusRTUSlave::_processReadDiscreteInputs() {
  uint16_t startAddress = _bytesToWord(_buf[2], _buf[3]);
  uint16_t quantity = _bytesToWord(_buf[4], _buf[5]);
  if (!_discreteInputs || _numDiscreteInputs == 0) _exceptionResponse(1);
  else if (quantity == 0 || quantity > 2000) _exceptionResponse(3);
  else if (quantity > _numDiscreteInputs || startAddress > (_numDiscreteInputs - quantity)) _exceptionResponse(2);
  else {
    _buf[2] = _div8RndUp(quantity);
    memset(&_buf[3], 0, _buf[2]);
    for (uint16_t i = 0; i < quantity; i++) {
      bitWrite(_buf[3 + (i >> 3)], i & 7, _discreteInputs[startAddress + i]);
    }
    _writeResponse(3 + _buf[2]);
  }
}

void ModbusRTUSlave::_processReadHoldingRegisters() {
  uint16_t startAddress = _bytesToWord(_buf[2], _buf[3]);
  uint16_t quantity = _bytesToWord(_buf[4], _buf[5]);
  if (!_holdingRegisters || _numHoldingRegisters == 0) _exceptionResponse(1);
  else if (quantity == 0 || quantity > 125) _exceptionResponse(3);
  else if (quantity > _numHoldingRegisters || startAddress > (_numHoldingRegisters - quantity)) _exceptionResponse(2);
  else {
    _buf[2] = quantity * 2;
    for (uint16_t i = 0; i < quantity; i++) {
      _buf[3 + (i * 2)] = highByte(_holdingRegisters[startAddress + i]);
      _buf[4 + (i * 2)] = lowByte(_holdingRegisters[startAddress + i]);
    }
    _writeResponse(3 + _buf[2]);
  }
}

void ModbusRTUSlave::_processReadInputRegisters() {
  uint16_t startAddress = _bytesToWord(_buf[2], _buf[3]);
  uint16_t quantity = _bytesToWord(_buf[4], _buf[5]);
  if (!_inputRegisters || _numInputRegisters == 0) _exceptionResponse(1);
  else if (quantity == 0 || quantity > 125) _exceptionResponse(3);
  else if (quantity > _numInputRegisters || startAddress > (_numInputRegisters - quantity)) _exceptionResponse(2);
  else {
    _buf[2] = quantity * 2;
    for (uint16_t i = 0; i < quantity; i++) {
      _buf[3 + (i * 2)] = highByte(_inputRegisters[startAddress + i]);
      _buf[4 + (i * 2)] = lowByte(_inputRegisters[startAddress + i]);
    }
    _writeResponse(3 + _buf[2]);
  }
}

void ModbusRTUSlave::_processWriteSingleCoil() {
  uint16_t address = _bytesToWord(_buf[2], _buf[3]);
  uint16_t value = _bytesToWord(_buf[4], _buf[5]);
  if (!_coils || _numCoils == 0) _exceptionResponse(1);
  else if (value != 0 && value != 0xFF00) _exceptionResponse(3);
  else if (address >= _numCoils) _exceptionResponse(2);
  else {
    _coils[address] = value;
    _writeResponse(6);
  }
}

void ModbusRTUSlave::_processWriteSingleHoldingRegister() {
  uint16_t address = _bytesToWord(_buf[2], _buf[3]);
  uint16_t value = _bytesToWord(_buf[4], _buf[5]);
  if (!_holdingRegisters || _numHoldingRegisters == 0) _exceptionResponse(1);
  else if (address >= _numHoldingRegisters) _exceptionResponse(2);
  else {
    _holdingRegisters[address] = value;
    _writeResponse(6);
  }
}

void ModbusRTUSlave::_processWriteMultipleCoils() {
  uint16_t startAddress = _bytesToWord(_buf[2], _buf[3]);
  uint16_t quantity = _bytesToWord(_buf[4], _buf[5]);
  if (!_coils || _numCoils == 0) _exceptionResponse(1);
  else if (quantity == 0 || quantity > 1968 || _buf[6] != _div8RndUp(quantity)) _exceptionResponse(3);
  else if (quantity > _numCoils || startAddress > (_numCoils - quantity)) _exceptionResponse(2);
  else {
    for (uint16_t i = 0; i < quantity; i++) {
      _coils[startAddress + i] = bitRead(_buf[7 + (i >> 3)], i & 7);
    }
    _writeResponse(6);
  }
}

void ModbusRTUSlave::_processWriteMultipleHoldingRegisters() {
  uint16_t startAddress = _bytesToWord(_buf[2], _buf[3]);
  uint16_t quantity = _bytesToWord(_buf[4], _buf[5]);
  if (!_holdingRegisters || _numHoldingRegisters == 0) _exceptionResponse(1);
  else if (quantity == 0 || quantity > 123 || _buf[6] != (quantity * 2)) _exceptionResponse(3);
  else if (quantity > _numHoldingRegisters || startAddress > (_numHoldingRegisters - quantity)) _exceptionResponse(2);
  else {
    for (uint16_t i = 0; i < quantity; i++) {
      _holdingRegisters[startAddress + i] = _bytesToWord(_buf[i * 2 + 7], _buf[i * 2 + 8]);
    }
    _writeResponse(6);
  }
}

void ModbusRTUSlave::_consumeRx(uint16_t count) {
  if (count == 0) return;
  if (count >= _rxLen) {
    _rxLen = 0;
  } else {
    memmove(_rxBuf, _rxBuf + count, _rxLen - count);
    _rxLen -= count;
  }
}

bool ModbusRTUSlave::_isSupportedFc(uint8_t fc) {
  return (fc >= 1 && fc <= 6) || fc == 15 || fc == 16;
}

uint16_t ModbusRTUSlave::_reqLenForFc(const uint8_t *buf, uint16_t avail) {
  if (avail < 2) return 0;
  uint8_t fc = buf[1];
  if (fc >= 1 && fc <= 6) {
    return 8;
  }
  if (fc == 15 || fc == 16) {
    if (avail < 7) return 0;
    uint16_t bc = buf[6];
    if (bc == 0 || (uint16_t)(9 + bc) > MODBUS_RTU_SLAVE_BUF_SIZE) {
      return 0xFFFF;
    }
    return 9 + bc;
  }
  return 0xFFFF;
}

bool ModbusRTUSlave::_hasValidAddressedFrameFrom(uint16_t startIdx) const {
  if (startIdx >= _rxLen) return false;
  for (uint16_t i = startIdx; i + 8 <= _rxLen; i++) {
    uint8_t addr = _rxBuf[i];
    uint8_t fc = _rxBuf[i + 1];
    bool isBroadcastWrite = (addr == 0) && (fc == 5 || fc == 6 || fc == 15 || fc == 16);
    if (!((_id != NO_ID && addr == _id) || isBroadcastWrite)) {
      continue;
    }
    uint16_t avail = _rxLen - i;
    uint16_t expectedLen = _reqLenForFc(&_rxBuf[i], avail);
    if (expectedLen == 0 || expectedLen == 0xFFFF || avail < expectedLen) {
      continue;
    }
    uint16_t rxCrc = ((uint16_t)_rxBuf[i + expectedLen - 1] << 8) | _rxBuf[i + expectedLen - 2];
    if (_crcBuf(&_rxBuf[i], expectedLen - 2) == rxCrc) {
      return true;
    }
  }
  return false;
}

bool ModbusRTUSlave::_readRequest() {
  while (_serial->available() && _rxLen < MODBUS_RTU_SLAVE_BUF_SIZE) {
    int b = _serial->read();
    if (b < 0) break;
    _rxBuf[_rxLen++] = (uint8_t)b;
    _lastRxTime = micros();
  }
  if (_rxLen == 0) return false;

  uint32_t now = micros();
  bool gapElapsed = (uint32_t)(now - _lastRxTime) >= _frameTimeout;

  while (_rxLen > 0) {
    // Pull any remaining UART bytes once a frame shift frees space in _rxBuf
    while (_serial->available() && _rxLen < MODBUS_RTU_SLAVE_BUF_SIZE) {
      int b = _serial->read();
      if (b < 0) break;
      _rxBuf[_rxLen++] = (uint8_t)b;
      _lastRxTime = micros();
      gapElapsed = false;
    }

    uint8_t addr = _rxBuf[0];
    // Slave addresses 248..255 are reserved/invalid in Modbus RTU
    if (addr > 247) {
      _consumeRx(1);
      continue;
    }

    if (_rxLen < 2) {
      if (gapElapsed) {
        _consumeRx(1);
        continue;
      }
      return false;
    }

    uint8_t fc = _rxBuf[1];
    bool isBroadcastWrite = (addr == 0) && (fc == 5 || fc == 6 || fc == 15 || fc == 16);

    // --- Addressed to this slave (_id != 0) or broadcast write ---
    if ((_id != NO_ID && addr == _id) || isBroadcastWrite) {
      uint16_t expectedLen = _reqLenForFc(_rxBuf, _rxLen);

      if (expectedLen != 0 && expectedLen != 0xFFFF) {
        if (_rxLen >= expectedLen) {
          uint16_t rxCrc = _bytesToWord(_rxBuf[expectedLen - 1], _rxBuf[expectedLen - 2]);
          if (_crcBuf(_rxBuf, expectedLen - 2) == rxCrc) {
            memcpy(_buf, _rxBuf, expectedLen);
            _consumeRx(expectedLen);
            return true;
          }

          // Check if our own echoed response slipped into RX (e.g., if /RE was enabled during TX)
          if (addr == _id && fc >= 1 && fc <= 4 && _rxLen >= 5 && _rxBuf[2] >= 1) {
            uint16_t echoLen = (uint16_t)5 + _rxBuf[2];
            if (echoLen <= MODBUS_RTU_SLAVE_BUF_SIZE && _rxLen >= echoLen) {
              uint16_t echoCrc = _bytesToWord(_rxBuf[echoLen - 1], _rxBuf[echoLen - 2]);
              if (_crcBuf(_rxBuf, echoLen - 2) == echoCrc) {
                _consumeRx(echoLen);
                continue;
              }
            }
          }
          if (addr == _id && (fc == 15 || fc == 16) && _rxLen >= 8) {
            uint16_t echoCrc = _bytesToWord(_rxBuf[7], _rxBuf[6]);
            if (_crcBuf(_rxBuf, 6) == echoCrc) {
              _consumeRx(8);
              continue;
            }
          }

          _consumeRx(1);
          continue;
        }

        if (gapElapsed) {
          if (addr == _id && fc >= 1 && fc <= 4 && _rxLen >= 5 && _rxBuf[2] >= 1) {
            uint16_t echoLen = (uint16_t)5 + _rxBuf[2];
            if (echoLen <= _rxLen) {
              uint16_t echoCrc = _bytesToWord(_rxBuf[echoLen - 1], _rxBuf[echoLen - 2]);
              if (_crcBuf(_rxBuf, echoLen - 2) == echoCrc) {
                _consumeRx(echoLen);
                continue;
              }
            }
          }
          _consumeRx(1);
          continue;
        }
        return false;
      }

      if (expectedLen == 0) {
        if (!gapElapsed && _rxLen < MODBUS_RTU_SLAVE_BUF_SIZE) {
          return false;
        }
        _consumeRx(1);
        continue;
      }

      // expectedLen == 0xFFFF: echoed exception, echoed FC 15/16 reply with unusual payload bytes,
      // invalid FC 15/16 byte count, or unsupported FC addressed to us
      if (addr == _id && (fc & 0x80) != 0 && _rxLen >= 5) {
        uint16_t exCrc = _bytesToWord(_rxBuf[4], _rxBuf[3]);
        if (_crcBuf(_rxBuf, 3) == exCrc) {
          _consumeRx(5);
          continue;
        }
      }
      if (addr == _id && (fc == 15 || fc == 16) && _rxLen >= 8) {
        uint16_t echoCrc = _bytesToWord(_rxBuf[7], _rxBuf[6]);
        if (_crcBuf(_rxBuf, 6) == echoCrc) {
          _consumeRx(8);
          continue;
        }
      }
      if (addr == _id && !_isSupportedFc(fc) && (fc & 0x80) == 0) {
        if (!gapElapsed && _rxLen < 8) {
          return false;
        }
        if (gapElapsed && _rxLen >= 4 && _rxLen <= 8) {
          uint16_t rxCrc = _bytesToWord(_rxBuf[_rxLen - 1], _rxBuf[_rxLen - 2]);
          if (_crcBuf(_rxBuf, _rxLen - 2) == rxCrc) {
            uint16_t flen = _rxLen;
            memcpy(_buf, _rxBuf, flen);
            _consumeRx(flen);
            return true;
          }
        }
      }
      _consumeRx(1);
      continue;
    }

    // --- Foreign frame (or invalid broadcast non-write): strip cleanly without discarding trailing frames ---
    bool stripped = false;
    bool canWaitMore = false;

    if (addr != 0) {
      if ((fc & 0x80) != 0) {
        if (_rxLen >= 5) {
          if (_crcBuf(_rxBuf, 3) == _bytesToWord(_rxBuf[4], _rxBuf[3])) {
            _consumeRx(5);
            stripped = true;
          }
        } else {
          canWaitMore = true;
        }
      } else if (fc >= 1 && fc <= 4) {
        uint16_t replyLen = 0;
        if (_rxLen >= 3 && _rxBuf[2] >= 1 && (uint16_t)(5 + _rxBuf[2]) <= MODBUS_RTU_SLAVE_BUF_SIZE) {
          replyLen = (uint16_t)5 + _rxBuf[2];
        }
        if (replyLen > 0 && _rxLen >= replyLen) {
          if (_crcBuf(_rxBuf, replyLen - 2) == _bytesToWord(_rxBuf[replyLen - 1], _rxBuf[replyLen - 2])) {
            _consumeRx(replyLen);
            stripped = true;
          }
        }
        if (!stripped && _rxLen >= 8) {
          if (_crcBuf(_rxBuf, 6) == _bytesToWord(_rxBuf[7], _rxBuf[6])) {
            _consumeRx(8);
            stripped = true;
          }
        }
        if (!stripped) {
          uint16_t maxNeed = (replyLen > 8) ? replyLen : 8;
          if (_rxLen < maxNeed) canWaitMore = true;
        }
      } else if (fc == 5 || fc == 6) {
        if (_rxLen >= 8) {
          if (_crcBuf(_rxBuf, 6) == _bytesToWord(_rxBuf[7], _rxBuf[6])) {
            _consumeRx(8);
            stripped = true;
          }
        } else {
          canWaitMore = true;
        }
      } else if (fc == 15 || fc == 16) {
        if (_rxLen >= 8) {
          if (_crcBuf(_rxBuf, 6) == _bytesToWord(_rxBuf[7], _rxBuf[6])) {
            _consumeRx(8);
            stripped = true;
          }
        }
        uint16_t reqLen = 0;
        if (!stripped && _rxLen >= 7 && _rxBuf[6] >= 1 && (uint16_t)(9 + _rxBuf[6]) <= MODBUS_RTU_SLAVE_BUF_SIZE) {
          reqLen = (uint16_t)9 + _rxBuf[6];
          if (_rxLen >= reqLen) {
            if (_crcBuf(_rxBuf, reqLen - 2) == _bytesToWord(_rxBuf[reqLen - 1], _rxBuf[reqLen - 2])) {
              _consumeRx(reqLen);
              stripped = true;
            }
          }
        }
        if (!stripped) {
          if (_rxLen < 8 || (reqLen > 0 && _rxLen < reqLen)) canWaitMore = true;
        }
      } else {
        // Unsupported foreign function code: never wait on FC 0 (invalid)
        if (fc != 0 && _rxLen < 32 && !gapElapsed) {
          canWaitMore = true;
        }
      }
    } else if (fc >= 1 && fc <= 4) {
      // Broadcast read (illegal in Modbus RTU): if complete and CRC-valid, strip all 8 bytes atomically.
      if (_rxLen >= 8 && _crcBuf(_rxBuf, 6) == _bytesToWord(_rxBuf[7], _rxBuf[6])) {
        _consumeRx(8);
        stripped = true;
      }
    }

    if (stripped) {
      continue;
    }
    if (canWaitMore && !gapElapsed && _rxLen < MODBUS_RTU_SLAVE_BUF_SIZE &&
        !_hasValidAddressedFrameFrom(1)) {
      return false;
    }
    _consumeRx(1);
  }
  return false;
}

void ModbusRTUSlave::_writeResponse(uint8_t len) {
  if (_buf[0] != 0) {
    // Enforce >= t3.5 inter-frame idle time since the last received byte so the
    // master has finished releasing DE before we assert DE on the half-duplex bus.
    while ((uint32_t)(micros() - _lastRxTime) < _frameTimeout) {
    }

    uint16_t crc = _crc(len);
    _buf[len] = lowByte(crc);
    _buf[len + 1] = highByte(crc);
    if (_dePin != NO_DE_PIN) digitalWrite(_dePin, HIGH);
    _serial->write(_buf, len + 2);
    if (_hardwareSerial) {
      _hardwareSerial->flush();
    }
    #ifdef HAVE_CDCSERIAL
    else if (_usbSerial) {
      _usbSerial->flush();
    }
    #endif
    // Work around Adafruit/SAMD SERCOM::flushUART() returning once DRE is empty
    // while the final byte (~1 char + stop bit) is still shifting out on TX.
    uint32_t tailUs = (_baud > 0) ? (18000000UL / _baud) : 1875UL;
    if (tailUs < 100UL) tailUs = 100UL;
    if (tailUs > 5000UL) tailUs = 5000UL;
    delayMicroseconds(tailUs);

    // Drain any local TX echo before releasing DE so subsequent foreign/master
    // frames arriving right after DE goes LOW are preserved.
    while (_serial->available()) {
      _serial->read();
    }

    if (_dePin != NO_DE_PIN) digitalWrite(_dePin, LOW);
  }
}

void ModbusRTUSlave::_exceptionResponse(uint8_t code) {
  _buf[1] |= 0x80;
  _buf[2] = code;
  _writeResponse(3);
}

void ModbusRTUSlave::_calculateTimeouts(uint32_t baud, uint16_t config) {
  if (baud == 0) baud = 9600;
  uint32_t bitsPerChar;
  if (config == SERIAL_8E2 || config == SERIAL_8O2) bitsPerChar = 12;
  else if (config == SERIAL_8N2 || config == SERIAL_8E1 || config == SERIAL_8O1) bitsPerChar = 11;
  else bitsPerChar = 10;
  if (baud <= 19200) {
    _charTimeout = (bitsPerChar * 2500000UL) / baud;
    _frameTimeout = (bitsPerChar * 4500000UL) / baud;
  }
  else {
    _charTimeout = (bitsPerChar * 1000000UL) / baud + 750UL;
    _frameTimeout = (bitsPerChar * 1000000UL) / baud + 1750UL;
  }
}

uint16_t ModbusRTUSlave::_crcBuf(const uint8_t *buf, uint16_t len) {
  uint16_t value = 0xFFFF;
  for (uint16_t i = 0; i < len; i++) {
    value ^= (uint16_t)buf[i];
    for (uint16_t j = 0; j < 8; j++) {
      bool lsb = value & 1;
      value >>= 1;
      if (lsb) value ^= 0xA001;
    }
  }
  return value;
}

uint16_t ModbusRTUSlave::_crc(uint16_t len) {
  return _crcBuf(_buf, len);
}

uint16_t ModbusRTUSlave::_div8RndUp(uint16_t value) {
  return (value + 7) >> 3;
}

uint16_t ModbusRTUSlave::_bytesToWord(uint8_t high, uint8_t low) {
  return ((uint16_t)high << 8) | low;
}
