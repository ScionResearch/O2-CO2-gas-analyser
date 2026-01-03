/**************************************************************************************************
 * PID_Tune - PID Tuning Interface Library
 * Implementation
 **************************************************************************************************/

#include "PID_Tune.h"
#include <ArduinoJson.h>
#include <HardwareSerial.h>

PID_Tune::PID_Tune(PID_Control& pid) : _pid(pid) {
    _enabled = false;
    _running = false;
    _stepTestActive = false;
    _stepTestAmplitude = 10.0;
    _originalSetpoint = 0.0;
    _stepTestStartTime = 0;
    _lastDataSend = 0;
    _loopPeriod = 100;
    _bufferIndex = 0;
    _buffer[0] = '\0';
    _serial = nullptr;  // No serial port assigned yet
}

// Implementation for Generic Stream (Assumes already initialized)
void PID_Tune::begin(Stream& serial) {
    // This is the fallback for any Stream object (like USB Serial). 
    // It assumes the object is ready or that initialization is handled elsewhere.
    _serial = &serial;
    
    _enabled = true;
    sendStatus();
    _serial->println("PID Tuning Interface Ready (Generic Stream)");
}


// Implementation for HardwareSerial (Calls begin(baudRate))
void PID_Tune::begin(HardwareSerial& serial, unsigned long baudRate) {
    _serial = &serial;
    
    // Call the specific HardwareSerial method
    serial.begin(baudRate);
    
    // Wait for the serial port to be ready
    while (!serial && millis() < 3000) delay(10);
    
    _enabled = true;
    sendStatus();
    _serial->println("PID Tuning Interface Ready (HardwareSerial)");
}

void PID_Tune::setSensorCallback(SensorCallback callback) {
    _sensorCallback = callback;
}

void PID_Tune::update() {
    if (!_enabled || !_serial) return;
    
    // Check for incoming commands
    while (_serial->available()) {
        char c = _serial->read();
        if (c == '\n' || c == '\r') {
            if (_bufferIndex > 0) {
                _buffer[_bufferIndex] = '\0';
                processCommand();
                clearBuffer();
            }
        } else if (_bufferIndex < PID_TUNE_BUFFER_SIZE - 1) {
            _buffer[_bufferIndex++] = c;
        }
    }
    
    // Send data at fixed rate
    unsigned long now = millis();
    if (now - _lastDataSend >= DATA_SEND_INTERVAL) {
        sendData();
        _lastDataSend = now;
    }
    
    // Handle step test timing
    if (_stepTestActive && (now - _stepTestStartTime >= 5000)) {
        stopStepTest();
    }
}

void PID_Tune::enable() {
    _enabled = true;
}

void PID_Tune::disable() {
    _enabled = false;
}

bool PID_Tune::isEnabled() {
    return _enabled;
}

void PID_Tune::setSetpoint(float setpoint) {
    _pid.setpoint(setpoint);
}

float PID_Tune::getSetpoint() {
    return _pid.getSetpoint();
}

void PID_Tune::setPID(float Kp, float Ki, float Kd) {
    _pid.setPID(Kp, Ki, Kd);
}

float PID_Tune::getKp() {
    return _pid.getKp();
}

float PID_Tune::getKi() {
    return _pid.getKi();
}

float PID_Tune::getKd() {
    return _pid.getKd();
}

void PID_Tune::setLoopPeriod(unsigned long periodMs) {
    _loopPeriod = periodMs;
    _pid.setSampleTime(periodMs);
}

unsigned long PID_Tune::getLoopPeriod() {
    return _loopPeriod;
}

void PID_Tune::setOutputLimits(float min, float max) {
    _pid.setOutputLimits(min, max);
}

void PID_Tune::setIntegralLimits(float min, float max) {
    _pid.setIntegralLimits(min, max);
}

void PID_Tune::startStepTest(float amplitude) {
    if (!_stepTestActive) {
        _stepTestAmplitude = amplitude;
        _originalSetpoint = _pid.getSetpoint();
        _pid.setpoint(_originalSetpoint + amplitude);
        _stepTestActive = true;
        _stepTestStartTime = millis();
        
        // Notify the Python app
        _serial->print("{\"type\": \"step_test_started\", \"amplitude\": ");
        _serial->print(_stepTestAmplitude, 2);
        _serial->println("}");
    }
}

void PID_Tune::stopStepTest() {
    if (_stepTestActive) {
        _pid.setpoint(_originalSetpoint);
        _stepTestActive = false;
        _serial->println("{\"type\": \"step_test_complete\"}");
    }
}

bool PID_Tune::isStepTestActive() {
    return _stepTestActive;
}

void PID_Tune::start() {
    _running = true;
    _pid.enable();  // Enable PID when starting
    if (_serial) {
        _serial->println("{\"type\": \"debug\", \"debug\": \"Control started\"}");
        sendStatus();  // Send status update
    }
}

void PID_Tune::stop() {
    _running = false;
    _pid.disable();  // Disable PID to force output to 0
    if (_serial) {
        _serial->println("{\"type\": \"debug\", \"debug\": \"Control stopped\"}");
        sendStatus();  // Send status update
    }
}

bool PID_Tune::isRunning() {
    return _running;
}

float PID_Tune::getProcessValue() {
    return readSensor();
}

float PID_Tune::getOutput() {
    return _pid.getOutput();
}

float PID_Tune::getError() {
    return _pid.getError();
}

float PID_Tune::getProportional() {
    return _pid.getProportional();
}

float PID_Tune::getIntegral() {
    return _pid.getIntegral();
}

float PID_Tune::getDerivative() {
    return _pid.getDerivative();
}

// Private methods

void PID_Tune::processCommand() {
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, _buffer);
    
    if (error) {
        _serial->println("{\"error\": \"Invalid JSON\"}");
        return;
    }
    
    String cmd = doc["cmd"];
    
    if (cmd == "set_params") {
        // Handle PID parameters - need to set all three at once
        float kp = _pid.getKp();
        float ki = _pid.getKi();
        float kd = _pid.getKd();
        
        if (doc["kp"].is<float>()) kp = doc["kp"];
        if (doc["ki"].is<float>()) ki = doc["ki"];
        if (doc["kd"].is<float>()) kd = doc["kd"];
        
        _pid.setPID(kp, ki, kd);
        
        if (doc["loop_period"].is<unsigned long>()) setLoopPeriod(doc["loop_period"]);
        
        // Handle anti-windup settings
        if (doc["output_limit"].is<bool>()) {
            if (doc["output_limit"]) {
                if (doc["output_min"].is<float>() && doc["output_max"].is<float>()) {
                    setOutputLimits(doc["output_min"], doc["output_max"]);
                }
            }
        }
        
        if (doc["integral_limit"].is<bool>()) {
            if (doc["integral_limit"]) {
                if (doc["integral_min"].is<float>() && doc["integral_max"].is<float>()) {
                    setIntegralLimits(doc["integral_min"], doc["integral_max"]);
                }
            }
        }
    }
    else if (cmd == "set_sp") {
        if (doc["value"].is<float>()) {
            setSetpoint(doc["value"]);
        }
    }
    else if (cmd == "start") {
        _serial->println("{\"type\": \"debug\", \"debug\": \"Received start command\"}");
        start();
    }
    else if (cmd == "stop") {
        _serial->println("{\"type\": \"debug\", \"debug\": \"Received stop command\"}");
        stop();
    }
    else if (cmd == "get_status") {
        sendStatus();
    }
    else if (cmd == "step_test") {
        if (doc["amplitude"].is<float>()) {
            startStepTest(doc["amplitude"]);
        }
    }
    else {
        _serial->println("{\"error\": \"Unknown command\"}");
    }
}

void PID_Tune::sendData() {
    if (!_serial) return;
    
    float pv = readSensor();
    float sp = _pid.getSetpoint();
    float output = _pid.getOutput();
    float error = sp - pv;
    unsigned long time = millis();
    
    // Get PID components
    float P = _pid.getProportional();
    float I = _pid.getIntegral();
    float D = _pid.getDerivative();
    
    // Send data JSON
    _serial->print("{\"type\": \"data\", ");
    _serial->print("\"pv\": " + String(pv, 2) + ", ");
    _serial->print("\"sp\": " + String(sp, 2) + ", ");
    _serial->print("\"output\": " + String(output, 0) + ", ");
    _serial->print("\"error\": " + String(error, 2) + ", ");
    _serial->print("\"P\": " + String(P, 2) + ", ");
    _serial->print("\"I\": " + String(I, 2) + ", ");
    _serial->print("\"D\": " + String(D, 2) + ", ");
    _serial->println("\"time\": " + String(time) + "}");
}

void PID_Tune::sendStatus() {
    if (!_serial) return;
    
    _serial->print("{\"type\": \"status\", ");
    _serial->print("\"running\": " + String(_running ? "true" : "false") + ", ");
    _serial->print("\"kp\": " + String(_pid.getKp(), 3) + ", ");
    _serial->print("\"ki\": " + String(_pid.getKi(), 4) + ", ");
    _serial->print("\"kd\": " + String(_pid.getKd(), 4) + ", ");
    _serial->print("\"sp\": " + String(_pid.getSetpoint(), 2) + ", ");
    _serial->print("\"loop_period\": " + String(_loopPeriod));
    _serial->println("}");
}

float PID_Tune::readSensor() {
    if (_sensorCallback) {
        return _sensorCallback();
    }
    // Return 0 if no callback is set
    return 0.0;
}

void PID_Tune::clearBuffer() {
    _bufferIndex = 0;
    _buffer[0] = '\0';
}
