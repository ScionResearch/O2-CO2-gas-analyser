/**************************************************************************************************
 * PID_Tune - PID Tuning Interface Library
 * 
 * Provides a complete PID tuning interface that can be integrated into any Arduino project.
 * Works with the PID_Control library for the actual PID algorithm.
 * 
 * Features:
 * - Serial communication with Python tuning app
 * - JSON protocol for robust data exchange
 * - Step response testing
 * - Real-time parameter adjustment
 * - Callback function for sensor reading
 * - Configurable serial port (Serial, Serial1, Serial2, etc.)
 **************************************************************************************************/

#pragma once

#include <Arduino.h>
#include <PID_Control.h>
#include <functional>
#include <HardwareSerial.h>

#ifndef PID_TUNE_BUFFER_SIZE
#define PID_TUNE_BUFFER_SIZE 256
#endif

class PID_Tune {
    public:
        // Callback function type for sensor reading
        using SensorCallback = std::function<float()>;
        
        // Constructor
        PID_Tune(PID_Control& pid);
        
        // Initialize the tuning interface (any Stream)
        void begin(Stream& serial);
        
        // Initialize with specific serial port
        void begin(HardwareSerial& serial, unsigned long baudRate = 115200);
        
        // Set the sensor reading callback function
        void setSensorCallback(SensorCallback callback);
        
        // Main update function - call this in your loop()
        void update();
        
        // Enable/disable the tuning interface
        void enable();
        void disable();
        bool isEnabled();
        
        // Setpoint control
        void setSetpoint(float setpoint);
        float getSetpoint();
        
        // PID parameter access
        void setPID(float Kp, float Ki, float Kd);
        float getKp();
        float getKi();
        float getKd();
        
        // Control loop period
        void setLoopPeriod(unsigned long periodMs);
        unsigned long getLoopPeriod();
        
        // Output and integral limits
        void setOutputLimits(float min, float max);
        void setIntegralLimits(float min, float max);
        
        // Step test control
        void startStepTest(float amplitude);
        void stopStepTest();
        bool isStepTestActive();
        
        // Manual control
        void start();
        void stop();
        bool isRunning();
        
        // Get current values
        float getProcessValue();
        float getOutput();
        float getError();
        
        // PID components for debugging
        float getProportional();
        float getIntegral();
        float getDerivative();
        
    private:
        PID_Control& _pid;  // Reference to the PID controller
        SensorCallback _sensorCallback;
        Stream* _serial;  // Pointer to the serial port
        
        // State variables
        bool _enabled;
        bool _running;
        bool _stepTestActive;
        float _stepTestAmplitude;
        float _originalSetpoint;
        unsigned long _stepTestStartTime;
        
        // Timing
        unsigned long _lastDataSend;
        unsigned long _loopPeriod;
        static const unsigned long DATA_SEND_INTERVAL = 100;  // Send data at 10Hz
        
        // Serial communication
        char _buffer[PID_TUNE_BUFFER_SIZE];
        int _bufferIndex;
        
        // Command processing
        void processCommand();
        void processSetParams();
        void processSetSP();
        void processStart();
        void processStop();
        void processGetStatus();
        void processStepTest();
        
        // Data transmission
        void sendData();
        void sendStatus();
        void sendDebug();
        
        // Helper functions
        float readSensor();
        void clearBuffer();
};
