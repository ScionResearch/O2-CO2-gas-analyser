/**************************************************************************************************
 * PID_Tune Example Sketch
 * 
 * This example shows how to integrate the PID_Tune library into your own project.
 * The tuning interface runs in parallel with your normal control code.
 **************************************************************************************************/

#include <PID_Control.h>
#include <PID_Tune.h>

// =========================================================================
// CONFIGURATION
// =========================================================================

#define SENSOR_PIN A0              // Analog input pin for your sensor
#define OUTPUT_PIN 3               // PWM output pin for your actuator
#define CONTROL_LOOP_PERIOD 100    // PID update time (ms)

// Serial port configuration
// Uncomment the appropriate serial port for your board:
#define TUNING_SERIAL Serial        // Most Arduino boards (Uno, Nano, etc.)
//#define TUNING_SERIAL Serial1      // Arduino Leonardo, Mega (hardware serial 1)
//#define TUNING_SERIAL Serial2      // Arduino Mega (hardware serial 2)
//#define TUNING_SERIAL Serial3      // Arduino Mega (hardware serial 3)

// Sensor calibration (adjust for your sensor)
#define SENSOR_MIN 0.0             // Minimum raw value
#define SENSOR_MAX 1023.0          // Maximum raw value
#define VALUE_MIN 0.0              // Minimum measured value
#define VALUE_MAX 100.0            // Maximum measured value

// =========================================================================
// GLOBAL OBJECTS
// =========================================================================

// Create PID controller
PID_Control pid(OUTPUT_PIN, true);

// Create tuning interface, linked to the PID controller
PID_Tune tuner(pid);

// =========================================================================
// SENSOR READING FUNCTION
// =========================================================================

// This function reads your sensor and returns the measured value
// The PID_Tune library will call this function automatically
float readMySensor() {
    // Read the raw analog value
    int rawValue = analogRead(SENSOR_PIN);
    
    // Convert to engineering units (adjust calibration as needed)
    float measuredValue = map(rawValue, SENSOR_MIN, SENSOR_MAX, VALUE_MIN, VALUE_MAX);
    
    return measuredValue;
}

// =========================================================================
// SETUP
// =========================================================================

void setup() {
    // Initialize the PID controller
    pid.begin(2.0, 0.1, 0.05, 25.0);  // Kp, Ki, Kd, setpoint
    pid.setOutputLimits(0, 255);
    pid.setSampleTime(CONTROL_LOOP_PERIOD);
    
    // Initialize the tuning interface with specified serial port
    tuner.begin(TUNING_SERIAL, 115200);  // Use configured serial port
    
    // Attach your sensor reading function
    tuner.setSensorCallback(readMySensor);
    
    // Enable the tuning interface (but don't start control)
    tuner.enable();
    // Control will be started from the Python app Start button
    // Or call tuner.start() here if you want automatic startup
    
    TUNING_SERIAL.println("System ready - PID Tuning Interface active");
}

// =========================================================================
// MAIN LOOP
// =========================================================================

void loop() {
    // Update the tuning interface (handles serial communication)
    tuner.update();
    
    // Your normal control code goes here
    if (tuner.isRunning()) {
        // Read sensor (you could also call readMySensor() directly)
        float processValue = readMySensor();
        
        // Update PID controller
        pid.update(processValue);
        
        // The output is automatically sent to the OUTPUT_PIN
        // You can also read it if needed: float output = pid.getOutput();
    }
    
    // Add any other application code here
    // The tuning interface runs in parallel without interfering
    
    delay(10);  // Small delay to prevent overwhelming the serial
}

// =========================================================================
// OPTIONAL: Additional Functions
// =========================================================================

// You can access tuning parameters from your code if needed
void checkTuningStatus() {
    if (TUNING_SERIAL.available()) {
        // The tuner.update() function handles serial communication
        // But you could add custom commands here if needed
    }
    
    // Example: Check if a step test is active
    if (tuner.isStepTestActive()) {
        // Maybe flash an LED or log to SD card during step test
    }
}

// Example: Emergency stop function
void emergencyStop() {
    tuner.stop();
    pid.disable();
    // Add other emergency actions here
}
