/**************************************************************************************************
 * PID Control Safety Features Example
 * 
 * This example demonstrates the safety features of the PID_Control library:
 * - Stale data detection
 * - Safe value limits
 * - NaN detection
 * - Error state handling
 **************************************************************************************************/

#include <PID_Control.h>

#define SENSOR_PIN A0
#define OUTPUT_PIN 3
#define LED_PIN 13  // Onboard LED for error indication

PID_Control pid(OUTPUT_PIN, true);

void setup() {
    pinMode(LED_PIN, OUTPUT);
    
    Serial.begin(115200);
    Serial.println("PID Safety Features Example");
    
    // Initialize PID
    pid.begin(2.0, 0.1, 0.05, 25.0);
    pid.setOutputLimits(0, 255);
    pid.setSampleTime(100);
    
    // Configure safety features
    
    // 1. Stale data detection
    //    - Minimum rate of change: 0.1 units/second
    //    - Maximum time before error: 5000ms (5 seconds)
    //    - Only active when away from setpoint
    pid.setStaleDataDetection(0.1, 5000);
    pid.enableStaleDataDetection();
    
    // 2. Safe value limits
    //    - Process value must be between -10 and 110
    //    - Controller will disable if outside these limits
    pid.setSafeValueLimits(-10.0, 110.0);
    pid.enableSafeValueLimits();
    
    Serial.println("Safety features configured:");
    Serial.println("- Stale data detection: enabled (min rate: 0.1/s, max time: 5s)");
    Serial.println("- Safe value limits: enabled (-10 to 110)");
    Serial.println("- NaN detection: always enabled");
    Serial.println("\nController ready. Send commands:");
    Serial.println("  'start' - Enable control");
    Serial.println("  'stop'  - Disable control");
    Serial.println("  'clear' - Clear error state");
    Serial.println("  'status' - Show current status");
}

void loop() {
    static unsigned long lastPrint = 0;
    
    // Read sensor (simulated with potentiometer)
    float sensorValue = analogRead(SENSOR_PIN) * 0.1;
    
    // Simulate various error conditions for testing
    // Uncomment one of these to test safety features:
    
    // Test 1: Stuck sensor (returns same value)
    // static float stuckValue = 25.0;
    // sensorValue = stuckValue;
    
    // Test 2: NaN value
    // sensorValue = NAN;
    
    // Test 3: Out of range value
    // sensorValue = 150.0;
    
    // Update PID
    pid.update(sensorValue);
    
    // Check error state and control LED
    if (pid.isInErrorState()) {
        digitalWrite(LED_PIN, HIGH);  // LED on = error
    } else {
        digitalWrite(LED_PIN, LOW);   // LED off = normal
    }
    
    // Print status every 2 seconds
    if (millis() - lastPrint > 2000) {
        lastPrint = millis();
        
        Serial.print("PV: ");
        Serial.print(sensorValue, 2);
        Serial.print(" | SP: ");
        Serial.print(pid.getSetpoint(), 2);
        Serial.print(" | Output: ");
        Serial.print(pid.getOutput(), 0);
        Serial.print(" | Error: ");
        Serial.print(pid.getError(), 2);
        Serial.print(" | State: ");
        
        if (pid.isInErrorState()) {
            Serial.print("ERROR!");
        } else if (pid.isEnabled()) {
            Serial.print("Running");
        } else {
            Serial.print("Stopped");
        }
        
        Serial.println();
    }
    
    // Process serial commands
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        
        if (cmd == "start") {
            pid.enable();
            Serial.println("Control enabled");
        }
        else if (cmd == "stop") {
            pid.disable();
            Serial.println("Control disabled");
        }
        else if (cmd == "clear") {
            pid.clearErrorState();
            pid.enable();
            Serial.println("Error state cleared, control enabled");
        }
        else if (cmd == "status") {
            Serial.print("Enabled: ");
            Serial.print(pid.isEnabled() ? "Yes" : "No");
            Serial.print(" | Error: ");
            Serial.print(pid.isInErrorState() ? "Yes" : "No");
            Serial.print(" | Output: ");
            Serial.println(pid.getOutput(), 0);
        }
    }
    
    delay(100);
}

/*
Safety Feature Usage Notes:

1. STALE DATA DETECTION:
   - Monitors rate of change when process value is away from setpoint
   - If rate is too low for too long, controller disables
   - Useful for detecting stuck sensors or frozen processes
   - Not active when at setpoint (within 0.1 units)

2. SAFE VALUE LIMITS:
   - Process value must stay within defined range
   - Controller disables immediately if value is out of range
   - Useful for detecting sensor failure or extreme conditions

3. NaN DETECTION:
   - Always active, cannot be disabled
   - Controller disables if NaN is received
   - Protects against invalid sensor readings

4. ERROR STATE:
   - Controller automatically disables on any error
   - Output is forced to 0
   - Must clear error state before re-enabling
   - LED can be used for visual indication

5. RECOVERY:
   - Fix the underlying problem (sensor, wiring, etc.)
   - Call clearErrorState() to reset
   - Re-enable control with enable()
*/
