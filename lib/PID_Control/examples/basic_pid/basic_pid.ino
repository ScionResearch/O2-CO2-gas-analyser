#include <PID_Control.h>

// Create PID controller instance
// Parameters: output pin, polarity (true = normal, false = reverse)
PID_Control pid(3, true); // Using pin 3 (PWM) for output

void setup() {
    Serial.begin(115200);
    Serial.println("PID Control Example");
    
    // Initialize PID controller
    // Parameters: Kp, Ki, Kd, setpoint
    pid.begin(2.0, 0.5, 0.1, 25.0); // Target temperature of 25°C
    
    // Optional: Configure output limits (default 0-255 for PWM)
    pid.setOutputLimits(0, 255);
    
    // Optional: Configure integral limits to prevent windup
    pid.setIntegralLimits(-500, 500);
    
    // Optional: Set sample time in milliseconds (default 100ms)
    pid.setSampleTime(100);
    
    Serial.println("PID Controller initialized");
    Serial.println("Format: Input, Setpoint, Output");
}

void loop() {
    // Simulate reading a sensor (e.g., temperature)
    float sensorValue = analogRead(A0);
    float temperature = sensorValue * 0.1; // Convert to temperature
    
    // Update PID controller - this is the ONLY method you need to call!
    pid.update(temperature);
    
    // Print current values
    Serial.print(temperature);
    Serial.print(", ");
    Serial.print(pid.getSetpoint());
    Serial.print(", ");
    Serial.println(pid.getOutput());
    
    // Simulate changing setpoint after 10 seconds
    static unsigned long lastChange = 0;
    if (millis() - lastChange > 10000) {
        float newSetpoint = 20.0 + random(0, 10); // Random setpoint between 20-30
        pid.setpoint(newSetpoint);
        Serial.print("Setpoint changed to: ");
        Serial.println(newSetpoint);
        lastChange = millis();
    }
    
    delay(100); // Match sample time
}
