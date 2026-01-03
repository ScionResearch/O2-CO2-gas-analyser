# PID Control Safety Features

The PID_Control library includes several safety features to protect your system from dangerous conditions caused by faulty sensors or abnormal process behavior.

## Overview of Safety Features

### 1. Stale Data Detection
Detects when the process value is not changing enough over time, indicating a potentially stuck sensor or frozen process.

### 2. Safe Value Limits
Ensures the process value stays within a defined safe operating range.

### 3. NaN Detection
Automatically detects and handles NaN (Not a Number) values from sensors.

### 4. Error State Management
Automatically disables the controller and forces output to 0 when any error is detected.

## API Reference

### Stale Data Detection

```cpp
// Configure stale data detection
void setStaleDataDetection(float minRateOfChange, unsigned long maxTimeMs);

// Enable/disable stale data detection
void enableStaleDataDetection();
void disableStaleDataDetection();
```

**Parameters:**
- `minRateOfChange`: Minimum rate of change in units/second
- `maxTimeMs`: Maximum time (in milliseconds) the value can stay below the minimum rate

**Behavior:**
- Only active when process value is away from setpoint (> 0.1 units difference)
- Resets timer when at setpoint or when rate of change is sufficient
- Disables controller if conditions are not met

### Safe Value Limits

```cpp
// Set safe operating range
void setSafeValueLimits(float minValue, float maxValue);

// Enable/disable safe value limits
void enableSafeValueLimits();
void disableSafeValueLimits();
```

**Parameters:**
- `minValue`: Minimum acceptable process value
- `maxValue`: Maximum acceptable process value

**Behavior:**
- Controller disables immediately if value is outside range
- Protects against sensor failure or extreme conditions

### Error State Management

```cpp
// Check if controller is in error state
bool isInErrorState();

// Clear error state (must be done before re-enabling)
void clearErrorState();
```

**Behavior:**
- Error state is set automatically when any safety check fails
- Controller is disabled and output forced to 0
- Must call `clearErrorState()` before re-enabling

## Usage Examples

### Basic Setup with Safety Features

```cpp
#include <PID_Control.h>

PID_Control pid(OUTPUT_PIN, true);

void setup() {
    // Initialize PID
    pid.begin(2.0, 0.1, 0.05, 25.0);
    
    // Configure stale data detection
    // Trigger if value changes less than 0.1 units/sec for 5 seconds
    pid.setStaleDataDetection(0.1, 5000);
    pid.enableStaleDataDetection();
    
    // Configure safe value limits (temperature sensor example)
    pid.setSafeValueLimits(-20.0, 120.0);  // -20°C to 120°C
    pid.enableSafeValueLimits();
    
    pid.enable();
}

void loop() {
    float temperature = readTemperatureSensor();
    
    // Update PID (safety checks are performed automatically)
    pid.update(temperature);
    
    // Check for error conditions
    if (pid.isInErrorState()) {
        // Handle error - could be alarm, notification, etc.
        digitalWrite(ERROR_LED, HIGH);
        
        // To recover: fix the issue and clear error state
        // pid.clearErrorState();
        // pid.enable();
    }
    
    delay(100);
}
```

### Temperature Control with Safety

```cpp
// For temperature control, you might use:
pid.setStaleDataDetection(0.5, 10000);  // 0.5°C/sec, 10 second timeout
pid.setSafeValueLimits(0.0, 100.0);     // 0°C to 100°C for water heating

// For flow control:
pid.setStaleDataDetection(1.0, 3000);   // 1 L/min, 3 second timeout
pid.setSafeValueLimits(-5.0, 50.0);     // -5 to 50 L/min

// For pressure control:
pid.setStaleDataDetection(0.1, 5000);   // 0.1 bar/sec, 5 second timeout
pid.setSafeValueLimits(0.0, 10.0);      // 0 to 10 bar
```

### Error Recovery

```cpp
void handlePIDError() {
    if (pid.isInErrorState()) {
        Serial.println("PID error detected!");
        
        // Investigate the cause
        float pv = readSensor();
        if (isnan(pv)) {
            Serial.println("Error: Sensor returning NaN");
        } else if (pv < -20.0 || pv > 120.0) {
            Serial.print("Error: Value out of range: ");
            Serial.println(pv);
        }
        
        // Attempt recovery after fixing the issue
        if (sensorIsHealthy()) {
            pid.clearErrorState();
            pid.enable();
            Serial.println("Error cleared, control resumed");
        }
    }
}
```

## Best Practices

1. **Choose appropriate thresholds:**
   - Set stale data threshold based on your process dynamics
   - Fast processes need higher rate thresholds
   - Slow processes need lower thresholds

2. **Safe value limits:**
   - Set wider than normal operating range
   - Include sensor error margin
   - Consider physical limitations of your system

3. **Error handling:**
   - Always check `isInErrorState()` in your main loop
   - Provide visual indication (LED, display)
   - Log errors for debugging
   - Implement automatic recovery where appropriate

4. **Testing:**
   - Test each safety feature individually
   - Simulate fault conditions:
     - Disconnect sensor (stuck value)
     - Short sensor (extreme value)
     - Send NaN from sensor
   - Verify error recovery works

## Integration with PID_Tune

The safety features work seamlessly with the PID_Tune library:

```cpp
#include <PID_Control.h>
#include <PID_Tune.h>

PID_Control pid(OUTPUT_PIN, true);
PID_Tune tuner(pid);

void setup() {
    pid.begin(2.0, 0.1, 0.05, 25.0);
    
    // Configure safety features
    pid.setStaleDataDetection(0.1, 5000);
    pid.enableStaleDataDetection();
    pid.setSafeValueLimits(-10.0, 110.0);
    pid.enableSafeValueLimits();
    
    tuner.begin(Serial, 115200);
    tuner.setSensorCallback(readSensor);
    tuner.enable();
}

void loop() {
    tuner.update();
    
    if (tuner.isRunning() && !pid.isInErrorState()) {
        pid.update(readSensor());
    } else if (pid.isInErrorState()) {
        // Notify tuning app of error
        Serial.println("{\"type\": \"error\", \"message\": \"Safety error triggered\"}");
    }
}
```

## Troubleshooting

### Controller keeps disabling:
1. Check if process value is within safe limits
2. Verify sensor is changing sufficiently when away from setpoint
3. Check for NaN values from sensor
4. Review rate of change threshold - might be too high

### False triggers on stale data:
1. Increase `maxTimeMs` for slower processes
2. Decrease `minRateOfChange` for slow-changing processes
3. Check if setpoint is being reached frequently

### Can't re-enable after error:
1. Must call `clearErrorState()` before `enable()`
2. Fix the underlying issue first
3. Check if safety conditions are still being violated
