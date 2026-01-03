# PID_Tune Library Documentation v1.0.0

## Overview

The PID_Tune library provides a complete PID tuning interface that can be integrated into any Arduino project. It communicates with the Python PID Tuning App via serial using JSON protocol and includes comprehensive safety features.

## Features

- Real-time PID parameter adjustment
- Step response testing with analysis
- Live data plotting (PV, SP, Output, P, I, D terms)
- Min/max tracking
- Data export to CSV
- Non-intrusive design - runs alongside your code
- Callback-based sensor reading
- Configurable serial port (Serial, Serial1, Serial2, etc.)
- Safety features integration (stale data, safe limits, error handling)

## Quick Start

1. Include the libraries:
```cpp
#include <PID_Control.h>
#include <PID_Tune.h>
```

2. Configure serial port (optional, defaults to Serial):
```cpp
#define TUNING_SERIAL Serial  // Or Serial1, Serial2, etc.
```

3. Create objects:
```cpp
PID_Control pid(OUTPUT_PIN, true);
PID_Tune tuner(pid);
```

4. In setup(), initialize:
```cpp
pid.begin(Kp, Ki, Kd, setpoint);
tuner.begin(TUNING_SERIAL, 115200);  // Specify serial port
tuner.setSensorCallback(yourSensorFunction);

// Configure safety features (optional but recommended)
pid.setStaleDataDetection(0.1, 5000);  // Min rate, max time
pid.enableStaleDataDetection();
pid.setSafeValueLimits(0.0, 100.0);     // Safe operating range
pid.enableSafeValueLimits();

tuner.enable();
// Control starts from Python app Start button
// Optional: Call tuner.start() here for automatic startup
```

5. In loop(), update:
```cpp
tuner.update();
if (tuner.isRunning()) {
    pid.update(readSensor());
}
```

## API Reference

### Constructor
```cpp
PID_Tune(PID_Control& pid)
```
Creates a tuning interface linked to a PID controller.

### Initialization
```cpp
void begin(unsigned long baudRate = 115200)
void begin(HardwareSerial& serial, unsigned long baudRate = 115200)
```
Starts serial communication and initializes the interface.
- First form uses Serial (default)
- Second form allows specifying any serial port

### Serial Port Configuration

Different Arduino boards have different serial ports:

- **Arduino Uno/Nano**: Only Serial (pins 0/1)
- **Arduino Leonardo**: Serial (USB) and Serial1 (pins 0/1)
- **Arduino Mega**: Serial (USB), Serial1 (pins 19/18), Serial2 (pins 17/16), Serial3 (pins 15/14)
- **Arduino Due**: Serial (USB), Serial1 (pins 19/18), Serial2 (pins 17/16), Serial3 (pins 15/14)
- **ESP32**: Multiple HardwareSerial ports

Example configuration:
```cpp
// For Arduino Mega using Serial2
#define TUNING_SERIAL Serial2
tuner.begin(TUNING_SERIAL, 115200);

// Or directly:
tuner.begin(Serial2, 115200);
```

### Sensor Callback
```cpp
void setSensorCallback(SensorCallback callback)
```
Sets the function to read your sensor. The function must take no parameters and return a float.

```cpp
// Example
float readTemperature() {
    // Your sensor reading code here
    return temperature;
}

tuner.setSensorCallback(readTemperature);
```

### Main Update
```cpp
void update()
```
Call this in your loop() to handle serial communication and data sending.

### Control
```cpp
void enable() / void disable()
bool isEnabled()
void start() / void stop()
bool isRunning()
```

### Parameters
```cpp
void setPID(float Kp, float Ki, float Kd)
float getKp(), getKi(), getKd()
void setSetpoint(float sp)
float getSetpoint()
void setLoopPeriod(unsigned long periodMs)
```

### Limits
```cpp
void setOutputLimits(float min, float max)
void setIntegralLimits(float min, float max)
```

### Step Testing
```cpp
void startStepTest(float amplitude)
void stopStepTest()
bool isStepTestActive()
```

### Data Access
```cpp
float getProcessValue()
float getOutput()
float getError()
float getProportional(), getIntegral(), getDerivative()
```

## Integration Tips

1. **Non-blocking**: The tuner.update() function is non-blocking and won't interfere with your code.

2. **Data Rate**: Data is sent at 10Hz regardless of your control loop frequency.

3. **Memory**: Uses minimal memory (~1KB RAM, ~8KB flash).

4. **Custom Commands**: You can extend the library by modifying the processCommand() method.

5. **Multiple Sensors**: Use a lambda function to select between sensors:
```cpp
tuner.setSensorCallback([]() {
    return sensor1;  // or sensor2 based on some condition
});
```

6. **Manual Control Start**: By default, control does not start automatically. 
   Users must click the Start button in the Python app to begin control.
   - This prevents unexpected actuator movement on startup
   - The Stop button will force output to 0 for safety
   - If you want automatic startup, call `tuner.start()` in setup()

7. **Safety Features**: Always configure appropriate safety features:
   ```cpp
   // For temperature control
   pid.setStaleDataDetection(0.5, 10000);  // 0.5°C/s, 10s timeout
   pid.enableStaleDataDetection();
   pid.setSafeValueLimits(-20.0, 120.0);    // Sensor failure limits
   pid.enableSafeValueLimits();
   
   // Check for errors in your loop
   if (pid.isInErrorState()) {
       // Handle error - notify user, log, etc.
       digitalWrite(ERROR_LED, HIGH);
   }
   ```

## Example Applications

### Temperature Control
```cpp
float readTemperature() {
    int raw = analogRead(THERMISTOR_PIN);
    return convertToTemperature(raw);
}

tuner.setSensorCallback(readTemperature);
```

### Motor Speed Control
```cpp
float readSpeed() {
    return encoder.getRPM();
}

tuner.setSensorCallback(readSpeed);
```

### Position Control
```cpp
float readPosition() {
    return potentiometer.read();
}

tuner.setSensorCallback(readPosition);
```

## Troubleshooting

1. **No Communication**: Check baud rate matches the Python app (115200).

2. **Garbled Data**: Ensure you're calling tuner.update() regularly.

3. **Sensor Not Updating**: Verify your callback function returns a valid float.

4. **Control Not Working**: Make sure you're calling pid.update() with the sensor value.

## Migration from Companion Sketch

To convert from the original companion sketch:

1. Move your sensor reading code into a callback function
2. Replace all serial handling with tuner.update()
3. Remove JSON parsing - the library handles it
4. Use tuner methods instead of direct variable access

The simplified companion sketch shows this conversion.
