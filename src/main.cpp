#include "defines.h"
#include "FlashStorage_SAMD.h"

uint32_t timeStamp = 0;

// Forward declarations
uint16_t serialConfigFromParts(uint8_t parity, uint8_t stopBits, uint8_t dataBits);
void serialConfigToParts(uint16_t config, uint16_t &stopBits, uint16_t &parity, uint16_t &dataBits);
void configToHolding();
bool holdingToConfig();
void sensorsToInputs();
float vapourPressure(float temperatureC, float relativeHumidity);
float absoluteHumidity(float temperatureC, float vapourPressure);


void setup() {
  asm(".global _printf_float");
  Serial.begin(115200);
  /*while (!Serial) {
    delay(100);
  }*/

  Serial.println("O2-CO2 Gas Analyser Starting...");

  // EEPROM init
  uint8_t eepromVersion = EEPROM.read(0);
  if (eepromVersion != EEPROM_VERSION_BYTE) {
    Serial.printf("Got EEPROM version: 0x%0X, expected: 0x%0X\n\r", eepromVersion, EEPROM_VERSION_BYTE);
    Serial.println("Writing EEPROM version...");
    EEPROM.put(0, EEPROM_VERSION_BYTE);
    EEPROM.put(1, config);
    EEPROM.commit();
  } else {
    Serial.printf("Got EEPROM version: 0x%0X\n\r", eepromVersion);
    EEPROM.get(1, config);
    Serial.printf("Got config:\n\r");
    Serial.printf("  modbusBaudrate: %u\n\r", config.modbusBaudrate);
    Serial.printf("  modbusConfig: %u\n\r", config.modbusConfig);
    Serial.printf("  modbusSlaveID: %u\n\r", config.modbusSlaveID);
    Serial.printf("  heaterKp: %.2f\n\r", config.heaterKp);
    Serial.printf("  heaterKi: %.2f\n\r", config.heaterKi);
    Serial.printf("  heaterKd: %.2f\n\r", config.heaterKd);
    Serial.printf("  heaterMaxI: %.2f\n\r", config.heaterMaxI);
    Serial.printf("  heaterSetpointC: %.2f\n\r", config.heaterSetpointC);
    Serial.printf("  heaterMinAmbientDeltaC: %.2f\n\r", config.heaterMinAmbientDeltaC);
    Serial.printf("  o2CalibrationOffset: %.2f\n\r", config.o2CalibrationOffset);
    Serial.printf("  o2CalibrationScale: %.2f\n\r", config.o2CalibrationScale);
    Serial.printf("  co2CalibrationOffset: %d\n\r", config.co2CalibrationOffset);
    Serial.printf("  co2CalibrationScale: %.2f\n\r", config.co2CalibrationScale);
    Serial.printf("  ntcAmbOffsetC: %.2f\n\r", config.ntcAmbOffsetC);
    Serial.printf("  ntcAmbScale: %.2f\n\r", config.ntcAmbScale);
    Serial.printf("  ntcHeatOffsetC: %.2f\n\r", config.ntcHeatOffsetC);
    Serial.printf("  ntcHeatScale: %.2f\n\r", config.ntcHeatScale);
  }
  
  // LED init
  led.begin();
  led.clear();
  led.setPixelColor(0, LED_BLUE);
  led.show();

  // Analog init
  analogReference(AR_EXTERNAL);
  analogReadResolution(12);
  ntcAmb.begin();
  ntcHeat.begin();
  ntcAmb.setOffset(config.ntcAmbOffsetC);
  ntcAmb.setScale(config.ntcAmbScale);  
  ntcHeat.setOffset(config.ntcHeatOffsetC);
  ntcHeat.setScale(config.ntcHeatScale);
  ntcAmb.enableMovingAverage(NTC_MOVING_AVERAGE_SAMPLES);
  ntcAmb.enableOversampling(NTC_OVERSAMPLING_SAMPLES);
  ntcHeat.enableMovingAverage(NTC_MOVING_AVERAGE_SAMPLES);
  ntcHeat.enableOversampling(NTC_OVERSAMPLING_SAMPLES);

  // O2 sensor init
  O2.begin();
  O2.setZeroCalibration(config.o2CalibrationOffset);
  O2.setSpanCalibration(config.o2CalibrationScale);

  Serial.printf("O2 sensor zero calibration: %.2f\n\r", O2.getZeroCalibration());
  Serial.printf("O2 sensor span calibration: %.2f\n\r", O2.getSpanCalibration());
  Serial.printf("O2: %.2f\n\r", O2.readO2());

  // CO2 sensor init
  CO2.begin();
  CO2.setCalibrationOffset(config.co2CalibrationOffset);
  CO2.setCalibrationScale(config.co2CalibrationScale);

  // SHT4x init
  Wire.begin();
  sht.begin(Wire, SHT40_I2C_ADDR_44);
  sht.softReset();
  uint32_t serialNumber = 0;
  uint16_t error = sht.serialNumber(serialNumber);
  Serial.println("Initialising SHT4x sensor...");
  if (error) {
    char errorMessage[64];
    Serial.print("Error trying to execute serialNumber(): ");
    errorToString(error, errorMessage, sizeof errorMessage);
    Serial.println(errorMessage);
    led.setPixelColor(0, LED_RED);
    led.show();
    while (1);
  }
  Serial.print("  serialNumber: ");
  Serial.print(serialNumber);
  Serial.println();

  // Modbus init
  Serial.println("Initialising Modbus...");
  configToHolding();
  modbus.configureHoldingRegisters(holding, NUM_HOLDING_REGISTERS);
  modbus.configureInputRegisters(input, NUM_INPUT_REGISTERS);

  modbus.begin(config.modbusSlaveID,config.modbusBaudrate, config.modbusConfig);
  
  // PID Control init
  Serial.println("Initialising PID controller...");
  heaterCtrl.begin(config.heaterKp, config.heaterKi, config.heaterKd, config.heaterSetpointC);
  heaterCtrl.setOutputLimits(0, 255);
  heaterCtrl.setIntegralLimits(-40, config.heaterMaxI);
  heaterCtrl.setSampleTime(500);
  heaterCtrl.setStaleDataDetection(0.1, 10000);
  heaterCtrl.setSafeValueLimits(0.0, 60.0);
  heaterCtrl.enable();
  heaterCtrl.setIterm(50.0); // Set inital Iterm to stop long wind up times when T is already close to setpoint

  led.setPixelColor(0, LED_AMBER);  // Heating to setpoint
  led.show();

  bool temperatureStable = false;
  uint32_t stableCount = 0;
  float maxDeviationC = 0.2;
  bool heaterError = false;
  
  timeStamp = millis();

  Serial.printf("Waiting for heater to reach %.2f °C...\n\r", config.heaterSetpointC);
  while (!temperatureStable) {
    float temperatureHeatC = ntcHeat.temperature();
    heaterCtrl.update(temperatureHeatC);
    Serial.printf("  Temp: %.2f °C P term: %.2f I term: %.2f\n\r", temperatureHeatC, heaterCtrl.getProportional(), heaterCtrl.getIntegral());
    if (abs(heaterCtrl.getError()) <= maxDeviationC) {
      stableCount++;
      if (stableCount >= 40) {  // ~ 20 seconds at 500ms interval sample time
        temperatureStable = true;
        break;
      }
    } else stableCount = 0; // Reset if error is too high

    if (millis() - timeStamp >= 900000) {
      heaterError = true;
      break;
    }

    delay(500);
  }

  if (heaterError) {
    Serial.println("Heater error - timed out while trying to reach setpoint.");
    led.setPixelColor(0, LED_RED);
    led.show();
    while (1);
  }

  Serial.printf("Reached setpoint %.2f °C in %.2f seconds\n\r", config.heaterSetpointC, (millis() - timeStamp) / 1000.0);

  led.setPixelColor(0, LED_GREEN);
  led.show();

  Serial.println("Initialisation complete.");
}

void loop() {
  sht.measureHighPrecision(modbusInputRegisters.gasTempC, modbusInputRegisters.gasRH);
  modbusInputRegisters.gasVP = vapourPressure(modbusInputRegisters.gasTempC, modbusInputRegisters.gasRH);
  modbusInputRegisters.gasAH = absoluteHumidity(modbusInputRegisters.gasTempC, modbusInputRegisters.gasVP);

  if (CO2.manage()) {
    led.setPixelColor(0, LED_CYAN);
    led.show();
  }

  static bool O2calibrating = false;
  if (O2.isCalibrating()) {
    if (!O2calibrating) {
      O2calibrating = true;
      led.setPixelColor(0, LED_BLUE);
      led.show();
    }    
    O2.manageCalibration();
  } else if (O2calibrating) {
    uint8_t error = O2.calibrationError();
    if (error == 0) {
      Serial.printf("O2 calibration complete, span coefficient: %.2f offset: %.2f\n\r", O2.getSpanCalibration(), O2.getZeroCalibration());
      led.setPixelColor(0, LED_GREEN);
      led.show();
    } else {
      const char *errorMessage[4] = {"did not stabilise", "current below min", "current above max", "unknown error"};
      if (error > 3) error = 3;
      Serial.printf("O2 calibration error: %s\n\r", errorMessage[error]);
      led.setPixelColor(0, LED_RED);
      led.show();
    }
    O2calibrating = false;
  }

  if (millis() - timeStamp >= 1000) {
    timeStamp = millis();
    
    modbusInputRegisters.ambTempC = ntcAmb.temperature();
    modbusInputRegisters.heatTempC = ntcHeat.temperature();
    heaterCtrl.update(modbusInputRegisters.heatTempC);
    modbusInputRegisters.O2percent = O2.readO2(modbusInputRegisters.gasTempC);
    modbusInputRegisters.CO2ppm = CO2.CO2();
    modbusInputRegisters.CO2percent = modbusInputRegisters.CO2ppm / 10000.0;

    led.setPixelColor(0, LED_GREEN);
    led.show();
  }

  sensorsToInputs();
  int FC = modbus.poll();
  if (FC > 0) {
    Serial.printf("FC: %d\n\r", FC);
    if (FC == 6 || FC == 16) {
      if (holdingToConfig()) {
        // Add EEPROM.commit() here
      }
    }
  }
}

uint16_t serialConfigFromParts(uint8_t parity, uint8_t stopBits, uint8_t dataBits) {
  // Parity: 0 = None, 1 = Even, 2 = Odd, 3 = None
  // Stop bits: 0 = 1, 1 = 1, 2 = 1.5, 3 = 2
  // Data bits: 0 = 8, 5-8 = 5-8
  if (parity > 3 || stopBits > 3 || dataBits > 8 || (dataBits > 0 && dataBits < 5)) return SERIAL_8N1;
  if (parity == 0) parity = 3;
  if (stopBits == 0) stopBits = 1;
  if (dataBits == 0) dataBits = 8;
  return (parity & 0x03) | ((stopBits & 0x03) << 4) | ((dataBits & 0x0F) << 8);
}

void serialConfigToParts(uint16_t config, uint16_t &stopBits, uint16_t &parity, uint16_t &dataBits) {
  parity = config & 0x03;
  stopBits = (config >> 4) & 0x03;
  dataBits = (config >> 8) & 0x0F;
}

// Copy main config to holding register struct
void configToHolding() {
  modbusHoldingRegisters.modbusSlaveID = config.modbusSlaveID;
  modbusHoldingRegisters.modbusBaudrate = config.modbusBaudrate / 10;
  uint16_t dataBits;
  serialConfigToParts(config.modbusConfig, modbusHoldingRegisters.modbusStopBits, modbusHoldingRegisters.modbusParity, dataBits);
  modbusHoldingRegisters.modbus120Rterm = config.modbus120Rterm;
  modbusHoldingRegisters.heaterKp = config.heaterKp;
  modbusHoldingRegisters.heaterKi = config.heaterKi;
  modbusHoldingRegisters.heaterKd = config.heaterKd;
  modbusHoldingRegisters.heaterMaxI = config.heaterMaxI;
  modbusHoldingRegisters.heaterSetpointC = config.heaterSetpointC;
  modbusHoldingRegisters.heaterMinAmbientDeltaC = config.heaterMinAmbientDeltaC;

  modbusHoldingRegisters.O2SetZeroPoint = 0;
  modbusHoldingRegisters.O2SetXPoint = 0;
  modbusHoldingRegisters.CO2SetZeroPoint = 0;
  modbusHoldingRegisters.CO2SetXPoint = 0;

  // Update holding registers array
  memcpy(holding, &modbusHoldingRegisters, sizeof(modbusHoldingRegisters));
}

// Update main config from holding register struct
bool holdingToConfig() {
  bool configToSave = false;
  bool reinitModbus = false;
  bool serialConfigChanged = false;
  ModbusHoldingRegisters temp;
  memcpy(&temp, holding, sizeof(temp));

  // Modbus settings
  if (temp.modbusSlaveID != modbusHoldingRegisters.modbusSlaveID) {
    if (temp.modbusSlaveID > 1 && temp.modbusSlaveID < 247) {
      modbusHoldingRegisters.modbusSlaveID = temp.modbusSlaveID;
      config.modbusSlaveID = temp.modbusSlaveID;
      configToSave = true;
      reinitModbus = true;
      Serial.printf("Modbus slave ID changed to %u\n\r", temp.modbusSlaveID);
    }
  }
  if (temp.modbusBaudrate != modbusHoldingRegisters.modbusBaudrate) {
    if (   temp.modbusBaudrate == 960 // Baudrate stored as 1 tenth to fit inside uint16
        || temp.modbusBaudrate == 1920
        || temp.modbusBaudrate == 3840
        || temp.modbusBaudrate == 5760
        || temp.modbusBaudrate == 11520) {
      
      modbusHoldingRegisters.modbusBaudrate = temp.modbusBaudrate;
      config.modbusBaudrate = temp.modbusBaudrate * 10;
      configToSave = true;
      reinitModbus = true;
      Serial.printf("Modbus baudrate changed to %u\n\r", temp.modbusBaudrate * 10);
    }    
  }
  if (temp.modbusParity != modbusHoldingRegisters.modbusParity) {
    if (temp.modbusParity > 0 && temp.modbusParity < 4) {
      modbusHoldingRegisters.modbusParity = temp.modbusParity;
      serialConfigChanged = true;
      Serial.printf("Modbus parity changed to %u\n\r", temp.modbusParity);
    }
  }
  if (temp.modbusStopBits != modbusHoldingRegisters.modbusStopBits) {
    if (temp.modbusStopBits > 0 && temp.modbusStopBits < 4) {
      modbusHoldingRegisters.modbusStopBits = temp.modbusStopBits;
      serialConfigChanged = true;
      Serial.printf("Modbus stop bits changed to %u\n\r", temp.modbusStopBits);
    }
  }
  if (serialConfigChanged) {
    config.modbusConfig = serialConfigFromParts(modbusHoldingRegisters.modbusParity, modbusHoldingRegisters.modbusStopBits, 8);
    configToSave = true;
    reinitModbus = true;
    Serial.printf("Modbus config changed to %04X\n\r", config.modbusConfig);
  }
  if (temp.modbus120Rterm != modbusHoldingRegisters.modbus120Rterm) {
    if (temp.modbus120Rterm >= 0 && temp.modbus120Rterm < 2) {
      modbusHoldingRegisters.modbus120Rterm = temp.modbus120Rterm;
      config.modbus120Rterm = temp.modbus120Rterm;
      digitalWrite(PIN_RS485_TERM, (bool)temp.modbus120Rterm);
      configToSave = true;
      Serial.printf("Modbus 120R term changed to %s\n\r", temp.modbus120Rterm ? "temrinated" : "un-terminated");
    }
  }

  // Heater settings
  if (temp.heaterKp != modbusHoldingRegisters.heaterKp) {
    if (temp.heaterKp > 0.0 && temp.heaterKp < 1000.0) {
      modbusHoldingRegisters.heaterKp = temp.heaterKp;
      config.heaterKp = temp.heaterKp;
      configToSave = true;
      Serial.printf("Heater Kp changed to %f\n\r", temp.heaterKp);
    }
  }
  if (temp.heaterKi != modbusHoldingRegisters.heaterKi) {
    if (temp.heaterKi > 0.0 && temp.heaterKi < 1000.0) {
      modbusHoldingRegisters.heaterKi = temp.heaterKi;
      config.heaterKi = temp.heaterKi;
      configToSave = true;
      Serial.printf("Heater Ki changed to %f\n\r", temp.heaterKi);
    }
  }
  if (temp.heaterKd != modbusHoldingRegisters.heaterKd) {
    if (temp.heaterKd > 0.0 && temp.heaterKd < 1000.0) {
      modbusHoldingRegisters.heaterKd = temp.heaterKd;
      config.heaterKd = temp.heaterKd;
      configToSave = true;
      Serial.printf("Heater Kd changed to %f\n\r", temp.heaterKd);
    }
  }
  if (temp.heaterMaxI != modbusHoldingRegisters.heaterMaxI) {
    if (temp.heaterMaxI > 0.0 && temp.heaterMaxI < 255.0) {
      modbusHoldingRegisters.heaterMaxI = temp.heaterMaxI;
      config.heaterMaxI = temp.heaterMaxI;
      configToSave = true;
      Serial.printf("Heater max I changed to %f\n\r", temp.heaterMaxI);
    }
  }
  if (temp.heaterSetpointC != modbusHoldingRegisters.heaterSetpointC) {
    if (temp.heaterSetpointC >= 20.0 && temp.heaterSetpointC <= 50.0) {
      modbusHoldingRegisters.heaterSetpointC = temp.heaterSetpointC;
      config.heaterSetpointC = temp.heaterSetpointC;
      configToSave = true;   
      Serial.printf("Heater setpoint changed to %f\n\r", temp.heaterSetpointC);  
    }
  }
  if (temp.heaterMinAmbientDeltaC != modbusHoldingRegisters.heaterMinAmbientDeltaC) {
    if (temp.heaterMinAmbientDeltaC >= 0.0 && temp.heaterMinAmbientDeltaC <= 20.0) {
      modbusHoldingRegisters.heaterMinAmbientDeltaC = temp.heaterMinAmbientDeltaC;
      config.heaterMinAmbientDeltaC = temp.heaterMinAmbientDeltaC;
      configToSave = true;
      Serial.printf("Heater min ambient delta changed to %f\n\r", temp.heaterMinAmbientDeltaC);
    }
  }

  // Sensor calibration
  if (temp.O2SetZeroPoint == 1) {
    // Start zero point calibration
    O2.startZeroCalibration(modbusInputRegisters.gasTempC);
    Serial.println("Start O2 zero point calibration");
  }
  if (temp.O2SetXPoint >= 10.0 && temp.O2SetXPoint <= 25.0) {
    // Start high point calibration
    O2.startSpanCalibration(temp.O2SetXPoint, modbusInputRegisters.gasTempC);
    Serial.println("Start O2 high point calibration");
  }
  if (temp.CO2SetZeroPoint > 0 && temp.CO2SetZeroPoint < 5000) {
    // Start zero point calibration
    CO2.setCalibrationOffset(temp.CO2SetZeroPoint - CO2.CO2());
    Serial.println("Start CO2 zero point calibration");
  }
  if (temp.CO2SetXPoint >= 300.0 && temp.CO2SetXPoint <= 50000.0) {
    // Start high point calibration
    // TODO
    Serial.println("Start CO2 high point calibration");
  }

  // Rewrite holding registers array with new (sanitised values)
  memcpy(holding, &modbusHoldingRegisters, sizeof(modbusHoldingRegisters));

  if (reinitModbus) {
    modbus.begin(config.modbusSlaveID, config.modbusBaudrate, config.modbusConfig);
  }

  if (configToSave) {
    EEPROM.put(1, config);
    return true;
  } else {
    return false;
  }
}

void sensorsToInputs() {
  memcpy(input, &modbusInputRegisters, sizeof(modbusInputRegisters));
}

float vapourPressure(float temperatureC, float relativeHumidity) {
  // Saturation vapour pressure (Pa)
  float es = 610.94f * expf((17.625f * temperatureC) / (temperatureC + 243.04f));

  // Actual vapour pressure (Pa)
  return (relativeHumidity / 100.0f) * es;
}

float absoluteHumidity(float temperatureC, float vapourPressure) {
  return (vapourPressure * 1000.0f) / (461.5f * (temperatureC + 273.15f));
}

