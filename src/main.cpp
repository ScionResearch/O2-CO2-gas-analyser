#include "defines.h"

float ntcTemperatureC(
    float R_pulldown,
    float R_NTC_25,
    float Beta,
    uint16_t adcMax,
    uint16_t adcRaw
);
static char errorMessage[64];
static int16_t error;

void setup() {
  uint32_t startTime = millis();
  asm(".global _printf_float");
  Serial.begin(115200);
  while (!Serial);
  Serial.printf("Starting Gas Analyser HW tests at TS %d...\n", startTime);

  // LED init
  startTime = millis();
  Serial.printf("Starting LED interface at TS: %d\n", startTime);
  led.begin();
  led.clear();
  led.setPixelColor(0, 0x00FFFF);
  led.show();
  Serial.printf("Completed LED init in %d ms\n", millis() - startTime);

  // Analog init
  startTime = millis();
  Serial.printf("Starting Analog interface at TS: %d\n", startTime);
  analogReference(AR_EXTERNAL);
  analogReadResolution(12);
  analogRead(PIN_VPSU_FB);
  analogRead(PIN_NTC_HEAT);
  analogRead(PIN_NTC_AMB);

  pinMode(PIN_HEAT_PWM, OUTPUT);
  digitalWrite(PIN_HEAT_PWM, LOW);

  analogRead(PIN_V_O2_SENS);
  Serial.printf("Completed Analog init in %d ms\n", millis() - startTime);

  // Read raw values
  delay(1000);
  Serial.println("Reading raw values:");
  Serial.printf("PSU: %d\n", analogRead(PIN_VPSU_FB));
  Serial.printf("Heat NTC: %d\n", analogRead(PIN_NTC_HEAT));
  Serial.printf("Ambient NTC: %d\n", analogRead(PIN_NTC_AMB));
  Serial.printf("O2 Sensor: %d\n", analogRead(PIN_V_O2_SENS));

  // SHT45 init
  startTime = millis();
  Serial.printf("Starting SHT45 interface at TS: %d\n", startTime);
  Wire.begin();
  sht.begin(Wire, SHT40_I2C_ADDR_44);
  sht.softReset();
  delay(10);
  uint32_t serialNumber = 0;
  error = sht.serialNumber(serialNumber);
  if (error != 0) {
    errorToString(error, errorMessage, sizeof errorMessage);
    Serial.printf("Error trying to execute serialNumber(): %s\n", errorMessage);
    while(1);
  }
  Serial.printf("serialNumber: %d\n", serialNumber);

  Serial.println("-- Done --");
}

void loop() {
  Serial.printf("VPSU: %0.2fV || O2 Sensor: %d || Heat NTC: %0.2f°C || Ambient NTC: %0.2f°C || ", 
                (analogRead(PIN_VPSU_FB) * VREF_ADC_MULTIPLIER)/1000.0, analogRead(PIN_V_O2_SENS),
                ntcTemperatureC(10000.0, 10000.0, 4100.0, 4095, analogRead(PIN_NTC_HEAT)),
                ntcTemperatureC(10000.0, 10000.0, 4100.0, 4095, analogRead(PIN_NTC_AMB)));
  float aTemperature = 0.0;
  float aHumidity = 0.0;
  error = sht.measureHighPrecision(aTemperature, aHumidity);
  if (error != 0) {
      Serial.print("Error trying to execute measureLowestPrecision(): ");
      errorToString(error, errorMessage, sizeof errorMessage);
      Serial.println(errorMessage);
      return;
  }
  Serial.print(" SHT Temperature: ");
  Serial.print(aTemperature);
  Serial.print(" || Humidity: ");
  Serial.print(aHumidity);
  Serial.println();

  //Serial.printf("NTC Amb: %d, NTC Heat: %d\n", analogRead(PIN_NTC_AMB), analogRead(PIN_NTC_HEAT));
  delay(200);
}

float ntcTemperatureC(
    float R_pulldown,   // Fixed resistor to GND
    float R_NTC_25,     // NTC resistance at 25°C
    float Beta,         // Beta constant
    uint16_t adcMax,    // ADC full scale (e.g. 4095)
    uint16_t adcRaw     // ADC reading
)
{
    if (adcRaw == 0 || adcRaw >= adcMax) {
        return NAN;
    }

    // High-side NTC, low-side fixed resistor
    float R_ntc = R_pulldown *
                  ((float)(adcMax - adcRaw) / (float)adcRaw);

    const float T0 = 298.15f; // 25°C in Kelvin
    float invT = (1.0f / T0) +
                 (1.0f / Beta) * log(R_ntc / R_NTC_25);

    return (1.0f / invT) - 273.15f;
}