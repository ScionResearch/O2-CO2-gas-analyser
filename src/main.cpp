#include "defines.h"

uint32_t timeStamp = 0;

float ntcTemperatureC(
    float R_pulldown,
    float R_NTC_25,
    float Beta,
    uint16_t adcMax,
    uint16_t adcRaw
);

void setup() {
  asm(".global _printf_float");
  Serial.begin(115200);
  while (!Serial) {
    delay(100);
  }

  Serial.println("O2-CO2 Gas Analyser Starting...");
  
  // LED init
  led.begin();
  led.clear();
  led.setPixelColor(0, LED_YELLOW);
  led.show();

  // Analog init
  analogReference(AR_EXTERNAL);
  analogReadResolution(12);
  ntcAmb.begin();
  ntcHeat.begin();

  // O2 sensor init
  O2.begin();

  // CO2 sensor init
  CO2.begin();

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
    while (1);
  }
  Serial.print("serialNumber: ");
  Serial.print(serialNumber);
  Serial.println();

  timeStamp = millis();

  led.setPixelColor(0, LED_GREEN);
  led.show();
}

void loop() {
  float shtTemperatureC = 0.0;
  float shtHumidity = 0.0;
  sht.measureHighPrecision(shtTemperatureC, shtHumidity);

  if (CO2.manage()) {
    led.setPixelColor(0, LED_CYAN);
    led.show();
  }

  if (millis() - timeStamp >= 1000) {
    timeStamp = millis();
    
    float temperatureAmbC = ntcAmb.temperature();
    float temperatureHeatC = ntcHeat.temperature();
    float o2Percent = O2.readO2(temperatureAmbC);
    uint32_t co2Ppm = CO2.CO2();

    Serial.printf("T Ambient: %.2f °C | T Heater: %.2f °C | T Gas: %.2f °C | RH Gas: %.2f %% | O2: %.2f %% | CO2: %u ppm\r\n",
                  temperatureAmbC,
                  temperatureHeatC,
                  shtTemperatureC,
                  shtHumidity,
                  o2Percent,
                  co2Ppm);
    led.setPixelColor(0, LED_GREEN);
    led.show();
  }
}