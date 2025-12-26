#pragma once

#include <Arduino.h>
// Lib includes
#include <Adafruit_NeoPixel.h>
#include <SensirionI2cSht4x.h>
#include <Wire.h>

// Pins
#define PIN_VPSU_FB 2
#define PIN_VREF 3
#define PIN_NTC_HEAT 4
#define PIN_NTC_AMB 5
#define PIN_V_O2_SENS 6
#define PIN_RS485_TERM 7
#define PIN_RS485_TX 8
#define PIN_RS485_RX 9
#define PIN_RS485_DE 10
#define PIN_HEAT_PWM 11
#define PIN_LED_DAT 14
#define PIN_GPIO_15 15
#define PIN_UART_CO2_TX 16
#define PIN_UART_CO2_RX 17
#define PIN_GPIO_18 18
#define PIN_GPIO_19 19
#define PIN_I2C_SDA 22
#define PIN_I2C_SCL 23
#define PIN_GPIO_27 27
#define PIN_GPIO_28 28

// Volatge divider constants
#define VPSU_R1 100000
#define VPSU_R2 7500
#define VPSU_V_GAIN (VPSU_R1 + VPSU_R2)/VPSU_R2
#define ADC_MAX 4095.0
#define ADC_VREF_mV  2048.0
#define ADC_mV_PER_LSB ADC_VREF_mV/ADC_MAX
#define VREF_ADC_MULTIPLIER ADC_mV_PER_LSB*VPSU_V_GAIN

// Lib Objects
Adafruit_NeoPixel led(1, PIN_LED_DAT, NEO_GRB + NEO_KHZ800);
SensirionI2cSht4x sht;