#pragma once

#include <Arduino.h>
// Lib includes
#include <Adafruit_NeoPixel.h>
#include <SensirionI2cSht4x.h>
#include <Wire.h>
#include "SGX_4OX.h"
#include "SGX_INIR2_CD100.h"
#include "NTC_Therm.h"
#include "PID_Control.h"
#include "modbus-rtu-slave.h"

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

// Analog defines
#define NTC_MOVING_AVERAGE_SAMPLES 10
#define NTC_OVERSAMPLING_SAMPLES 10

// Lib Objects
Adafruit_NeoPixel led(1, PIN_LED_DAT, NEO_GRB + NEO_KHZ800);
SensirionI2cSht4x sht;
SGX40X O2(PIN_V_O2_SENS, 12, 2.048, 100.0, 100.0);
CD100 CO2(Serial2);
NTC_Therm ntcHeat(PIN_NTC_HEAT, 10000.0, 10000.0, 4100.0, true, 12, 2.048);
NTC_Therm ntcAmb(PIN_NTC_AMB, 10000.0, 10000.0, 4100.0, true, 12, 2.048);
PID_Control heaterCtrl(PIN_HEAT_PWM, true);
ModbusRTUSlave modbus(Serial1, PIN_RS485_DE);

// LED colours
#define LED_RED 0xFF0000
#define LED_GREEN 0x00FF00
#define LED_BLUE 0x0000FF
#define LED_YELLOW 0xAAFF00
#define LED_AMBER 0xFFFF00
#define LED_CYAN 0x00FFFF
#define LED_MAGENTA 0xFF00FF
#define LED_WHITE 0xFFFFFF
#define LED_OFF 0x000000

// EEPROM
#define EEPROM_VERSION_BYTE 0xA0

// Config array structure
struct Config {
    uint32_t modbusBaudrate = 9600;
    uint16_t modbusConfig = SERIAL_8N1;
    uint8_t modbusSlaveID = 100;
    uint8_t modbus120Rterm = 0;
    float heaterKp = 100.0;
    float heaterKi = 0.5;
    float heaterKd = 0.0;
    float heaterMaxI = 90.0;
    float heaterSetpointC = 35.0;
    float heaterMinAmbientDeltaC = 5.0;
    float o2CalibrationOffset = 0.06;
    float o2CalibrationScale = 0.2;
    int32_t co2CalibrationOffset = 0;
    float co2CalibrationScale = 1.0;
    float ntcAmbOffsetC = 0.0;
    float ntcAmbScale = 1.0;
    float ntcHeatOffsetC = 0.0;
    float ntcHeatScale = 1.0;
};

Config config;

// Modbus structures
struct ModbusHoldingRegisters {
    uint16_t modbusSlaveID = 100;
    uint16_t modbusBaudrate = 960; // baudrate/10 to fit inside uint16
    uint16_t modbusStopBits = 1;
    uint16_t modbusParity = 0;
    uint16_t modbus120Rterm = 0;
    uint16_t O2SetZeroPoint = 0;
    float O2SetXPoint = 0;
    uint16_t CO2SetZeroPoint = 0;
    float CO2SetXPoint = 0;
    float heaterKp = 0;
    float heaterKi = 0;
    float heaterKd = 0;
    float heaterMaxI = 0;
    float heaterSetpointC = 0;
    float heaterMinAmbientDeltaC = 0;
};

struct ModbusInputRegisters {
    float O2percent = 0.0;
    float CO2percent = 0.0;
    float CO2ppm = 0;
    float gasTempC = 0.0;
    float gasRH = 0.0;
    float gasAH = 0.0;
    float gasVP = 0.0;
    float ambTempC = 0.0;
    float heatTempC = 0.0;
};

#define NUM_HOLDING_REGISTERS 50
#define NUM_INPUT_REGISTERS 50

ModbusHoldingRegisters modbusHoldingRegisters;
ModbusInputRegisters modbusInputRegisters;
uint16_t holding[NUM_HOLDING_REGISTERS] = {0};
uint16_t input[NUM_INPUT_REGISTERS] = {0};