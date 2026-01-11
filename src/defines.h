#pragma once

#include <Arduino.h>
#include "debug/debug.h"
// Lib includes
#include <wdt_samd21.h>
#include <Adafruit_NeoPixel_ZeroDMA.h>
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
//Adafruit_NeoPixel led(1, PIN_LED_DAT, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel_ZeroDMA led(1, PIN_LED_DAT, NEO_GRB + NEO_KHZ800);
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
#define LED_AMBER 0xFFAA00
#define LED_CYAN 0x00FFFF
#define LED_MAGENTA 0xFF00FF
#define LED_WHITE 0xFFFFFF
#define LED_OFF 0x000000

// States and LED colours
#define STATE_NORMAL 0
#define STATE_STARTUP 1
#define STATE_ERROR 2
#define STATE_CALIBRATING_O2 3
#define STATE_TEMP_UNSTABLE 4

#define STATE_NORMAL_COLOUR LED_GREEN
#define STATE_STARTUP_COLOUR LED_MAGENTA
#define STATE_ERROR_COLOUR LED_RED
#define STATE_CALIBRATING_O2_COLOUR LED_BLUE
#define STATE_TEMP_UNSTABLE_COLOUR LED_AMBER

#define STATE_NORMAL_BLINK_Hz 1
#define STATE_STARTUP_BLINK_Hz 1
#define STATE_ERROR_BLINK_Hz 0.5
#define STATE_CALIBRATING_O2_BLINK_Hz 0.5
#define STATE_TEMP_UNSTABLE_BLINK_Hz 0.5

struct sysState {
    uint8_t state;
    uint32_t colour[5] = {  STATE_NORMAL_COLOUR,
                            STATE_STARTUP_COLOUR,
                            STATE_ERROR_COLOUR,
                            STATE_CALIBRATING_O2_COLOUR,
                            STATE_TEMP_UNSTABLE_COLOUR  };
    float blinkHz[5] = {    STATE_NORMAL_BLINK_Hz,
                            STATE_STARTUP_BLINK_Hz,
                            STATE_ERROR_BLINK_Hz,
                            STATE_CALIBRATING_O2_BLINK_Hz,
                            STATE_TEMP_UNSTABLE_BLINK_Hz  };
} systemState;

// staus register definitions
#define STATUS_SYSTEM_bp 0
#define STATUS_O2_SENSOR_bp 4
#define STATUS_CO2_SENSOR_bp 8
#define STATUS_HEATER_bp 12
#define STATUS_GAS_TEMP_HUMIDITY_bp 16
#define STATUS_AMB_TEMP_bp 20
#define STATUS_WDT_RESETS_bp 24

// System status values
#define STATUS_SYSTEM_OK_bm 0x01
#define STATUS_SYSTEM_WDT_RESET_bm 0x02

// O2 sensor status values
#define STATUS_O2_SENSOR_OK_bm 0x01
#define STATUS_O2_SENSOR_TOO_HIGH_bm 0x02
#define STATUS_O2_SENSOR_NOT_CONNECTED_bm 0x04

// CO2 sensor status values
#define STATUS_CO2_SENSOR_OK_bm 0x01
#define STATUS_CO2_SENSOR_NOT_CONNECTED_bm 0x02
#define STATUS_CO2_SENSOR_ERROR_bm 0x04

// Heater status values
#define STATUS_HEATER_OK_bm 0x01
#define STATUS_HEATER_TEMP_UNSTABLE_bm 0x02
#define STATUS_HEATER_SENSOR_ERROR_bm 0x04

// Gas temp and humidity sensor status values
#define STATUS_GAS_TEMP_HUMIDITY_OK_bm 0x01
#define STATUS_GAS_TEMP_HUMIDITY_SENOSR_ERROR_bm 0x02

// Ambient temp sensor status values
#define STATUS_AMB_TEMP_OK_bm 0x01
#define STATUS_AMB_TEMP_SENSOR_ERROR_bm 0x02

// CO2 constants
#define CO2_MAX_DATA_AGE_MS 3000

// EEPROM
#define EEPROM_VERSION_BYTE 0xA0

// Config array structure
struct Config {
    uint32_t wdtTimerResetCounter = 0;
    uint32_t modbusBaudrate = 9600;
    uint16_t modbusConfig = SERIAL_8N1;
    uint8_t modbusSlaveID = 100;
    uint8_t modbus120Rterm = 0;
    float heaterKp = 100.0;
    float heaterKi = 0.5;
    float heaterKd = 0.0;
    float heaterMaxI = 90.0;
    float heaterSetpointC = 30.0;
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
    uint32_t status = 0;            // 0-1: System status register
    uint16_t modbusSlaveID = 100;   // 2: Modbus slave ID
    uint16_t modbusBaudrate = 960;  // 3: Modbus baudrate/10 to fit inside uint16
    uint16_t modbusStopBits = 1;    // 4: Modbus stop bits
    uint16_t modbusParity = 0;      // 5: Modbus parity
    uint16_t modbus120Rterm = 0;    // 6: Modbus 120R termination resistor enabled/disabled
    uint16_t reserved1 = 0;         // 7: Reserved for future use
    uint16_t O2SetZeroPoint = 0;    // 8: O2 sensor zero point calibration
    uint16_t CO2SetZeroPoint = 0;   // 9: CO2 sensor zero point calibration
    float O2SetXPoint = 0;          // 10: O2 sensor span calibration point
    float CO2SetXPoint = 0;         // 12: CO2 sensor span calibration point
    float heaterKp = 0;             // 14: Heater proportional gain
    float heaterKi = 0;             // 16: Heater integral gain
    float heaterKd = 0;             // 18: Heater derivative gain
    float heaterMaxI = 0;           // 20: Heater maximum integral term
    float heaterSetpointC = 0;      // 22: Heater temperature setpoint in °C
    float heaterMinAmbientDeltaC = 0;  // 24: Heater minimum ambient delta in °C
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