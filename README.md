# O2-CO2 Gas Analyser

A precision gas analysis system built on the SAMD21 microcontroller platform, featuring simultaneous O2 and CO2 measurement with temperature-controlled sampling chamber and Modbus RTU communication.

![Gas Analyser](images/analyser.png)

## Features

- **O2 Measurement**: SGX 4OX electrochemical oxygen sensor with temperature compensation
- **CO2 Measurement**: SGX INIR2-CD100 infrared CO2 sensor (0-10% range)
- **Temperature Control**: PID-controlled heater maintaining stable measurement conditions
- **Humidity Sensing**: Sensirion SHT4x for gas temperature and relative humidity
- **Communication**: Modbus RTU over RS-485 with configurable baud rate and addressing
- **Status Monitoring**: Comprehensive status register for system health monitoring
- **Calibration**: In-field zero and span calibration for both O2 and CO2 sensors

## Hardware

- **MCU**: SAMD21-based custom board (gas_analyser_m0)
- **O2 Sensor**: SGX 4OX (galvanic cell, 70-130 uA in air)
- **CO2 Sensor**: SGX INIR2-CD100 (NDIR, UART interface)
- **Temperature/Humidity**: Sensirion SHT40
- **Heater**: PWM-controlled with NTC thermistor feedback
- **Interface**: RS-485 with optional 120 Ohm termination

---

## Modbus RTU Interface

### Default Communication Settings

| Parameter | Default Value |
|-----------|---------------|
| Slave ID | 100 |
| Baud Rate | 19200 |
| Data Bits | 8 |
| Parity | Even |
| Stop Bits | 2 |

These defaults match the CM5 ORC RS-485 Port 2 bus (19200 8E2). Keep the ORC
port settings unchanged when it is shared with other devices.

### Supported Function Codes

| Function Code | Description |
|---------------|-------------|
| 03 | Read Holding Registers |
| 04 | Read Input Registers |
| 06 | Write Single Holding Register |
| 16 | Write Multiple Holding Registers |

---

## Register Definitions

Register addresses are zero-based. Multi-register IEEE-754 floats use
low-word-first order: the lower address contains the low 16-bit word. Bytes
within each 16-bit Modbus register remain in standard Modbus order.

### Input Registers (Function Code 04) - Read Only

Input registers 0-17 contain sensor measurements; 18-37 append calibration
status and active calibration/configuration values. Floating-point values are
IEEE-754 single-precision (32-bit) values spanning two consecutive registers.

| Address | Name | Type | Unit | Description |
|---------|------|------|------|-------------|
| 0-1 | O2percent | float | % | Oxygen concentration |
| 2-3 | CO2percent | float | % | CO2 concentration (as percentage) |
| 4-5 | CO2ppm | float | ppm | CO2 concentration (as ppm) |
| 6-7 | gasTempC | float | C | Gas stream temperature (SHT40) |
| 8-9 | gasRH | float | %RH | Gas stream relative humidity |
| 10-11 | gasAH | float | g/m3 | Absolute humidity |
| 12-13 | gasVP | float | Pa | Vapour pressure |
| 14-15 | ambTempC | float | C | Ambient temperature (NTC) |
| 16-17 | heatTempC | float | C | Heater block temperature (NTC) |
| 18 | calibrationMagic | uint16 | - | Protocol marker `0xCA01` |
| 19 | calibrationState | uint16 | - | 0=idle, 1=running, 2=succeeded, 3=failed |
| 20 | calibrationError | uint16 | - | 0=none, 1=did not stabilise, 2=current below min, 3=current above max, 4=unknown, 5=storage failure, 6=unsupported |
| 21 | calibrationOperation | uint16 | - | 0=none, 1=O2 zero, 2=O2 span, 3=CO2 zero, 4=CO2 span, 5=heater setpoint |
| 22 | calibrationSequence | uint16 | - | Increments for each accepted operation; wraps at 16 bits |
| 23 | calibrationFlags | uint16 | - | Bit 0=latest O2 save/commit calls completed; bit 1=config save pending |
| 24-25 | calibrationElapsedMs | uint32 | ms | Elapsed time for the current/last operation; wraps at 32 bits |
| 26-27 | o2CalibrationOffset | float | uA | Active O2 zero-current coefficient |
| 28-29 | o2CalibrationScale | float | - | Active O2 span coefficient |
| 30-31 | co2CalibrationOffset | float | ppm | Active CO2 software offset |
| 32-33 | co2CalibrationScale | float | - | Active CO2 scale |
| 34-35 | activeHeaterSetpointC | float | C | Current heater setpoint |
| 36-37 | committedHeaterSetpointC | float | C | Heater setpoint last recorded after an EEPROM commit call |

The calibration status is volatile: after boot it starts idle with sequence 0
and does not invent a previous calibration date/result. Coefficients and heater
values report the values loaded/active in this boot. The sequence, operation,
state, error, and elapsed time remain available until another accepted
operation or reboot.

The O2 result is based on the sensor library's completed stability algorithm,
not on a register write or a changed reading. Library error 1 means the reading
did not stabilise, 2 means current below the minimum, and 3 means current above
the maximum; other nonzero library results map to error 4. Flag bit 0 clears
when a new O2 operation is accepted and sets only after that operation succeeds
and its `EEPROM.put()` and `EEPROM.commit()` calls return. The EEPROM
API does not report a flash-write result, so this flag is not an independent
power-cycle persistence check. Error 5 is reserved because the current EEPROM
API has no returned failure status. Flag bit 1 marks a config save waiting for
the firmware's five-second save window; it clears when the commit call completes.
Heater configuration writes are ignored while an O2 zero/span operation is
running (and in the same holding-register request that starts one); the last
accepted heater settings remain active.
The unchanged EEPROM `Config` structure and holding-register layout are
preserved.

### Holding Registers (Function Code 03/06/16) - Read/Write

Holding registers contain configuration and calibration parameters. Changes are automatically saved to non-volatile memory.

| Address | Name | Type | Unit | Description |
|---------|------|------|------|-------------|
| 0-1 | status | uint32 | - | System status register (see below) |
| 2 | modbusSlaveID | uint16 | - | Modbus slave address (2-246) |
| 3 | modbusBaudrate | uint16 | x10 | Baud rate / 10 (e.g., 960 = 9600) |
| 4 | modbusStopBits | uint16 | - | Stop bits (1-3) |
| 5 | modbusParity | uint16 | - | Parity: 1=Even, 2=Odd, 3=None |
| 6 | modbus120Rterm | uint16 | - | RS-485 termination: 0=Off, 1=On |
| 7 | reserved1 | uint16 | - | Reserved for future use |
| 8 | O2SetZeroPoint | uint16 | - | Write 1 to start O2 zero cal |
| 9 | CO2SetZeroPoint | uint16 | ppm | CO2 zero cal reference (1-4999) |
| 10-11 | O2SetXPoint | float | % | O2 span cal point (10.0-25.0) |
| 12-13 | CO2SetXPoint | float | ppm | CO2 span cal point (300-50000) |
| 14-15 | heaterKp | float | - | PID proportional gain |
| 16-17 | heaterKi | float | - | PID integral gain |
| 18-19 | heaterKd | float | - | PID derivative gain |
| 20-21 | heaterMaxI | float | - | PID integral limit (0-255) |
| 22-23 | heaterSetpointC | float | C | Heater setpoint (20.0-50.0) |
| 24-25 | heaterMinAmbientDeltaC | float | C | Min delta above ambient (0-20) |

#### Valid Baud Rate Values

| Register Value | Actual Baud Rate |
|----------------|------------------|
| 960 | 9600 |
| 1920 | 19200 |
| 3840 | 38400 |
| 5760 | 57600 |
| 11520 | 115200 |

---

## Status Register

The status register (holding registers 0-1) is a 32-bit value containing system health information. Each 4-bit nibble represents a different subsystem.

### Status Register Bit Layout

```
Bits 31-24       23-20      19-16               15-12       11-8          7-4    3-0
     [WDT Count] [Amb Temp] [Gas Temp/Humidity] [Heater]    [CO2 Sensor]  [O2]   [System]
```

### Subsystem Status Definitions

#### System Status (Bits 3-0)
| Value | Meaning |
|-------|---------|
| 0x01 | System OK |
| 0x02 | WDT Reset Occurred |
| 0x03 | System OK + WDT Reset History |

#### O2 Sensor Status (Bits 7-4)
| Value | Meaning |
|-------|---------|
| 0x01 | Sensor OK |
| 0x02 | Reading too high (>25%) |
| 0x04 | Sensor not connected (<1%) |

#### CO2 Sensor Status (Bits 11-8)
| Value | Meaning |
|-------|---------|
| 0x01 | Sensor OK |
| 0x02 | Sensor not connected (no data) |
| 0x04 | Sensor communication error |

#### Heater Status (Bits 15-12)
| Value | Meaning |
|-------|---------|
| 0x01 | Temperature stable |
| 0x02 | Temperature unstable (>0.2C from setpoint) |
| 0x04 | Temperature sensor error |

#### Gas Temperature/Humidity Status (Bits 19-16)
| Value | Meaning |
|-------|---------|
| 0x01 | Sensor OK |
| 0x02 | Sensor error |

#### Ambient Temperature Status (Bits 23-20)
| Value | Meaning |
|-------|---------|
| 0x01 | Sensor OK |
| 0x02 | Sensor error |

#### WDT Reset Count (Bits 27-24)
Contains the number of watchdog timer resets since last EEPROM clear (0-255, saturates at 255).

---

## Usage Examples

### Reading Sensor Values (Python with pymodbus)

```python
from pymodbus.client import ModbusSerialClient
import struct

# Connect to the analyser
client = ModbusSerialClient(
    port='COM3',
    baudrate=9600,
    parity='N',
    stopbits=1,
    bytesize=8,
    timeout=1
)
client.connect()

SLAVE_ID = 100

def read_float(client, address, slave_id):
    """Read a 32-bit float from two consecutive input registers."""
    result = client.read_input_registers(address, 2, slave=slave_id)
    if result.isError():
        return None
    # Register order is low word first; bytes inside each register are big-endian.
    raw = (result.registers[1] << 16) | result.registers[0]
    return struct.unpack('>f', struct.pack('>I', raw))[0]

# Read all sensor values
o2_percent = read_float(client, 0, SLAVE_ID)
co2_percent = read_float(client, 2, SLAVE_ID)
co2_ppm = read_float(client, 4, SLAVE_ID)
gas_temp = read_float(client, 6, SLAVE_ID)
gas_rh = read_float(client, 8, SLAVE_ID)
heater_temp = read_float(client, 16, SLAVE_ID)

print(f"O2: {o2_percent:.2f}%")
print(f"CO2: {co2_ppm:.0f} ppm ({co2_percent:.3f}%)")
print(f"Gas Temperature: {gas_temp:.1f}C")
print(f"Gas RH: {gas_rh:.1f}%")
print(f"Heater Temperature: {heater_temp:.1f}C")

client.close()
```

### Parsing the Status Register

```python
def parse_status(client, slave_id):
    """Read and parse the status register."""
    result = client.read_holding_registers(0, 2, slave=slave_id)
    if result.isError():
        return None
    
    status = result.registers[0] | (result.registers[1] << 16)
    
    # Extract each nibble
    system_status = status & 0x0F
    o2_status = (status >> 4) & 0x0F
    co2_status = (status >> 8) & 0x0F
    heater_status = (status >> 12) & 0x0F
    gas_th_status = (status >> 16) & 0x0F
    amb_temp_status = (status >> 20) & 0x0F
    wdt_count = (status >> 24) & 0xFF
    
    # Interpret statuses
    status_names = {
        'system': {0x01: 'OK', 0x02: 'WDT Reset', 0x03: 'OK (WDT history)'},
        'o2': {0x01: 'OK', 0x02: 'Too High', 0x04: 'Not Connected'},
        'co2': {0x01: 'OK', 0x02: 'Not Connected', 0x04: 'Error'},
        'heater': {0x01: 'Stable', 0x02: 'Unstable', 0x04: 'Sensor Error'},
        'gas_th': {0x01: 'OK', 0x02: 'Error'},
        'amb_temp': {0x01: 'OK', 0x02: 'Error'}
    }
    
    print(f"System: {status_names['system'].get(system_status, 'Unknown')}")
    print(f"O2 Sensor: {status_names['o2'].get(o2_status, 'Unknown')}")
    print(f"CO2 Sensor: {status_names['co2'].get(co2_status, 'Unknown')}")
    print(f"Heater: {status_names['heater'].get(heater_status, 'Unknown')}")
    print(f"Gas Temp/RH: {status_names['gas_th'].get(gas_th_status, 'Unknown')}")
    print(f"Ambient Temp: {status_names['amb_temp'].get(amb_temp_status, 'Unknown')}")
    print(f"WDT Reset Count: {wdt_count}")
    
    return status

parse_status(client, SLAVE_ID)
```

### Writing Configuration (Changing Heater Setpoint)

```python
def write_float(client, address, value, slave_id):
    """Write a 32-bit float to two consecutive holding registers."""
    # The lower address contains the low 16-bit word.
    raw = struct.unpack('>I', struct.pack('>f', value))[0]
    low_word = raw & 0xFFFF
    high_word = (raw >> 16) & 0xFFFF
    
    # Write both registers
    result = client.write_registers(address, [low_word, high_word], slave=slave_id)
    return not result.isError()

# Set heater setpoint to 40C
if write_float(client, 22, 40.0, SLAVE_ID):
    print("Heater setpoint updated to 40C")
else:
    print("Failed to update heater setpoint")
```

### Changing Modbus Settings

```python
# Change slave ID to 50
client.write_register(2, 50, slave=SLAVE_ID)

# Change baud rate to 19200 (write 1920 = 19200/10)
client.write_register(3, 1920, slave=SLAVE_ID)

# Enable 120 Ohm termination
client.write_register(6, 1, slave=SLAVE_ID)

# Note: After changing communication settings, 
# you will need to reconnect with the new parameters
```

---

## Calibration Procedures

### O2 Sensor Calibration

The O2 sensor supports two-point calibration: zero (0% O2) and span (typically 20.9% in air).

#### Zero Calibration (0% O2)

1. Flow nitrogen or other zero-O2 gas through the sensor
2. Wait for readings to stabilise
3. Write `1` to register 8 (`O2SetZeroPoint`)

```python
# Start O2 zero calibration
client.write_register(8, 1, slave=SLAVE_ID)
# Read input registers 18-23 for live state, error, operation, sequence, and flags.
# Calibration takes up to 60 seconds; poll state until it is 2 (success) or 3 (failure).
```

#### Span Calibration (e.g., 20.9% O2)

1. Flow calibration gas (e.g., ambient air at 20.9% O2)
2. Wait for readings to stabilise
3. Write the known O2 percentage as a float to registers 10-11

```python
# Calibrate with 20.9% O2 (ambient air)
write_float(client, 10, 20.95, SLAVE_ID)
# Poll input registers 18-23 for completion and input registers 26-29 for the
# active coefficients. A successful save sets calibrationFlags bit 0.
```

**Calibration Requirements:**
- Sensor current must be within 70-130 uA for air
- Reading must stabilise (+/-0.1 uA for 5 consecutive readings)
- Maximum calibration time: 60 seconds
- Read the appended input-register status (18-23) for progress and the actual
  algorithm result. The USB console remains available for diagnostics.

### Engineer Verification Script

Install the only Python dependency:

```bash
python3 -m pip install minimalmodbus
```

The analyser's USB CDC connection is a diagnostic console, not Modbus. The
`--port` below must be a USB-RS485 adapter connected to the analyser's RS-485
bus. Do not connect a second Modbus master while the ORC is using the same bus.
Use the USB console separately to inspect calibration completion logs. The
script defaults to the analyser firmware's 19200 baud, 8 data bits, even parity,
and two stop bits; use `--baudrate`, `--parity`, or `--stopbits` only when the
device has a different saved Modbus configuration.

Inspect live measurements, heater setpoint, and status bits without writing:

```bash
python3 scripts/verify_modbus.py --port /dev/ttyUSB0
```

Run a reversible heater-setpoint test with a target within 2 C of the current
setpoint. The script checks holding-register readback and Modbus responsiveness
for at least 5.5 seconds, then restores the original value:

```bash
python3 scripts/verify_modbus.py --port /dev/ttyUSB0 \
  --heater-setpoint-test 30.5 --confirm-heater-write
```

Add `--verify-persistence` to power-cycle the analyser and verify the test
setpoint survived reboot before restoring the original setting.

To trigger an ambient-air O2 span calibration, ensure ambient air is flowing
and stable before running:

```bash
python3 scripts/verify_modbus.py --port /dev/ttyUSB0 \
  --o2-span-air 20.95 --confirm-o2-span
```

Span calibration updates O2 calibration values. The script polls the firmware
operation/result registers and O2 readback for up to 60 seconds. Add
`--verify-persistence` to confirm the successful-save message,
power-cycle while ambient air remains stable, confirm the startup console shows
the saved span coefficient, and verify that Modbus O2 readback remains stable
near the span point after reboot.

Run the host-side verifier tests without hardware or serial dependencies:

```bash
python3 -m unittest discover -s tests -v
```

The heater test also polls the restored setpoint through the firmware's
five-second save window before returning. Readback is not proof of persistence;
only the explicit power-cycle check verifies persistence of the test value.

### CO2 Sensor Calibration

#### Zero Calibration

1. Flow nitrogen or zero-CO2 gas through the sensor
2. Wait for readings to stabilise
3. Write the expected zero reading (typically 0-400 ppm) to register 9

```python
# Set a 400 ppm reference when the sensor reads 450 ppm
client.write_register(9, 400, slave=SLAVE_ID)
```

#### Span Calibration

Span calibration for CO2 is not implemented. A valid request in registers
12-13 is reported as operation 4, failed with error 6 (unsupported).

Writing zero to CO2 zero-reference register 9 is a no-op; it does not start a
calibration or change the status sequence. A nonzero reference currently
applies the software offset to the live CO2 reading and reports operation 3,
but that offset is volatile and is not written to the EEPROM `Config`.

---

## LED Status Indicator

The analyser has an RGB LED that indicates system state:

| Colour | Pattern | State |
|--------|---------|-------|
| Green | 1 Hz blink | Normal operation |
| Magenta | 1 Hz blink | Startup |
| Amber | 0.5 Hz blink | Temperature unstable |
| Blue | 0.5 Hz blink | O2 calibration in progress |
| Red | 0.5 Hz blink | Error condition |

---

## Building and Flashing

This project uses PlatformIO. The NTC thermistor library is vendored in `lib/`;
no submodule initialization or SAMD core patch is required.

```bash
# Build
pio run -e gas_analyser_m0

# Upload
pio run -e gas_analyser_m0 --target upload
```

Confirm the upload port belongs to the gas analyser, not an ORC MCU, before
uploading. USB CDC is diagnostic output at 115200 baud; opening it at 1200 baud
requests bootloader entry. The UF2 bootloader identifies as `GAS ANALYSER M0`,
Board-ID `SAMD21E18A-GA-M0`, and mounts as `GA_BOOT`. Application firmware
identifies as `BSI Gas Analyser M0`.

USB enumeration alone does not prove sensor initialization or main-loop health.
Capture the startup console to check initialization and configuration. The
current full-image UF2 includes the flash-backed EEPROM area: a bench upload
reported `Got EEPROM version: 0xFF, expected: 0xA0` and wrote default
configuration/calibration. An unchanged `EEPROM_VERSION_BYTE` does not prevent
this. Save calibration/configuration before uploading, and plan explicit
restoration or recalibration before measurement use; do not assume an upload
preserves calibration.

A bootloader `CURRENT.UF2` readback can help recover stored configuration, but
is not automatically a bootable rollback image. The bench readback had an erased
application vector-table block; validate a recovery image before using it.

### Dependencies

All libraries are included in the `lib/` folder:
- Adafruit NeoPixel ZeroDMA
- Sensirion I2C SHT4x
- FlashStorage SAMD
- Custom libraries: SGX_4OX, SGX_INIR2_CD100, NTC_Therm, PID_Control, modbus-rtu-slave

---

## Troubleshooting

### No Modbus Response
- Check RS-485 wiring (A/B polarity)
- Verify baud rate and slave ID
- Check termination resistor setting if at end of bus

### O2 Reading Shows 0%
- Check O2 sensor connection
- Verify sensor current is within range (status register)
- Sensor may need replacement if current is <70 uA in air

### Temperature Unstable (Amber LED)
- Allow 5+ minutes for heater to stabilise after power-on
- Check ambient temperature is not too close to setpoint
- Verify heater NTC is properly installed

### CO2 Sensor Not Responding
- Check UART connection (38400 baud)
- Verify sensor is powered and warmed up
- Check status register for sensor errors

---

## License

This project is provided as-is for reference and educational purposes.
