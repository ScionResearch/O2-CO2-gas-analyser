# O2-CO2 Gas Analyser - Connector Pinout

## 8-Pin Front Panel Connector

**Connector Type:** A-Cod 8-pin Circular (Male)

**Mating Connector Part Number:** A-Cod 8-pin Female connector

### Pin Assignments

| Pin | Signal Name | Direction | Description |
|:---:|-------------|:---------:|-------------|
| 1 | +24V DC | In | Power supply input (24V DC) |
| 2 | GND (0V) | - | Ground reference |
| 3 | RS485 A | Bidirectional | RS-485 data line A (non-inverting) |
| 4 | RS485 B | Bidirectional | RS-485 data line B (inverting) |
| 5 | GND (0V) | - | Ground reference |
| 6 | O2 Sensor | Output | O2 sensor signal (analog) |
| 7 | GND (0V) | - | Ground reference |
| 8 | CO2 Sensor | Output | CO2 sensor signal (UART/digital) |

### Pin Numbering Reference

```
    Front View (looking at connector on device)
    Notch at top between Pin 1 and Pin 2
    
         ____________
        /             \
       | Pin2     Pin1 |
       | (0V)    (24V) |
       |      Pin8     |
       | Pin3 (CO2)Pin7|
       | (A)       (0V)|
       | Pin4      Pin6|
       | (B)       (O2)|
        \     Pin5     /
         \    (0V)    / 
           -----------
```

---

## Signal Descriptions

### Power

| Signal | Voltage | Max Current | Notes |
|--------|---------|-------------|-------|
| +24V DC (Pin 1) | 24V DC | 500mA | Input power supply |
| GND (Pins 2, 5, 7) | 0V | - | Multiple ground connections for robustness |

### RS-485 Interface (Modbus RTU)

| Signal | Description |
|--------|-------------|
| RS485 A | Non-inverting line (+), also called D+ or TxD+/RxD+ |
| RS485 B | Inverting line (-), also called D- or TxD-/RxD- |

**Default Modbus Settings:**
- Slave ID: 100
- Baud Rate: 9600
- Data Bits: 8
- Parity: None
- Stop Bits: 1

### Sensor Outputs

| Signal | Pin | Type | Voltage | Description |
|--------|-----|------|---------|-------------|
| O2 Sensor | 6 | Analog Output | 0-2.048V | Electrochemical sensor output (SGX 4OX) |
| CO2 Sensor | 8 | UART/Digital | 3.3V logic | Infrared sensor output (SGX INIR2-CD100) |

---

## Cable Wiring Reference

Your 6-wire cable connections:

| Wire Color | Signal | Device Pin | Connection Description |
|------------|--------|:----------:|------------------------|
| **Red** | +24V DC | Pin 1 | Power supply positive |
| **Black** | GND (0V) | Pin 2/5/7 | Ground reference |
| **White** | RS485 A | Pin 3 | Modbus A line (non-inverting, +) |
| **Blue** | RS485 B | Pin 4 | Modbus B line (inverting, -) |
| **Green** | O2 Sensor | Pin 6 | O2 analog output |
| **Yellow** | CO2 Sensor | Pin 8 | CO2 sensor output |

## Wiring Example

```
Device Connector         Cable Wires         Destination
─────────────────       ──────────       ──────────────────
Pin 1 (+24V DC)  ──(Red)──────►   +24V Power Supply
Pin 2 (GND)      ──(Black)──────►  0V / Ground Reference
Pin 3 (RS485 A)  ──(White)──────►  RS-485 A line (+)
Pin 4 (RS485 B)  ──(Blue)───────►  RS-485 B line (-)
Pin 6 (O2)       ──(Green)──────►  O2 sensor input (ADC)
Pin 8 (CO2)      ──(Yellow)─────►  CO2 sensor input
```

### RS-485 Connection Details

The device uses RS-485 (Modbus RTU) for communication via your 6-wire cable:
- **White wire (Pin 3 - RS485 A)**: Connect to RS-485 "A" or "+" line from your master device
- **Blue wire (Pin 4 - RS485 B)**: Connect to RS-485 "B" or "-" line from your master device
- Ensure twisted-pair shielded cable is used for RS-485 connections for noise immunity
- Connect the cable shield to the Black (GND) wire at the controller end only
- For bus termination: Connect 120Ω resistor between A and B lines at the end of the bus (if enabled via register 6)

---

## Notes

- Connector type is A-Cod 8-pin circular male connector on the device
- Ensure proper orientation using the notch marking at the top of the connector
- Multiple ground pins (2, 5, 7) provide robust grounding for noise immunity
- Pin 8 (CO2 sensor) is the center pin of the connector
- RS-485 signals (Pins 3 & 4) should use twisted-pair shielded cable
- 24V DC supply should be clean and stable (use a good quality power supply)
- Modbus communication settings: 9600 baud, 8 data bits, 1 stop bit, no parity, Slave ID 100 (default) 

---

## Revision History

| Date | Version | Changes | Author |
|------|---------|---------|--------|
| 2026-01-15 | 1.1 | Added cable wire color mapping for 6-wire configuration | User |
| 2026-01-15 | 1.0 | Initial pinout documentation - A-Cod 8-pin connector traced | User |

