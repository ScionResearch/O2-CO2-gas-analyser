#!/usr/bin/env python3
"""Inspect or explicitly verify an O2-CO2 gas analyser over Modbus RTU."""

import argparse
import math
import struct
import time


INPUT_O2_PERCENT = 0
INPUT_GAS_TEMP_C = 6
INPUT_HEATER_TEMP_C = 16
HOLDING_STATUS = 0
HOLDING_O2_SPAN = 10
HOLDING_HEATER_SETPOINT = 22
INPUT_CALIBRATION_STATUS = 18
INPUT_CALIBRATION_ELAPSED_MS = 24
CALIBRATION_MAGIC = 0xCA01
CALIBRATION_RUNNING = 1
CALIBRATION_SUCCEEDED = 2
CALIBRATION_FAILED = 3
CALIBRATION_OP_O2_SPAN = 2
CALIBRATION_FLAG_O2_COMMIT_COMPLETED = 1

CALIBRATION_ERRORS = {
    0: "none",
    1: "did not stabilise",
    2: "current below minimum",
    3: "current above maximum",
    4: "unknown error",
    5: "storage failure",
    6: "unsupported",
}

STATUS_NAMES = {
    "system": {0x01: "OK", 0x02: "watchdog reset", 0x03: "OK (watchdog history)"},
    "o2": {0x01: "OK", 0x02: "too high", 0x04: "not connected"},
    "co2": {0x01: "OK", 0x02: "not connected", 0x04: "sensor error"},
    "heater": {0x01: "stable", 0x02: "unstable", 0x04: "sensor error"},
    "gas_temp_humidity": {0x01: "OK", 0x02: "sensor error"},
    "ambient_temp": {0x01: "OK", 0x02: "sensor error"},
}


def float_to_registers(value):
    """Encode an IEEE-754 float as low-word-first Modbus registers."""
    if not math.isfinite(value):
        raise ValueError("float value must be finite")
    return list(struct.unpack("<HH", struct.pack("<f", value)))


def registers_to_float(registers):
    """Decode two low-word-first Modbus registers as an IEEE-754 float."""
    if len(registers) != 2:
        raise ValueError("a float requires exactly two Modbus registers")
    if any(not 0 <= register <= 0xFFFF for register in registers):
        raise ValueError("Modbus registers must be unsigned 16-bit values")
    return struct.unpack("<f", struct.pack("<HH", *registers))[0]


def status_fields(registers):
    """Decode the 32-bit status register, stored low word at address zero."""
    if len(registers) != 2:
        raise ValueError("status requires exactly two Modbus registers")
    raw = registers[0] | (registers[1] << 16)
    return {
        "raw": raw,
        "system": raw & 0x0F,
        "o2": (raw >> 4) & 0x0F,
        "co2": (raw >> 8) & 0x0F,
        "heater": (raw >> 12) & 0x0F,
        "gas_temp_humidity": (raw >> 16) & 0x0F,
        "ambient_temp": (raw >> 20) & 0x0F,
        "watchdog_reset_count": (raw >> 24) & 0xFF,
    }


def calibration_status_fields(registers):
    """Decode input registers 18-23 from the calibration-status contract."""
    if len(registers) != 6:
        raise ValueError("calibration status requires six Modbus registers")
    if registers[0] != CALIBRATION_MAGIC:
        raise ValueError("device does not expose the calibration-status protocol")
    return {
        "state": registers[1],
        "error": registers[2],
        "operation": registers[3],
        "sequence": registers[4],
        "flags": registers[5],
    }


def read_float(instrument, address, function_code):
    registers = instrument.read_registers(address, 2, functioncode=function_code)
    return registers_to_float(registers)


def print_snapshot(instrument):
    status = status_fields(
        instrument.read_registers(HOLDING_STATUS, 2, functioncode=3)
    )
    o2 = read_float(instrument, INPUT_O2_PERCENT, 4)
    gas_temp = read_float(instrument, INPUT_GAS_TEMP_C, 4)
    heater_temp = read_float(instrument, INPUT_HEATER_TEMP_C, 4)
    setpoint = read_float(instrument, HOLDING_HEATER_SETPOINT, 3)

    print(f"O2: {o2:.2f}% | gas temperature: {gas_temp:.2f} C")
    print(f"Heater: {heater_temp:.2f} C | setpoint: {setpoint:.2f} C")
    for field in (
        "system",
        "o2",
        "co2",
        "heater",
        "gas_temp_humidity",
        "ambient_temp",
    ):
        value = status[field]
        label = STATUS_NAMES[field].get(value, "unknown")
        print(f"Status {field}: 0x{value:X} ({label})")
    print(f"Watchdog reset count: {status['watchdog_reset_count']}")


def poll_heater_setpoint(instrument, target, wait_seconds):
    deadline = time.monotonic() + wait_seconds
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return
        time.sleep(min(1.0, remaining))
        observed = read_float(instrument, HOLDING_HEATER_SETPOINT, 3)
        status = status_fields(
            instrument.read_registers(HOLDING_STATUS, 2, functioncode=3)
        )
        o2 = read_float(instrument, INPUT_O2_PERCENT, 4)
        if not math.isclose(observed, target, abs_tol=0.1):
            raise RuntimeError(f"setpoint readback changed to {observed:.2f} C")
        print(
            f"responsive: setpoint={observed:.2f} C, O2={o2:.2f}%, "
            f"system=0x{status['system']:X}, heater=0x{status['heater']:X}"
        )


def verify_heater_setpoint(instrument, target, wait_seconds, verify_persistence):
    if not 20.0 <= target <= 50.0:
        raise ValueError("heater setpoint must be between 20 and 50 C")
    if not math.isfinite(wait_seconds) or wait_seconds < 5.5:
        raise ValueError("post-write wait must be at least 5.5 seconds")

    original = read_float(instrument, HOLDING_HEATER_SETPOINT, 3)
    if not math.isfinite(original) or not 20.0 <= original <= 50.0:
        raise RuntimeError(f"device returned an invalid original setpoint: {original}")
    if math.isclose(target, original, abs_tol=0.05):
        raise ValueError(
            f"test target {target:.2f} C must differ from current {original:.2f} C"
        )
    if abs(target - original) > 2.0:
        raise ValueError(
            "choose a test target within 2 C of the current setpoint to limit "
            "heater disturbance"
        )

    restore_required = False
    try:
        restore_required = True
        instrument.write_registers(
            HOLDING_HEATER_SETPOINT, float_to_registers(target)
        )
        print(f"Setpoint write sent: {target:.2f} C; checking response for {wait_seconds:g}s")

        poll_heater_setpoint(instrument, target, wait_seconds)

        if verify_persistence:
            input(
                "Power-cycle the analyser, wait for Modbus to return, "
                "then press Enter to verify the saved setpoint: "
            )
            observed = read_float(instrument, HOLDING_HEATER_SETPOINT, 3)
            if not math.isclose(observed, target, abs_tol=0.1):
                raise RuntimeError(
                    f"setpoint did not persist across power cycle: {observed:.2f} C"
                )
            print(f"setpoint persisted across power cycle: {observed:.2f} C")
    finally:
        if restore_required:
            instrument.write_registers(
                HOLDING_HEATER_SETPOINT, float_to_registers(original)
            )
            restored = read_float(instrument, HOLDING_HEATER_SETPOINT, 3)
            if not math.isclose(restored, original, abs_tol=0.1):
                raise RuntimeError(
                    f"failed to restore original setpoint {original:.2f} C; "
                    f"device reports {restored:.2f} C"
                )
            poll_heater_setpoint(instrument, original, wait_seconds)
            print(f"original setpoint restored through save window: {restored:.2f} C")

    print("PASS: heater setpoint write and post-write Modbus responsiveness")


def poll_o2_span(instrument, span, phase, expected_sequence=None):
    deadline = time.monotonic() + 60.0
    stable_readings = 0
    while time.monotonic() < deadline:
        time.sleep(2.0)
        calibration = calibration_status_fields(
            instrument.read_registers(INPUT_CALIBRATION_STATUS, 6, functioncode=4)
        )
        if expected_sequence is not None:
            if calibration["sequence"] != expected_sequence:
                print("Waiting for analyser to accept O2 span command.")
                continue
            if calibration["operation"] != CALIBRATION_OP_O2_SPAN:
                raise RuntimeError(
                    "analyser accepted a different calibration operation "
                    f"(operation {calibration['operation']})"
                )
            if calibration["state"] == CALIBRATION_FAILED:
                error = CALIBRATION_ERRORS.get(calibration["error"], "unknown error")
                raise RuntimeError(f"O2 span calibration failed: {error}")
            if calibration["state"] != CALIBRATION_SUCCEEDED:
                elapsed = instrument.read_registers(
                    INPUT_CALIBRATION_ELAPSED_MS, 2, functioncode=4
                )
                elapsed_ms = elapsed[0] | (elapsed[1] << 16)
                print(
                    f"O2 calibration state={calibration['state']} "
                    f"elapsed={elapsed_ms} ms"
                )
                continue
            if not calibration["flags"] & CALIBRATION_FLAG_O2_COMMIT_COMPLETED:
                raise RuntimeError(
                    "O2 calibration succeeded but firmware did not report "
                    "completed EEPROM save/commit calls"
                )

        o2 = read_float(instrument, INPUT_O2_PERCENT, 4)
        status = status_fields(
            instrument.read_registers(HOLDING_STATUS, 2, functioncode=3)
        )
        print(
            f"O2={o2:.2f}% (target {span:.2f}%), "
            f"O2 status=0x{status['o2']:X}, system=0x{status['system']:X}"
        )
        if status["o2"] == 0x01 and math.isclose(o2, span, abs_tol=0.25):
            stable_readings += 1
            if stable_readings >= 3:
                print(
                    "READBACK OK: O2 is stable near the span point. "
                    f"Verified {phase}."
                )
                return
        else:
            stable_readings = 0

    raise RuntimeError(
        f"O2 readback did not settle near the span point {phase} within "
        "60 seconds; inspect analyser USB console diagnostics"
    )


def run_o2_span(instrument, span, verify_persistence):
    if not 10.0 <= span <= 25.0:
        raise ValueError("O2 span must be between 10 and 25 percent")

    gas_temp = read_float(instrument, INPUT_GAS_TEMP_C, 4)
    if not math.isfinite(gas_temp):
        raise RuntimeError("gas temperature is invalid; refusing O2 span calibration")

    before = calibration_status_fields(
        instrument.read_registers(INPUT_CALIBRATION_STATUS, 6, functioncode=4)
    )
    if before["state"] == CALIBRATION_RUNNING:
        raise RuntimeError("an O2 calibration is already running")
    expected_sequence = (before["sequence"] + 1) & 0xFFFF

    print(
        f"Sending O2 span point {span:.2f}% to holding registers "
        f"{HOLDING_O2_SPAN}-{HOLDING_O2_SPAN + 1}."
    )
    instrument.write_registers(HOLDING_O2_SPAN, float_to_registers(span))
    print(
        "Calibration may take up to 60 seconds. Polling input registers "
        "18-23 for the actual calibration result and save status."
    )
    poll_o2_span(instrument, span, "after calibration", expected_sequence)

    if verify_persistence:
        completion_confirmed = input(
            "Did the USB console report successful calibration and saved "
            "values? Type yes to continue: "
        )
        if completion_confirmed.strip().lower() not in ("y", "yes"):
            raise RuntimeError("USB console did not confirm saved O2 calibration")

        persistence_confirmed = input(
            "Keep ambient air flowing, power-cycle the analyser, confirm the "
            "USB startup log shows the saved O2 span coefficient, wait for "
            "Modbus to return, then type yes: "
        )
        if persistence_confirmed.strip().lower() not in ("y", "yes"):
            raise RuntimeError("USB console did not confirm O2 calibration persistence")
        poll_o2_span(instrument, span, "after power cycle")


def make_parser():
    parser = argparse.ArgumentParser(
        description="Inspect or explicitly verify a gas analyser over Modbus RTU."
    )
    parser.add_argument("--port", required=True, help="RS-485 serial port, e.g. /dev/ttyUSB0")
    parser.add_argument("--slave-id", type=int, default=100)
    parser.add_argument("--baudrate", type=int, default=19200)
    parser.add_argument("--parity", choices=("N", "E", "O"), default="E")
    parser.add_argument("--stopbits", type=int, choices=(1, 2), default=2)
    parser.add_argument("--timeout", type=float, default=1.0)
    parser.add_argument(
        "--heater-setpoint-test",
        type=float,
        help="temporarily write a nearby setpoint, verify responsiveness, and restore it",
    )
    parser.add_argument(
        "--confirm-heater-write",
        action="store_true",
        help="required to run the temporary heater setpoint test",
    )
    parser.add_argument(
        "--post-write-seconds",
        type=float,
        default=6.0,
        help="time to poll after setpoint write (minimum 5.5; default 6)",
    )
    parser.add_argument(
        "--verify-persistence",
        action="store_true",
        help="prompt for a power cycle and verify the heater setpoint or O2 span persisted",
    )
    parser.add_argument(
        "--o2-span-air",
        type=float,
        nargs="?",
        const=20.95,
        help="trigger an O2 span calibration using this ambient-air reference (default: 20.95)",
    )
    parser.add_argument(
        "--confirm-o2-span",
        action="store_true",
        help="required because span calibration updates persistent O2 calibration",
    )
    return parser


def open_instrument(args):
    try:
        import minimalmodbus
    except ModuleNotFoundError as error:
        raise SystemExit(
            "Missing dependency: install it with `python3 -m pip install minimalmodbus`"
        ) from error

    instrument = minimalmodbus.Instrument(args.port, args.slave_id)
    instrument.serial.baudrate = args.baudrate
    instrument.serial.bytesize = 8
    instrument.serial.parity = args.parity
    instrument.serial.stopbits = args.stopbits
    instrument.serial.timeout = args.timeout
    instrument.clear_buffers_before_each_transaction = True
    return instrument


def main():
    parser = make_parser()
    args = parser.parse_args()

    if args.heater_setpoint_test is not None and args.o2_span_air is not None:
        parser.error("choose either a heater test or O2 span calibration, not both")
    if args.heater_setpoint_test is not None and not args.confirm_heater_write:
        parser.error("heater test requires --confirm-heater-write")
    if args.o2_span_air is not None and not args.confirm_o2_span:
        parser.error("O2 span calibration requires --confirm-o2-span")
    if (
        args.verify_persistence
        and args.heater_setpoint_test is None
        and args.o2_span_air is None
    ):
        parser.error(
            "--verify-persistence requires --heater-setpoint-test or --o2-span-air"
        )
    if args.confirm_heater_write and args.heater_setpoint_test is None:
        parser.error("--confirm-heater-write requires --heater-setpoint-test")
    if args.confirm_o2_span and args.o2_span_air is None:
        parser.error("--confirm-o2-span requires --o2-span-air")

    instrument = open_instrument(args)
    try:
        if args.heater_setpoint_test is not None:
            verify_heater_setpoint(
                instrument,
                args.heater_setpoint_test,
                args.post_write_seconds,
                args.verify_persistence,
            )
        elif args.o2_span_air is not None:
            run_o2_span(instrument, args.o2_span_air, args.verify_persistence)
        else:
            print_snapshot(instrument)
    finally:
        instrument.serial.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
