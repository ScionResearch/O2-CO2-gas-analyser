import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from scripts.verify_modbus import (
    float_to_registers,
    main,
    make_parser,
    open_instrument,
    registers_to_float,
    run_o2_span,
    status_fields,
    verify_heater_setpoint,
)


class RegisterEncodingTests(unittest.TestCase):
    def test_float_registers_use_low_word_first(self):
        self.assertEqual(float_to_registers(20.95), [0x999A, 0x41A7])

    def test_float_register_round_trip(self):
        for value in (20.0, 20.95, 30.0, 50.0):
            with self.subTest(value=value):
                self.assertAlmostEqual(
                    registers_to_float(float_to_registers(value)), value, places=5
                )

    def test_status_register_uses_low_word_first(self):
        self.assertEqual(
            status_fields([0x1421, 0x0712]),
            {
                "raw": 0x07121421,
                "system": 1,
                "o2": 2,
                "co2": 4,
                "heater": 1,
                "gas_temp_humidity": 2,
                "ambient_temp": 1,
                "watchdog_reset_count": 7,
            },
        )

    def test_o2_span_defaults_to_ambient_air(self):
        args = make_parser().parse_args(["--port", "/dev/ttyUSB0", "--o2-span-air"])
        self.assertEqual(args.o2_span_air, 20.95)

    def test_serial_defaults_match_firmware(self):
        args = make_parser().parse_args(["--port", "/dev/ttyUSB0"])
        self.assertEqual((args.baudrate, args.parity, args.stopbits), (19200, "E", 2))

    def test_open_instrument_applies_serial_framing(self):
        instrument = Mock()
        serial = Mock()
        instrument.serial = serial
        minimalmodbus = SimpleNamespace(Instrument=Mock(return_value=instrument))
        args = make_parser().parse_args(["--port", "/dev/ttyUSB0"])

        with patch.dict(sys.modules, {"minimalmodbus": minimalmodbus}):
            self.assertIs(open_instrument(args), instrument)

        minimalmodbus.Instrument.assert_called_once_with("/dev/ttyUSB0", 100)
        self.assertEqual(serial.baudrate, 19200)
        self.assertEqual(serial.bytesize, 8)
        self.assertEqual(serial.parity, "E")
        self.assertEqual(serial.stopbits, 2)

    def test_writes_require_explicit_confirmation_before_opening_port(self):
        cases = (
            ["--heater-setpoint-test", "30.5"],
            ["--o2-span-air", "20.95"],
        )
        for action in cases:
            with self.subTest(action=action):
                argv = ["verify_modbus.py", "--port", "/dev/ttyUSB0", *action]
                with patch.object(sys, "argv", argv):
                    with patch("scripts.verify_modbus.open_instrument") as open_port:
                        with self.assertRaises(SystemExit) as error:
                            main()
                self.assertEqual(error.exception.code, 2)
                open_port.assert_not_called()

    def test_persistence_requires_a_calibration_or_heater_test(self):
        argv = ["verify_modbus.py", "--port", "/dev/ttyUSB0", "--verify-persistence"]
        with patch.object(sys, "argv", argv):
            with patch("scripts.verify_modbus.open_instrument") as open_port:
                with self.assertRaises(SystemExit) as error:
                    main()
        self.assertEqual(error.exception.code, 2)
        open_port.assert_not_called()


class HeaterVerificationTests(unittest.TestCase):
    def setUp(self):
        self.instrument = Mock()
        self.setpoint = 30.0
        self.now = 0.0
        self.writes = []
        self.instrument.read_registers.side_effect = self.read_registers
        self.instrument.write_registers.side_effect = self.write_registers
        self.clock = patch(
            "scripts.verify_modbus.time.monotonic", side_effect=lambda: self.now
        )
        self.sleep = patch(
            "scripts.verify_modbus.time.sleep", side_effect=self.advance_clock
        )
        self.clock.start()
        self.sleep.start()
        self.addCleanup(self.clock.stop)
        self.addCleanup(self.sleep.stop)

    def advance_clock(self, seconds):
        self.now += seconds

    def read_registers(self, address, count, functioncode):
        if functioncode == 3 and address == 22:
            return float_to_registers(self.setpoint)
        if functioncode == 3 and address == 0:
            return [0x1111, 0x0011]
        if functioncode == 4 and address == 0:
            return float_to_registers(20.95)
        self.fail(f"unexpected read: {address}, {count}, {functioncode}")

    def write_registers(self, address, registers):
        self.assertEqual(address, 22)
        self.setpoint = registers_to_float(registers)
        self.writes.append((self.now, self.setpoint))

    def test_setpoint_is_restored_and_polled_through_save_window(self):
        verify_heater_setpoint(self.instrument, 30.5, 5.5, False)
        self.assertEqual(self.writes, [(0.0, 30.5), (5.5, 30.0)])
        self.assertEqual(self.setpoint, 30.0)
        self.assertGreaterEqual(self.now - self.writes[-1][0], 5.5)

    def test_failed_write_still_attempts_restoration(self):
        def write_then_timeout(address, registers):
            self.write_registers(address, registers)
            if len(self.writes) == 1:
                raise TimeoutError("response lost after device accepted write")

        self.instrument.write_registers.side_effect = write_then_timeout
        with self.assertRaisesRegex(TimeoutError, "response lost"):
            verify_heater_setpoint(self.instrument, 30.5, 5.5, False)
        self.assertEqual(self.setpoint, 30.0)
        self.assertEqual(len(self.writes), 2)
        self.assertGreaterEqual(self.now, 5.5)

    def test_unsafe_targets_and_waits_never_write(self):
        for target, wait in (
            (19.9, 6), (50.1, 6), (33, 6), (30, 6),
            (30.5, 5.4), (30.5, float("nan")), (30.5, float("inf")),
        ):
            with self.subTest(target=target, wait=wait):
                with self.assertRaises(ValueError):
                    verify_heater_setpoint(self.instrument, target, wait, False)
        self.instrument.write_registers.assert_not_called()


class O2CalibrationTests(unittest.TestCase):
    def setUp(self):
        self.instrument = Mock()
        self.now = 0.0
        self.o2 = 20.95
        self.instrument.read_registers.side_effect = self.read_registers
        self.clock = patch(
            "scripts.verify_modbus.time.monotonic", side_effect=lambda: self.now
        )
        self.sleep = patch(
            "scripts.verify_modbus.time.sleep", side_effect=self.advance_clock
        )
        self.clock.start()
        self.sleep.start()
        self.addCleanup(self.clock.stop)
        self.addCleanup(self.sleep.stop)

    def advance_clock(self, seconds):
        self.now += seconds

    def read_registers(self, address, count, functioncode):
        if functioncode == 4 and address == 6:
            return float_to_registers(25.0)
        if functioncode == 4 and address == 0:
            return float_to_registers(self.o2)
        if functioncode == 3 and address == 0:
            return [0x0011, 0]
        self.fail(f"unexpected read: {address}, {count}, {functioncode}")

    def test_o2_span_write_requires_stable_readback(self):
        run_o2_span(self.instrument, 20.95, False)
        self.instrument.write_registers.assert_called_once_with(
            10, float_to_registers(20.95)
        )

    def test_o2_span_persistence_checks_after_power_cycle(self):
        with patch("builtins.input", side_effect=("yes", "yes")) as confirm_power_cycle:
            run_o2_span(self.instrument, 20.95, True)

        self.assertEqual(confirm_power_cycle.call_count, 2)
        self.instrument.write_registers.assert_called_once_with(
            10, float_to_registers(20.95)
        )
        self.assertEqual(self.now, 12.0)

    def test_o2_span_persistence_requires_usb_console_confirmation(self):
        with patch("builtins.input", return_value="no"):
            with self.assertRaisesRegex(RuntimeError, "did not confirm saved"):
                run_o2_span(self.instrument, 20.95, True)
        self.assertEqual(self.now, 6.0)

    def test_o2_span_persistence_fails_when_readback_changes_after_reboot(self):
        confirmations = 0

        def confirm_and_power_cycle(_prompt):
            nonlocal confirmations
            confirmations += 1
            if confirmations == 2:
                self.o2 = 19.0
            return "yes"

        with patch("builtins.input", side_effect=confirm_and_power_cycle):
            with self.assertRaisesRegex(RuntimeError, "after power cycle"):
                run_o2_span(self.instrument, 20.95, True)


if __name__ == "__main__":
    unittest.main()
