import unittest
from unittest.mock import patch

from tools.pc_serial_bridge import (
    SerialBridge,
    format_pid_command,
    parse_vision_line,
)


class FakeSerial:
    def __init__(self, lines=None):
        self.lines = list(lines or [])
        self.writes = []
        self.bridge = None

    def readline(self):
        if self.lines:
            return self.lines.pop(0)
        if self.bridge is not None:
            self.bridge.stop_event.set()
        return b""

    def write(self, payload):
        self.writes.append(payload)


class ParseVisionLineTests(unittest.TestCase):
    def test_accepts_exact_four_field_frame(self):
        self.assertEqual(parse_vision_line(b"B,523,12,2\n"), "B,523,12,2\n")

    def test_normalizes_crlf_and_negative_position(self):
        self.assertEqual(parse_vision_line("B,7,-125,3\r\n"), "B,7,-125,3\n")

    def test_rejects_human_log(self):
        self.assertIsNone(parse_vision_line("FPS: 35.0 DIST:+1.2cm"))

    def test_rejects_extra_field(self):
        self.assertIsNone(parse_vision_line("B,1,2,2,extra\n"))

    def test_rejects_invalid_state(self):
        self.assertIsNone(parse_vision_line("B,1,2,4\n"))

    def test_rejects_position_outside_guard(self):
        self.assertIsNone(parse_vision_line("B,1,151,2\n"))

    def test_rejects_frame_id_outside_uno_range(self):
        self.assertIsNone(parse_vision_line("B,2147483648,0,2\n"))

    def test_rejects_non_ascii(self):
        self.assertIsNone(parse_vision_line(b"B,1,2,2\xff\n"))


class PidCommandTests(unittest.TestCase):
    def test_formats_three_valid_pid_gains(self):
        self.assertEqual(format_pid_command([2.0, 0.05, 1.0]), "PID 2 0.05 1")

    def test_rejects_nonpositive_kp(self):
        with self.assertRaises(ValueError):
            format_pid_command([0.0, 0.0, 0.0])

    def test_rejects_negative_or_nonfinite_gain(self):
        with self.assertRaises(ValueError):
            format_pid_command([2.0, -0.1, 0.0])
        with self.assertRaises(ValueError):
            format_pid_command([2.0, 0.0, float("nan")])


class SerialBridgeTests(unittest.TestCase):
    def test_forwards_only_canonical_vision_frames(self):
        k230 = FakeSerial([b"FPS: 35\n", b"B,9,-12,2\r\n"])
        uno = FakeSerial()
        bridge = SerialBridge(
            k230,
            uno,
            max_position_mm=150,
            forward=True,
            show_k230_log=False,
        )
        k230.bridge = bridge

        bridge.read_k230()

        self.assertEqual(uno.writes, [b"B,9,-12,2\n"])
        self.assertEqual(bridge.stats.forwarded_frames, 1)
        self.assertEqual(bridge.stats.rejected_lines, 1)

    def test_user_command_is_single_newline_terminated_write(self):
        uno = FakeSerial()
        bridge = SerialBridge(
            FakeSerial(),
            uno,
            max_position_mm=150,
            forward=True,
            show_k230_log=False,
        )
        bridge.send_uno("STATUS\r\n")
        self.assertEqual(uno.writes, [b"STATUS\n"])

    def test_quiet_status_shows_initial_and_hides_periodic_updates(self):
        uno = FakeSerial(
            [
                b"STATUS mode=LOCKED frame=1\n",
                b"STATUS mode=LOCKED frame=2\n",
                b"OK ZERO confirmed\n",
            ]
        )
        bridge = SerialBridge(
            FakeSerial(),
            uno,
            max_position_mm=150,
            forward=True,
            show_k230_log=False,
            quiet_uno_status=True,
        )
        uno.bridge = bridge

        with patch("builtins.print") as mocked_print:
            bridge.read_uno()

        printed = [call.args[0] for call in mocked_print.call_args_list]
        self.assertEqual(
            printed,
            [
                "[UNO] STATUS mode=LOCKED frame=1",
                "[UNO] OK ZERO confirmed",
            ],
        )

    def test_quiet_status_samples_active_motion_twice_per_second(self):
        uno = FakeSerial(
            [
                b"STATUS mode=ARMED pulse=-10\n",
                b"STATUS mode=ARMED pulse=-20\n",
                b"STATUS mode=RETURNING pulse=-15\n",
            ]
        )
        bridge = SerialBridge(
            FakeSerial(),
            uno,
            max_position_mm=150,
            forward=True,
            show_k230_log=False,
            quiet_uno_status=True,
        )
        bridge.show_next_uno_status.clear()
        uno.bridge = bridge

        with patch(
            "tools.pc_serial_bridge.time.monotonic",
            side_effect=[10.0, 10.1, 10.6],
        ), patch("builtins.print") as mocked_print:
            bridge.read_uno()

        printed = [call.args[0] for call in mocked_print.call_args_list]
        self.assertEqual(
            printed,
            [
                "[UNO] STATUS mode=ARMED pulse=-10",
                "[UNO] STATUS mode=RETURNING pulse=-15",
            ],
        )


if __name__ == "__main__":
    unittest.main()
