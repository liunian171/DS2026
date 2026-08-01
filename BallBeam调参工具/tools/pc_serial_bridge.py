#!/usr/bin/env python3
"""Filter K230D USB vision frames and forward them to the UNO USB serial port.

The K230D console may also contain human-readable debug messages. Only exact
``B,<frame_id>,<position_mm>,<state>`` lines pass to the UNO. Commands typed in
this program are sent to the UNO; ``quit`` exits and ``stats`` is local only.
"""

from __future__ import annotations

import argparse
import math
import re
import sys
import threading
import time
from dataclasses import dataclass
from typing import Optional


VISION_PATTERN = re.compile(r"B,([0-9]+),(-?[0-9]+),([0-3])")
MAX_FRAME_ID = 2_147_483_647
DEFAULT_MAX_POSITION_MM = 150


def format_pid_command(gains: list[float] | tuple[float, float, float]) -> str:
    """Validate three gains and return the single UNO PID command."""

    if len(gains) != 3:
        raise ValueError("exactly three PID gains are required")
    kp, ki, kd = (float(value) for value in gains)
    if not all(math.isfinite(value) for value in (kp, ki, kd)):
        raise ValueError("PID gains must be finite")
    if kp <= 0.0 or ki < 0.0 or kd < 0.0:
        raise ValueError("KP must be > 0 and KI/KD must be >= 0")
    if max(kp, ki, kd) > 10000.0:
        raise ValueError("PID gains must not exceed 10000")
    return f"PID {kp:g} {ki:g} {kd:g}"


def parse_vision_line(
    raw_line: bytes | str,
    max_position_mm: int = DEFAULT_MAX_POSITION_MM,
) -> Optional[str]:
    """Return a canonical vision line, or ``None`` for non-control output."""

    if isinstance(raw_line, bytes):
        try:
            text = raw_line.decode("ascii", errors="strict")
        except UnicodeDecodeError:
            return None
    else:
        text = raw_line

    text = text.strip("\r\n")
    match = VISION_PATTERN.fullmatch(text)
    if match is None:
        return None

    frame_id = int(match.group(1))
    position_mm = int(match.group(2))
    state = int(match.group(3))
    if frame_id > MAX_FRAME_ID or abs(position_mm) > max_position_mm:
        return None
    return f"B,{frame_id},{position_mm},{state}\n"


@dataclass
class BridgeStats:
    k230_lines: int = 0
    forwarded_frames: int = 0
    rejected_lines: int = 0
    uno_lines: int = 0
    user_commands: int = 0


class SerialBridge:
    def __init__(
        self,
        k230_serial,
        uno_serial,
        *,
        max_position_mm: int,
        forward: bool,
        show_k230_log: bool,
        quiet_uno_status: bool = False,
        no_periodic_status: bool = False,
    ) -> None:
        self.k230_serial = k230_serial
        self.uno_serial = uno_serial
        self.max_position_mm = max_position_mm
        self.forward = forward
        self.show_k230_log = show_k230_log
        self.quiet_uno_status = quiet_uno_status
        self.no_periodic_status = no_periodic_status
        self.stop_event = threading.Event()
        self.show_next_uno_status = threading.Event()
        self.requested_status = threading.Event()
        if self.quiet_uno_status:
            # Show one initial status after the UNO resets, then stay quiet.
            self.show_next_uno_status.set()
        self.last_active_status_print_s = 0.0
        self.uno_write_lock = threading.Lock()
        self.stats = BridgeStats()

    def send_uno(self, line: str) -> None:
        payload = line.rstrip("\r\n") + "\n"
        if self.quiet_uno_status and payload.strip().upper() == "STATUS":
            self.show_next_uno_status.set()
        if payload.strip().upper() == "STATUS":
            self.requested_status.set()
        with self.uno_write_lock:
            self.uno_serial.write(payload.encode("ascii"))

    def read_k230(self) -> None:
        while not self.stop_event.is_set():
            try:
                raw_line = self.k230_serial.readline()
            except Exception as exc:  # pyserial exception type is runtime-loaded
                print(f"\n[BRIDGE] K230 read failed: {exc}", file=sys.stderr)
                self.stop_event.set()
                return
            if not raw_line:
                continue

            self.stats.k230_lines += 1
            canonical = parse_vision_line(raw_line, self.max_position_mm)
            if canonical is None:
                self.stats.rejected_lines += 1
                if self.show_k230_log:
                    text = raw_line.decode("utf-8", errors="replace").rstrip()
                    print(f"[K230] {text}")
                continue

            if self.forward:
                try:
                    self.send_uno(canonical)
                except Exception as exc:
                    print(f"\n[BRIDGE] UNO write failed: {exc}", file=sys.stderr)
                    self.stop_event.set()
                    return
            self.stats.forwarded_frames += 1

    def read_uno(self) -> None:
        while not self.stop_event.is_set():
            try:
                raw_line = self.uno_serial.readline()
            except Exception as exc:
                print(f"\n[BRIDGE] UNO read failed: {exc}", file=sys.stderr)
                self.stop_event.set()
                return
            if not raw_line:
                continue
            self.stats.uno_lines += 1
            text = raw_line.decode("utf-8", errors="replace").rstrip()
            if self.no_periodic_status and text.startswith("STATUS "):
                # Only show STATUS lines that were explicitly requested.
                if self.requested_status.is_set():
                    self.requested_status.clear()
                    print(f"[UNO] {text}")
                continue
            if self.quiet_uno_status and text.startswith("STATUS "):
                active_motion = (
                    "mode=ARMED" in text or "mode=RETURNING" in text
                )
                now_s = time.monotonic()
                if self.show_next_uno_status.is_set():
                    self.show_next_uno_status.clear()
                    if active_motion:
                        self.last_active_status_print_s = now_s
                elif active_motion:
                    if now_s - self.last_active_status_print_s < 0.5:
                        continue
                    self.last_active_status_print_s = now_s
                else:
                    continue
            print(f"[UNO] {text}")

    def print_stats(self) -> None:
        print(
            "[STATS] "
            f"k230_lines={self.stats.k230_lines} "
            f"forwarded={self.stats.forwarded_frames} "
            f"rejected={self.stats.rejected_lines} "
            f"uno_lines={self.stats.uno_lines} "
            f"commands={self.stats.user_commands}"
        )

    def poll_command_file(self, path: str) -> Optional[str]:
        """Read and consume one pending command from the command file."""
        try:
            with open(path, "r", encoding="ascii", newline="") as fh:
                content = fh.read()
        except OSError:
            return None
        command = content.strip()
        if not command:
            return None
        try:
            with open(path, "w", encoding="ascii", newline="") as fh:
                fh.write("")
        except OSError:
            return None
        return command

    def run_command_file_loop(self, path: str, interval: float) -> None:
        """Poll the command file until stopped; forward commands to the UNO."""
        while not self.stop_event.is_set():
            command = self.poll_command_file(path)
            if command is not None:
                lowered = command.lower()
                if lowered in {"quit", "exit"}:
                    self.stop_event.set()
                    print("[CMD-FILE] quit requested")
                    return
                if lowered == "stats":
                    self.print_stats()
                    continue
                self.send_uno(command)
                self.stats.user_commands += 1
                print(f"[CMD-FILE] {command}")
            time.sleep(interval)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--k230", default="COM7", help="K230D USB console port")
    parser.add_argument("--uno", default="COM9", help="Arduino UNO USB port")
    parser.add_argument("--k230-baud", type=int, default=115200)
    parser.add_argument("--uno-baud", type=int, default=115200)
    parser.add_argument(
        "--max-position-mm", type=int, default=DEFAULT_MAX_POSITION_MM
    )
    parser.add_argument(
        "--no-forward",
        action="store_true",
        help="validate K230D frames without forwarding them to the UNO",
    )
    parser.add_argument(
        "--show-k230-log",
        action="store_true",
        help="show non-control K230D console output",
    )
    parser.add_argument(
        "--quiet-uno-status",
        action="store_true",
        help="hide periodic UNO STATUS lines except the first and requested status",
    )
    parser.add_argument(
        "--cmd-file",
        default=None,
        help="optional command file polled for commands (write one command per write)",
    )
    parser.add_argument(
        "--cmd-file-interval",
        type=float,
        default=0.2,
        help="command file poll interval in seconds (default 0.2)",
    )
    parser.add_argument(
        "--reset-uno",
        action="store_true",
        help="reset the UNO with a DTR falling edge after opening its port",
    )
    parser.add_argument(
        "--no-periodic-status",
        action="store_true",
        help="never print periodic UNO STATUS lines; only show requested ones",
    )
    parser.add_argument(
        "--pid",
        nargs=3,
        type=float,
        metavar=("KP", "KI", "KD"),
        help="send PID gains to the UNO and enable automatic closed loop",
    )
    return parser


def main(argv: Optional[list[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    if args.max_position_mm <= 0:
        print("--max-position-mm must be positive", file=sys.stderr)
        return 2
    try:
        pid_command = format_pid_command(args.pid) if args.pid is not None else None
    except ValueError as exc:
        print(f"Invalid --pid values: {exc}", file=sys.stderr)
        return 2

    try:
        import serial
    except ModuleNotFoundError:
        print(
            "pyserial is required. Run: python -m pip install -r requirements.txt",
            file=sys.stderr,
        )
        return 2

    try:
        k230_port = serial.Serial(
            args.k230, args.k230_baud, timeout=0.10, write_timeout=0.50
        )
    except serial.SerialException as exc:
        print(f"Cannot open K230D port {args.k230}: {exc}", file=sys.stderr)
        return 2

    try:
        uno_port = serial.Serial(
            args.uno, args.uno_baud, timeout=0.10, write_timeout=0.50
        )
    except serial.SerialException as exc:
        k230_port.close()
        print(f"Cannot open UNO port {args.uno}: {exc}", file=sys.stderr)
        return 2

    # Opening an UNO USB serial port normally resets the board.
    time.sleep(2.0)
    if args.reset_uno:
        print("[BRIDGE] Resetting UNO with DTR falling edge...")
        uno_port.dtr = True
        time.sleep(0.1)
        uno_port.dtr = False  # falling edge -> UNO reset
        time.sleep(0.3)
        uno_port.dtr = True   # release
        time.sleep(2.5)       # wait for the ATmega328 reboot
    k230_port.reset_input_buffer()
    uno_port.reset_input_buffer()

    bridge = SerialBridge(
        k230_port,
        uno_port,
        max_position_mm=args.max_position_mm,
        forward=not args.no_forward,
        show_k230_log=args.show_k230_log,
        quiet_uno_status=args.quiet_uno_status,
        no_periodic_status=args.no_periodic_status,
    )
    threads = [
        threading.Thread(target=bridge.read_k230, name="k230-reader", daemon=True),
        threading.Thread(target=bridge.read_uno, name="uno-reader", daemon=True),
    ]
    for thread in threads:
        thread.start()

    if pid_command is not None:
        bridge.send_uno(pid_command)
        bridge.stats.user_commands += 1
        print(f"[BRIDGE] Sent {pid_command}")

    if args.cmd_file is not None:
        cmd_thread = threading.Thread(
            target=bridge.run_command_file_loop,
            args=(args.cmd_file, max(0.05, args.cmd_file_interval)),
            name="cmd-file-loop",
            daemon=True,
        )
        cmd_thread.start()
        print(f"[BRIDGE] Command file: {args.cmd_file} (write quit to stop)")

    mode = "MONITOR ONLY" if args.no_forward else "FORWARDING"
    print(f"[BRIDGE] {mode}: {args.k230} -> {args.uno}")
    print("[BRIDGE] Type 'PID kp ki kd' to retune, STATUS, RETURN, DISARM, stats, or quit.")
    if args.no_periodic_status:
        print("[BRIDGE] Periodic STATUS hidden; type STATUS to see the current state.")

    try:
        while not bridge.stop_event.is_set():
            command = input("[CMD] > ").strip()
            if command.startswith("\ufeff"):
                command = command.lstrip("\ufeff")  # tolerate UTF-8 BOM on stdin
            if not command:
                continue
            if command.lower() in {"quit", "exit"}:
                break
            if command.lower() == "stats":
                bridge.print_stats()
                continue
            bridge.send_uno(command)
            bridge.stats.user_commands += 1
    except (KeyboardInterrupt, EOFError):
        print("\n[BRIDGE] stopping")
    finally:
        bridge.stop_event.set()
        # Best effort: invalidate zero and lock motor output before closing COM9.
        try:
            bridge.send_uno("DISARM")
            time.sleep(0.05)
        except Exception:
            pass
        k230_port.close()
        uno_port.close()
        bridge.print_stats()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
