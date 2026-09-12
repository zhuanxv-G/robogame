"""Hardware-free tests for the read-only line sensor capture tool."""

from __future__ import annotations

import argparse
import csv
import io
import sys
import tempfile
import unittest
from collections import deque
from pathlib import Path


TESTS_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = TESTS_DIR.parent
for search_path in (str(PROJECT_ROOT), str(TESTS_DIR)):
    if search_path not in sys.path:
        sys.path.insert(0, search_path)

from test_line_sensor_capture import (  # noqa: E402
    CSV_FIELDS,
    LineSensorStatus,
    create_csv_writer,
    normalize_operator_input,
    parse_line_status,
    request_line_status,
    run_capture,
    write_csv_sample,
)


class FakeStatusLink:
    def __init__(self, responses: list[bytes]) -> None:
        self.responses = deque(responses)
        self.commands: list[str] = []
        self.connected = False
        self.closed = False

    def connect(self) -> None:
        self.connected = True

    def send_command(self, command: str) -> str:
        self.commands.append(command)
        return command

    def readline(self) -> bytes:
        return self.responses.popleft() if self.responses else b""

    def close(self) -> None:
        self.closed = True
        self.connected = False


class LineSensorCaptureTests(unittest.TestCase):
    def test_parse_valid_line_status(self) -> None:
        status = parse_line_status("LINE_STATUS,1,0,1,1,0,0\r\n")

        self.assertEqual(
            status,
            LineSensorStatus(
                sensor_ok=1,
                s1=0,
                s2=1,
                s3=1,
                s4=0,
                active=0,
            ),
        )
        self.assertEqual(status.sensor_bits, "0110")

    def test_sensor_io_failure_is_preserved(self) -> None:
        status = parse_line_status("LINE_STATUS,0,1,0,0,1,0")

        self.assertEqual(status.sensor_ok, 0)
        self.assertEqual(status.sensor_bits, "1001")
        self.assertEqual(status.active, 0)

    def test_request_sends_only_line_status_and_skips_unrelated_reply(
        self,
    ) -> None:
        link = FakeStatusLink(
            [b"ACK,LINE_START\r\n", b"LINE_STATUS,1,1,0,0,1,0\r\n"]
        )

        status = request_line_status(link)

        self.assertEqual(link.commands, ["LINE_STATUS"])
        self.assertEqual(status.sensor_bits, "1001")

    def test_csv_output_has_required_fields_and_mark(self) -> None:
        stream = io.StringIO(newline="")
        writer = create_csv_writer(stream)
        write_csv_sample(
            writer,
            timestamp=1.23456,
            status=LineSensorStatus(1, 0, 1, 1, 0, 0),
            mark="T_CENTER",
        )

        stream.seek(0)
        reader = csv.DictReader(stream)
        self.assertEqual(tuple(reader.fieldnames or ()), CSV_FIELDS)
        self.assertEqual(
            list(reader),
            [
                {
                    "timestamp": "1.235",
                    "sensor_ok": "1",
                    "s1": "0",
                    "s2": "1",
                    "s3": "1",
                    "s4": "0",
                    "active": "0",
                    "mark": "T_CENTER",
                }
            ],
        )

    def test_mark_accepts_plain_or_prefixed_form(self) -> None:
        self.assertEqual(
            normalize_operator_input("t_center"),
            (False, "T_CENTER"),
        )
        self.assertEqual(
            normalize_operator_input("MARK,LEFT_OFFSET"),
            (False, "LEFT_OFFSET"),
        )
        self.assertEqual(normalize_operator_input("q"), (True, None))

    def test_capture_loop_connects_and_closes_with_only_status_query(
        self,
    ) -> None:
        link = FakeStatusLink([b"LINE_STATUS,1,0,1,1,0,0\r\n"])
        output: list[str] = []

        with tempfile.TemporaryDirectory() as temporary_directory:
            csv_path = Path(temporary_directory) / "capture.csv"
            result = run_capture(
                argparse.Namespace(rate_hz=10.0, output=csv_path),
                stm32_factory=lambda: link,  # type: ignore[arg-type]
                input_waiter=lambda _timeout: "Q",
                output=output.append,
            )

            with csv_path.open(encoding="utf-8", newline="") as stream:
                rows = list(csv.DictReader(stream))

        self.assertEqual(result, 0)
        self.assertEqual(link.commands, ["LINE_STATUS"])
        self.assertTrue(link.closed)
        self.assertFalse(link.connected)
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]["sensor_ok"], "1")
        self.assertEqual(rows[0]["mark"], "")
        self.assertTrue(any("sensors=0110" in line for line in output))


if __name__ == "__main__":
    unittest.main()
