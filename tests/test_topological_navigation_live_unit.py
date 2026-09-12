"""Hardware-free checks for the manual navigation/topology live entry."""

from __future__ import annotations

import argparse
import sys
import unittest
from collections import deque
from pathlib import Path
from typing import Any


TESTS_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = TESTS_DIR.parent
for search_path in (str(PROJECT_ROOT), str(TESTS_DIR)):
    if search_path not in sys.path:
        sys.path.insert(0, search_path)

from hardware.stm32_link import Stm32Link  # noqa: E402
from navigation.topological_navigation import (  # noqa: E402
    TopologicalLocation,
    TopologicalNavigator,
)
from test_navigation_topology_live import run_live_test  # noqa: E402
from vision.apriltag_service import AprilTagService  # noqa: E402


class FakeSerial:
    def __init__(self) -> None:
        self.is_open = True
        self.writes: list[bytes] = []
        self.reset_called = False
        self.closed = False

    def write(self, data: bytes) -> int:
        self.writes.append(data)
        return len(data)

    def flush(self) -> None:
        pass

    def readline(self) -> bytes:
        if self.writes and self.writes[-1] == b"STOP\n":
            return b"ACK,STOP\n"
        return b""

    def reset_input_buffer(self) -> None:
        self.reset_called = True

    def close(self) -> None:
        self.is_open = False
        self.closed = True


class FakeDetector:
    def __init__(self, frames: list[object]) -> None:
        self._frames = deque(frames)
        self.released = False

    def detect(self) -> list[dict[str, Any]]:
        if not self._frames:
            return []
        frame = self._frames.popleft()
        if isinstance(frame, Exception):
            raise frame
        return frame  # type: ignore[return-value]

    def release(self) -> None:
        self.released = True


class RecordingTopologicalNavigator(TopologicalNavigator):
    def __init__(self) -> None:
        super().__init__()
        self.update_calls: list[int] = []

    def update_from_tag(self, tag_id: int) -> TopologicalLocation | None:
        self.update_calls.append(tag_id)
        return super().update_from_tag(tag_id)


class FirstStopTimeoutLink:
    """Model arrival STOP timeout followed by a successful cleanup STOP."""

    def __init__(self) -> None:
        self.is_connected = False
        self.commands: list[str] = []
        self.stop_labels: list[str] = []

    def connect(self) -> None:
        self.is_connected = True

    def send_command(self, command: str) -> str:
        self.commands.append(command)
        return command

    def stop(
        self,
        *,
        require_ack: bool = False,
        timeout: float = 2.0,
        debug_label: str = "stop",
    ) -> str | None:
        del require_ack, timeout
        self.stop_labels.append(debug_label)
        if len(self.stop_labels) == 1:
            raise TimeoutError("Timeout waiting for: ACK,STOP")
        return "ACK,STOP"

    def close(self) -> None:
        self.is_connected = False


def make_args(
    *,
    tag_id: int = 2,
    arrival_area: float = 4000.0,
    confirm_frames: int = 2,
) -> argparse.Namespace:
    return argparse.Namespace(
        tag_id=tag_id,
        arrival_area=arrival_area,
        confirm_frames=confirm_frames,
        stop_debug=False,
    )


class NavigationTopologyLiveTests(unittest.TestCase):
    def test_cancel_does_not_construct_or_connect_hardware(self) -> None:
        output: list[str] = []

        def forbidden_factory(*_args: object, **_kwargs: object) -> Any:
            self.fail("hardware owner was constructed before START")

        result = run_live_test(
            make_args(),
            input_func=lambda _prompt: "cancel",
            output=output.append,
            stm32_factory=forbidden_factory,
            apriltag_factory=forbidden_factory,
        )

        self.assertEqual(result, 0)
        self.assertIn("target_tag=2", output)
        self.assertIn("arrival_area=4000.0", output)
        self.assertIn("confirm_frames=2", output)
        self.assertIn(
            "Cancelled; no hardware was opened and no command was sent.",
            output,
        )

    def test_wrong_tag_never_updates_topology_before_target_arrival(self) -> None:
        serial_connection = FakeSerial()
        detector = FakeDetector(
            [
                [{"id": 3, "error_x": 0, "area": 9000}],
                [{"id": 2, "error_x": 10, "area": 4500}],
                [{"id": 2, "error_x": 8, "area": 4600}],
            ]
        )
        navigator = RecordingTopologicalNavigator()
        output: list[str] = []

        result = run_live_test(
            make_args(),
            input_func=lambda _prompt: "START",
            output=output.append,
            stm32_factory=lambda **_kwargs: Stm32Link(
                port="fake",
                serial_factory=lambda *_args, **_kwargs: serial_connection,
            ),
            apriltag_factory=lambda: AprilTagService(
                detector_factory=lambda **_kwargs: detector,
            ),
            navigator_factory=lambda: navigator,
            clock=lambda: 0.0,
        )

        self.assertEqual(result, 0)
        self.assertEqual(navigator.update_calls, [2])
        self.assertIs(
            navigator.current_location,
            TopologicalLocation.LEFT_ORANGE_MATERIAL,
        )
        self.assertIn("detected_tag=3", "\n".join(output))
        self.assertIn("confirmed_tag=2", output)
        self.assertIn("current_location=LEFT_ORANGE_MATERIAL", output)
        self.assertEqual(
            serial_connection.writes,
            [b"LINE_START\n", b"STOP\n"],
        )
        self.assertTrue(detector.released)
        self.assertTrue(serial_connection.closed)

    def test_detection_fault_attempts_stop_and_closes_both_devices(self) -> None:
        serial_connection = FakeSerial()
        detector = FakeDetector([RuntimeError("camera frame failed")])
        output: list[str] = []

        result = run_live_test(
            make_args(),
            input_func=lambda _prompt: "START",
            output=output.append,
            stm32_factory=lambda **_kwargs: Stm32Link(
                port="fake",
                serial_factory=lambda *_args, **_kwargs: serial_connection,
            ),
            apriltag_factory=lambda: AprilTagService(
                detector_factory=lambda **_kwargs: detector,
            ),
            clock=lambda: 0.0,
        )

        self.assertEqual(result, 1)
        self.assertGreaterEqual(serial_connection.writes.count(b"STOP\n"), 1)
        self.assertTrue(any(line.startswith("FAULT:") for line in output))
        self.assertTrue(detector.released)
        self.assertTrue(serial_connection.closed)

    def test_arrival_stop_timeout_sends_one_cleanup_stop(self) -> None:
        stm32 = FirstStopTimeoutLink()
        detector = FakeDetector(
            [[{"id": 2, "error_x": 0, "area": 5000}]]
        )
        navigator = RecordingTopologicalNavigator()
        output: list[str] = []
        args = make_args(confirm_frames=1)
        args.stop_debug = True

        result = run_live_test(
            args,
            input_func=lambda _prompt: "START",
            output=output.append,
            stm32_factory=lambda **_kwargs: stm32,  # type: ignore[arg-type]
            apriltag_factory=lambda: AprilTagService(
                detector_factory=lambda **_kwargs: detector,
            ),
            navigator_factory=lambda: navigator,
            clock=lambda: 0.0,
        )

        self.assertEqual(result, 1)
        self.assertEqual(stm32.commands, ["LINE_START"])
        self.assertEqual(stm32.stop_labels, ["stop", "cleanup"])
        self.assertEqual(navigator.update_calls, [])
        self.assertTrue(any(line.startswith("FAULT:") for line in output))
        self.assertIn("[STOPDBG] cleanup STOP requested", output)
        self.assertIn("STOP acknowledged by STM32.", output)
        self.assertTrue(detector.released)
        self.assertFalse(stm32.is_connected)


if __name__ == "__main__":
    unittest.main()
