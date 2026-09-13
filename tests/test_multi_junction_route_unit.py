"""Hardware-safety tests for the multi-junction live entry point."""

from __future__ import annotations

import argparse
import sys
import unittest
from collections import deque
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from hardware.stm32_link import Stm32Link  # noqa: E402
from tests.test_multi_junction_route import run_test  # noqa: E402
from vision.apriltag_service import AprilTagService  # noqa: E402


class MultiJunctionLiveSafetyTests(unittest.TestCase):
    def test_without_exact_start_no_hardware_factory_is_called(self) -> None:
        stm32_calls = 0
        camera_calls = 0

        def forbidden_stm32_factory() -> Stm32Link:
            nonlocal stm32_calls
            stm32_calls += 1
            raise AssertionError("STM32 factory must not be called")

        def forbidden_apriltag_factory() -> AprilTagService:
            nonlocal camera_calls
            camera_calls += 1
            raise AssertionError("AprilTag factory must not be called")

        output: list[str] = []
        result = run_test(
            argparse.Namespace(
                start="right_material_t_junction",
                target="right_orange_material",
                target_tag=4,
                arrival_area=10000.0,
                confirm_frames=3,
                heading="EAST",
                junction_timeout=30.0,
                turn_timeout=5.0,
                tag_timeout=30.0,
            ),
            input_fn=lambda prompt: "start",
            stm32_factory=forbidden_stm32_factory,
            apriltag_factory=forbidden_apriltag_factory,
            output=output.append,
        )

        self.assertEqual(result, 2)
        self.assertEqual(stm32_calls, 0)
        self.assertEqual(camera_calls, 0)
        self.assertIn("LEFT -> STRAIGHT", output)
        self.assertIn("Cancelled: exact START confirmation not received.", output)

    def test_fake_two_junction_route_reaches_tag_and_closes(self) -> None:
        class FakeStm32:
            def __init__(self) -> None:
                self.is_connected = False
                self.lines = deque(
                    [
                        b"JUNCTION_DETECTED\r\n",
                        b"TURN_STARTED,LEFT\r\n",
                        b"TURN_DONE,LEFT\r\n",
                        b"JUNCTION_DETECTED\r\n",
                        b"TURN_STARTED,STRAIGHT\r\n",
                        b"TURN_DONE,STRAIGHT\r\n",
                    ]
                )
                self.commands: list[object] = []

            def connect(self) -> None:
                self.is_connected = True
                self.commands.append("connect")

            def set_next_turn(self, turn: object, timeout: float = 2.0) -> str:
                value = getattr(turn, "value", turn)
                self.commands.append(("next_turn", value))
                return f"ACK,NEXT_TURN,{value}"

            def send_command(self, command: str) -> str:
                self.commands.append(("send", command))
                return command

            def wait_ack(self, command: str, timeout: float = 2.0) -> str:
                self.commands.append(("wait_ack", command))
                return f"ACK,{command}"

            def readline(self) -> bytes:
                return self.lines.popleft() if self.lines else b""

            def stop(self, **kwargs: object) -> str:
                self.commands.append(("stop", kwargs.get("require_ack")))
                return "ACK,STOP"

            def close(self) -> None:
                self.is_connected = False
                self.commands.append("close")

        class FakeAprilTag:
            def __init__(self) -> None:
                self.is_connected = False
                self.closed = False

            def connect(self) -> None:
                self.is_connected = True

            def detect(self) -> list[dict[str, object]]:
                return [{"id": 4, "area": 12000, "error_x": 0}]

            def close(self) -> None:
                self.is_connected = False
                self.closed = True

        stm32 = FakeStm32()
        camera = FakeAprilTag()
        output: list[str] = []
        result = run_test(
            argparse.Namespace(
                start="right_material_t_junction",
                target="right_orange_material",
                target_tag=4,
                arrival_area=10000.0,
                confirm_frames=1,
                heading="EAST",
                junction_timeout=30.0,
                turn_timeout=5.0,
                tag_timeout=30.0,
            ),
            input_fn=lambda prompt: "START",
            stm32_factory=lambda: stm32,  # type: ignore[arg-type]
            apriltag_factory=lambda: camera,  # type: ignore[arg-type]
            output=output.append,
        )

        self.assertEqual(result, 0)
        self.assertEqual(
            [command for command in stm32.commands if isinstance(command, tuple)],
            [
                ("next_turn", "LEFT"),
                ("send", "LINE_START"),
                ("wait_ack", "LINE_START"),
                ("next_turn", "STRAIGHT"),
                ("stop", True),
            ],
        )
        self.assertEqual(stm32.commands[-1], "close")
        self.assertTrue(camera.closed)
        self.assertIn("TAG 4 ARRIVED", output)
        self.assertIn("current_location=right_orange_material", output)
        self.assertIn("ROUTE FINISHED", output)


if __name__ == "__main__":
    unittest.main()
