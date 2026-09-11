"""Pure-software tests for AprilTag-assisted line arrival."""

from __future__ import annotations

import runpy
import sys
import unittest
from pathlib import Path
from unittest import mock


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from competition.navigation_controller import (  # noqa: E402
    NavigationController,
    NavigationState,
)
from hardware.stm32_link import Stm32Link  # noqa: E402
from vision.apriltag_service import AprilTagService  # noqa: E402


class FakeStm32Link:
    def __init__(self) -> None:
        self.commands: list[str] = []
        self.stop_calls = 0
        self.send_error: Exception | None = None
        self.stop_error: Exception | None = None

    def send_command(self, command: str) -> str:
        if self.send_error is not None:
            raise self.send_error
        self.commands.append(command)
        return command

    def stop(
        self,
        *,
        require_ack: bool = False,
        timeout: float = 2.0,
    ) -> str | None:
        del timeout
        self.stop_calls += 1
        if self.stop_error is not None:
            raise self.stop_error
        return "ACK,STOP" if require_ack else None


class FakeAprilTagService:
    def __init__(self, frames: list[object]) -> None:
        self._frames = iter(frames)
        self.detect_calls = 0

    def detect(self) -> list[dict[str, object]]:
        self.detect_calls += 1
        frame = next(self._frames, [])
        if isinstance(frame, Exception):
            raise frame
        return frame  # type: ignore[return-value]


def detection(tag_id: int, area: float, error_x: float = 0) -> dict[str, object]:
    return {"id": tag_id, "area": area, "error_x": error_x}


class LineToTagTests(unittest.TestCase):
    def make_controller(
        self,
        frames: list[object],
        *,
        confirm_frames: int = 3,
    ) -> tuple[NavigationController, FakeStm32Link, FakeAprilTagService]:
        stm32 = FakeStm32Link()
        apriltag = FakeAprilTagService(frames)
        controller = NavigationController(stm32, apriltag)
        controller.start_line_to_tag(
            2,
            arrival_area=10000,
            confirm_frames=confirm_frames,
        )
        return controller, stm32, apriltag

    def test_missing_target_keeps_following_without_stop(self) -> None:
        controller, stm32, _ = self.make_controller([[]])

        self.assertIs(controller.update(), NavigationState.LINE_FOLLOWING)
        status = controller.get_navigation_status()
        self.assertFalse(status["tag_visible"])
        self.assertEqual(status["confirm_count"], 0)
        self.assertEqual(stm32.stop_calls, 0)

    def test_wrong_tag_keeps_following_without_stop(self) -> None:
        controller, stm32, _ = self.make_controller(
            [[detection(3, 20000)]]
        )

        self.assertIs(controller.update(), NavigationState.LINE_FOLLOWING)
        self.assertFalse(controller.get_navigation_status()["tag_visible"])
        self.assertEqual(stm32.stop_calls, 0)

    def test_target_below_threshold_keeps_following(self) -> None:
        controller, stm32, _ = self.make_controller(
            [[detection(2, 9999, error_x=-12)]]
        )

        self.assertIs(controller.update(), NavigationState.LINE_FOLLOWING)
        status = controller.get_navigation_status()
        self.assertTrue(status["tag_visible"])
        self.assertEqual(status["error_x"], -12.0)
        self.assertEqual(status["area"], 9999.0)
        self.assertEqual(status["confirm_count"], 0)
        self.assertEqual(stm32.stop_calls, 0)

    def test_consecutive_qualifying_frames_stop_once_and_arrive(self) -> None:
        frames = [[detection(2, area)] for area in (10000, 11000, 12000)]
        controller, stm32, apriltag = self.make_controller(frames)

        self.assertIs(controller.update(), NavigationState.LINE_FOLLOWING)
        self.assertIs(controller.update(), NavigationState.LINE_FOLLOWING)
        self.assertIs(controller.update(), NavigationState.ARRIVED)
        self.assertIs(controller.update(), NavigationState.ARRIVED)

        self.assertEqual(stm32.stop_calls, 1)
        self.assertEqual(apriltag.detect_calls, 3)
        self.assertEqual(
            controller.get_navigation_status()["last_command"],
            "STOP",
        )

    def test_missing_frame_resets_confirmation_count(self) -> None:
        frames = [
            [detection(2, 12000)],
            [],
            [detection(2, 12000)],
            [detection(2, 12000)],
            [detection(2, 12000)],
        ]
        controller, stm32, _ = self.make_controller(frames)

        controller.update()
        self.assertEqual(
            controller.get_navigation_status()["confirm_count"],
            1,
        )
        controller.update()
        self.assertEqual(
            controller.get_navigation_status()["confirm_count"],
            0,
        )
        controller.update()
        controller.update()
        self.assertIs(controller.current_state, NavigationState.LINE_FOLLOWING)
        self.assertEqual(stm32.stop_calls, 0)
        self.assertIs(controller.update(), NavigationState.ARRIVED)
        self.assertEqual(stm32.stop_calls, 1)

    def test_stop_exception_enters_fault(self) -> None:
        controller, stm32, _ = self.make_controller(
            [[detection(2, 12000)]],
            confirm_frames=1,
        )
        stm32.stop_error = RuntimeError("STOP link failed")

        with self.assertRaisesRegex(RuntimeError, "STOP link failed"):
            controller.update()

        self.assertIs(controller.current_state, NavigationState.FAULT)
        self.assertEqual(stm32.stop_calls, 1)

    def test_camera_exception_attempts_stop_and_enters_fault(self) -> None:
        controller, stm32, _ = self.make_controller(
            [RuntimeError("camera failed")]
        )

        with self.assertRaisesRegex(RuntimeError, "camera failed"):
            controller.update()

        self.assertIs(controller.current_state, NavigationState.FAULT)
        self.assertEqual(stm32.stop_calls, 1)
        self.assertEqual(
            controller.get_navigation_status()["last_command"],
            "STOP",
        )

    def test_stm32_start_exception_attempts_stop_and_enters_fault(self) -> None:
        stm32 = FakeStm32Link()
        stm32.send_error = RuntimeError("serial write failed")
        controller = NavigationController(stm32, FakeAprilTagService([]))

        with self.assertRaisesRegex(RuntimeError, "serial write failed"):
            controller.start_line_to_tag(2, 10000, 3)

        self.assertIs(controller.current_state, NavigationState.FAULT)
        self.assertEqual(stm32.stop_calls, 1)

    def test_manual_entry_import_does_not_open_hardware(self) -> None:
        script = PROJECT_ROOT / "tests" / "test_line_to_tag.py"
        with (
            mock.patch.object(Stm32Link, "connect") as stm32_connect,
            mock.patch.object(AprilTagService, "connect") as camera_connect,
        ):
            runpy.run_path(str(script), run_name="line_to_tag_import_test")

        stm32_connect.assert_not_called()
        camera_connect.assert_not_called()


if __name__ == "__main__":
    unittest.main()
