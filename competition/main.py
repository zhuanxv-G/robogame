"""Future RoboGame competition entry point.

This skeleton only constructs software components and prints their state.  It
does not connect to serial devices or cameras and does not send robot commands.
"""

from pathlib import Path
import sys


if __package__ in (None, ""):
    project_root = Path(__file__).resolve().parents[1]
    sys.path.insert(0, str(project_root))

from competition.mission_controller import MissionController
from hardware.stm32_link import Stm32Link
from vision.apriltag_service import AprilTagService
from vision.k230_link import K230Link


def main() -> None:
    """Initialize the competition software skeleton without hardware I/O."""
    mission = MissionController()
    stm32 = Stm32Link()
    k230 = K230Link()
    apriltag = AprilTagService()

    print("RoboGame competition skeleton initialized")
    print(f"Mission state: {mission.current_state.value}")
    print(f"STM32 connected: {stm32.is_connected}")
    print(f"K230 connected: {k230.is_connected}")
    print(f"AprilTag camera connected: {apriltag.is_connected}")


if __name__ == "__main__":
    main()
