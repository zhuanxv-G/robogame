"""RoboGame vision interfaces with lazy optional-dependency imports."""

from typing import Any


__all__ = ["AprilTagDetector", "AprilTagService", "K230Link"]


def __getattr__(name: str) -> Any:
    """Keep importing ``vision`` free of camera and serial dependencies."""
    if name == "AprilTagDetector":
        from .apriltag_detector import AprilTagDetector

        return AprilTagDetector
    if name == "AprilTagService":
        from .apriltag_service import AprilTagService

        return AprilTagService
    if name == "K230Link":
        from .k230_link import K230Link

        return K230Link
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
