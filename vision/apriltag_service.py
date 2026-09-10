"""Lifecycle wrapper around the existing AprilTag detector interface."""

from __future__ import annotations

from collections.abc import Callable
from typing import Any, Protocol


class DetectorInterface(Protocol):
    """Public interface already provided by ``AprilTagDetector``."""

    def detect(self) -> list[dict[str, Any]]: ...

    def release(self) -> None: ...


DetectorFactory = Callable[..., DetectorInterface]


class AprilTagServiceNotConnectedError(RuntimeError):
    """Raised when detection is requested before camera initialization."""


class AprilTagService:
    """Expose AprilTag detection without changing the verified algorithm.

    The existing :class:`vision.apriltag_detector.AprilTagDetector` is created
    only by :meth:`connect`, so constructing this service does not open a USB
    camera.
    """

    def __init__(
        self,
        camera_device: str | int = "/dev/video0",
        width: int = 640,
        height: int = 480,
        *,
        detector_factory: DetectorFactory | None = None,
    ) -> None:
        self.camera_device = camera_device
        self.width = width
        self.height = height
        self._detector_factory = detector_factory
        self._detector: DetectorInterface | None = None

    @property
    def is_connected(self) -> bool:
        return self._detector is not None

    def connect(self) -> None:
        """Explicitly initialize the existing camera and AprilTag detector."""
        if self._detector is not None:
            return

        factory = self._detector_factory or self._load_detector_factory()
        self._detector = factory(
            camera_device=self.camera_device,
            width=self.width,
            height=self.height,
        )

    @staticmethod
    def _load_detector_factory() -> DetectorFactory:
        from .apriltag_detector import AprilTagDetector

        return AprilTagDetector

    def detect(self) -> list[dict[str, Any]]:
        """Return detections produced by the existing detector unchanged."""
        if self._detector is None:
            raise AprilTagServiceNotConnectedError(
                "AprilTag service is not connected; call connect() first"
            )
        return self._detector.detect()

    def close(self) -> None:
        """Release the camera if it was initialized."""
        if self._detector is None:
            return
        detector = self._detector
        self._detector = None
        detector.release()
