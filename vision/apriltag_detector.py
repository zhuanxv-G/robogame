"""USB camera based AprilTag detection."""

from typing import Any

import cv2
import numpy as np
from pupil_apriltags import Detector


class AprilTagDetector:
    """Detect tag36h11 AprilTags from a USB camera."""

    def __init__(
        self,
        camera_device: str | int = "/dev/video0",
        width: int = 640,
        height: int = 480,
    ) -> None:
        self._camera = cv2.VideoCapture(camera_device)
        if not self._camera.isOpened():
            self._camera.release()
            raise RuntimeError(f"Cannot open camera: {camera_device}")

        self._camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self._camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

        try:
            self._detector = Detector(families="tag36h11")
        except Exception:
            self._camera.release()
            raise

    def detect(self) -> list[dict[str, Any]]:
        """Capture one frame and return its AprilTag detections."""
        success, frame = self._camera.read()
        if not success or frame is None:
            raise RuntimeError("Failed to read a frame from the camera")

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        frame_height, frame_width = gray.shape[:2]
        image_center_x = frame_width / 2.0
        image_center_y = frame_height / 2.0

        results: list[dict[str, Any]] = []
        for detection in self._detector.detect(gray):
            cx = float(detection.center[0])
            cy = float(detection.center[1])
            corners = np.asarray(detection.corners, dtype=np.float32)

            results.append(
                {
                    "id": int(detection.tag_id),
                    "cx": cx,
                    "cy": cy,
                    "error_x": cx - image_center_x,
                    "error_y": cy - image_center_y,
                    "area": float(abs(cv2.contourArea(corners))),
                    "corners": corners,
                }
            )

        return results

    def release(self) -> None:
        """Release the USB camera."""
        self._camera.release()
