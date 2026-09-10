"""AprilTag 轻量定位接口人工测试程序，按 Ctrl+C 退出。"""

import sys
import time
from pathlib import Path


# 确保可以在项目根目录直接运行：python tests/test_localization.py
PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from navigation.localization import AprilTagLocalization, LocalizationResult
from vision.apriltag_detector import AprilTagDetector


def print_location(location: LocalizationResult) -> None:
    """按统一格式输出一帧定位结果。"""
    if not location["visible"]:
        print("NO TAG")
        return

    print("=" * 16)
    print("TAG DETECTED")
    print(f"id: {location['tag_id']}")
    print(f"name: {location['tag_name']}")
    print(f"error_x: {location['error_x']:.1f}")
    print(f"area: {location['area']:.1f}")
    print(f"confidence: {location['confidence']:.2f}")
    print("=" * 16)


def main() -> None:
    detector = None
    localization = AprilTagLocalization()

    try:
        detector = AprilTagDetector()

        while True:
            print_location(localization.localize(detector.detect()))
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nLocalization test stopped")
    except RuntimeError as error:
        print(f"CAMERA ERROR: {error}")
    finally:
        if detector is not None:
            detector.release()


if __name__ == "__main__":
    main()
