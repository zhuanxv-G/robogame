"""Manual AprilTag camera test. Exit with Ctrl+C."""

import sys
import time
from pathlib import Path


# Allow direct execution with: python3 tests/test_apriltag.py
PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from navigation.tag_map import TAG_MAP
from vision.apriltag_detector import AprilTagDetector


def main() -> None:
    detector = AprilTagDetector()

    try:
        while True:
            tags = detector.detect()
            if not tags:
                print("NO TAG")
            else:
                for tag in tags:
                    tag_info = TAG_MAP.get(tag["id"])
                    name = tag_info["name"] if tag_info else "unknown"

                    print(f"TAG {tag['id']}")
                    print(f"name: {name}")
                    print(f"error_x: {tag['error_x']:.1f}")
                    print(f"area: {tag['area']:.1f}")

            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nAprilTag test stopped")
    finally:
        detector.release()


if __name__ == "__main__":
    main()
