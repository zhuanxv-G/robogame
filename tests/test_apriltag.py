"""AprilTag 摄像头人工测试程序，按 Ctrl+C 退出。"""

import argparse
import math
import sys
import time
from pathlib import Path


# 确保可以在项目根目录直接运行：python tests/test_apriltag.py
PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from navigation.tag_map import TAG_MAP


DEFAULT_PRINT_INTERVAL = 1.0


def positive_interval(value: str) -> float:
    """Parse a strictly positive terminal output interval."""
    try:
        interval = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("must be a number") from exc

    if not math.isfinite(interval) or interval <= 0:
        raise argparse.ArgumentTypeError("must be a finite number greater than 0")

    return interval


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Continuously detect AprilTags and periodically print the latest result."
    )
    parser.add_argument(
        "--print-interval",
        type=positive_interval,
        default=DEFAULT_PRINT_INTERVAL,
        metavar="SECONDS",
        help="terminal output interval in seconds (default: %(default)s)",
    )
    return parser.parse_args(argv)


def print_latest(tags: list[dict]) -> None:
    """Print the latest detection state without affecting detector cadence."""
    if not tags:
        print("NO TAG")
        return

    for tag in tags:
        tag_info = TAG_MAP.get(tag["id"])
        name = tag_info["name"] if tag_info else "unknown"

        print(f"TAG {tag['id']}")
        print(f"name: {name}")
        print(f"cx: {tag['cx']:.1f}")
        print(f"cy: {tag['cy']:.1f}")
        print(f"error_x: {tag['error_x']:.1f}")
        print(f"error_y: {tag['error_y']:.1f}")
        print(f"area: {tag['area']:.1f}")


def create_detector():
    """Create the real detector only after CLI arguments have been parsed."""
    from vision.apriltag_detector import AprilTagDetector

    return AprilTagDetector()


def main(argv: list[str] | None = None) -> None:
    args = parse_args(argv)
    detector = None

    try:
        detector = create_detector()
        last_print_time = time.monotonic() - args.print_interval

        while True:
            tags = detector.detect()
            now = time.monotonic()

            if now - last_print_time >= args.print_interval:
                print_latest(tags)
                last_print_time = now
    except KeyboardInterrupt:
        print("\nAprilTag test stopped")
    except RuntimeError as error:
        print(f"CAMERA ERROR: {error}")
    finally:
        if detector is not None:
            detector.release()


if __name__ == "__main__":
    main()
