"""将 AprilTag 检测结果转换为上层状态机可用的定位信息。"""

from typing import Any, Mapping, TypedDict

from .tag_map import TAG_MAP


# 面积等于该值时置信度为 0.5；后续可根据实车数据调整。
_CONFIDENCE_AREA_SCALE = 2500.0


class LocalizationResult(TypedDict):
    """AprilTag 定位结果的固定字段。"""

    visible: bool
    tag_id: int | None
    tag_name: str
    error_x: float
    area: float
    confidence: float


class AprilTagLocalization:
    """从一帧 AprilTag 检测结果中生成轻量定位信息。"""

    def __init__(
        self,
        tag_map: Mapping[int, Mapping[str, Any]] | None = None,
    ) -> None:
        self._tag_map = TAG_MAP if tag_map is None else tag_map

    def localize(self, detections: list[dict[str, Any]]) -> LocalizationResult:
        """选择面积最大的 Tag，并返回状态机可直接读取的结果。"""
        if not detections:
            return {
                "visible": False,
                "tag_id": None,
                "tag_name": "unknown",
                "error_x": 0.0,
                "area": 0.0,
                "confidence": 0.0,
            }

        # 面积较大的 Tag 通常距离更近、角点更稳定，优先作为当前定位目标。
        primary_tag = max(detections, key=lambda tag: float(tag["area"]))
        tag_id = int(primary_tag["id"])
        error_x = float(primary_tag["error_x"])
        area = max(0.0, float(primary_tag["area"]))
        tag_info = self._tag_map.get(tag_id)
        tag_name = str(tag_info["name"]) if tag_info else "unknown"

        # 这是用于比赛状态判断的面积启发值，不代表检测器的统计概率。
        confidence = area / (area + _CONFIDENCE_AREA_SCALE) if area else 0.0

        return {
            "visible": True,
            "tag_id": tag_id,
            "tag_name": tag_name,
            "error_x": error_x,
            "area": area,
            "confidence": confidence,
        }
