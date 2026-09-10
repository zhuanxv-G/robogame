"""RoboGame 固定比赛场地的轻量世界模型。"""

from typing import TypedDict

from .tag_map import TAG_MAP


LINE_WIDTH_CM = 5.0
APRILTAG_SIZE_CM = (15.0, 15.0)
TAG_HEIGHT_CM = 40.0


class ZoneInfo(TypedDict):
    """Tag 对应的比赛区域和任务。"""

    name: str
    task: str


# 区域名称复用 TAG_MAP，避免两个地图中的 Tag 名称不一致。
TAG_ZONES: dict[int, ZoneInfo] = {
    1: {"name": str(TAG_MAP[1]["name"]), "task": "initialize"},
    2: {"name": str(TAG_MAP[2]["name"]), "task": "collect_orange"},
    3: {"name": str(TAG_MAP[3]["name"]), "task": "collect_purple"},
    4: {"name": str(TAG_MAP[4]["name"]), "task": "build"},
    5: {"name": str(TAG_MAP[5]["name"]), "task": "localize"},
    6: {"name": str(TAG_MAP[6]["name"]), "task": "localize"},
}


# 单位统一使用厘米，后续可继续加入场地尺寸和区域坐标。
FIELD_MAP = {
    "line_width_cm": LINE_WIDTH_CM,
    "apriltag_size_cm": APRILTAG_SIZE_CM,
    "tag_height_cm": TAG_HEIGHT_CM,
    "zones": TAG_ZONES,
}


def get_zone_by_tag(tag_id: int) -> ZoneInfo | None:
    """根据 Tag 编号查询区域；未知编号返回 None。"""
    zone = TAG_ZONES.get(tag_id)
    return zone.copy() if zone is not None else None
