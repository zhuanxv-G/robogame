"""RoboGame 固定比赛场地的轻量世界模型。"""

from typing import TypedDict

from .field_model import FIELD_INFO, TAG_MAP as WORLD_TAG_MAP


LINE_WIDTH_CM = float(FIELD_INFO["line_width_cm"])
APRILTAG_SIZE_CM = tuple(FIELD_INFO["tag_size_cm"])
TAG_HEIGHT_CM = float(FIELD_INFO["tag_top_edge_height_cm"])


class ZoneInfo(TypedDict):
    """Tag 对应的比赛区域和任务。"""

    name: str
    task: str


# 区域名称和任务从统一世界模型派生，保持旧查询接口不变。
TAG_ZONES: dict[int, ZoneInfo] = {
    tag_id: {"name": str(tag_info["name"]), "task": str(tag_info["task"])}
    for tag_id, tag_info in WORLD_TAG_MAP.items()
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
