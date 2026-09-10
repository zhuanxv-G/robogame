"""兼容旧接口的 AprilTag 语义地图。"""

from .field_model import TAG_MAP as WORLD_TAG_MAP


# 字段名称保持不变，数据统一从 field_model.py 派生。
TAG_MAP = {
    tag_id: {
        "name": tag_info["name"],
        "purpose": tag_info["purpose"],
        "x": tag_info["x"],
        "y": tag_info["y"],
        "yaw": tag_info["yaw"],
        "tag_size": tag_info["size_cm"],
    }
    for tag_id, tag_info in WORLD_TAG_MAP.items()
}
