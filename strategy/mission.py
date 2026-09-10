"""比赛任务流程的基础状态定义。"""

from enum import Enum
from typing import TypedDict


class MissionState(str, Enum):
    """比赛任务状态。"""

    START = "START"
    FOLLOW_LINE = "FOLLOW_LINE"
    GET_ORANGE = "GET_ORANGE"
    GET_PURPLE = "GET_PURPLE"
    BUILD = "BUILD"


# 顺序元组只描述比赛流程，不在这里实现运动或状态切换逻辑。
MISSION_FLOW = (
    MissionState.START,
    MissionState.FOLLOW_LINE,
    MissionState.GET_ORANGE,
    MissionState.GET_PURPLE,
    MissionState.BUILD,
)


class MissionInfo(TypedDict):
    """每个任务关联的目标区域、标签和视觉能力。"""

    target_zone: str
    target_tag: int | None
    vision: str


# Tag关联沿用项目语义；规则只规定机器人依次取材料并前往搭建区。
MISSION_PLAN: dict[MissionState, MissionInfo] = {
    MissionState.START: {
        "target_zone": "start_zone",
        "target_tag": 1,
        "vision": "AprilTag startup localization",
    },
    MissionState.FOLLOW_LINE: {
        "target_zone": "orange_material",
        "target_tag": 2,
        "vision": "line following and AprilTag search",
    },
    MissionState.GET_ORANGE: {
        "target_zone": "orange_material",
        "target_tag": 2,
        "vision": "K230D orange detection",
    },
    MissionState.GET_PURPLE: {
        "target_zone": "purple_material",
        "target_tag": 3,
        "vision": "K230D purple detection",
    },
    MissionState.BUILD: {
        "target_zone": "build_zone",
        "target_tag": 4,
        "vision": "AprilTag alignment",
    },
}
