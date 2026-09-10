"""比赛任务流程的基础状态定义。"""

from enum import Enum


class MissionState(str, Enum):
    """比赛任务状态。"""

    START = "START"
    GET_ORANGE = "GET_ORANGE"
    GET_PURPLE = "GET_PURPLE"
    BUILD = "BUILD"


# 顺序元组只描述比赛流程，不在这里实现运动或状态切换逻辑。
MISSION_FLOW = (
    MissionState.START,
    MissionState.GET_ORANGE,
    MissionState.GET_PURPLE,
    MissionState.BUILD,
)
