"""比赛导航状态机的基础状态定义。"""

from enum import Enum


class NavigationState(str, Enum):
    """预留给上层比赛状态机使用的导航阶段。"""

    LINE_FOLLOW = "LINE_FOLLOW"
    SEARCH_TAG = "SEARCH_TAG"
    TAG_ALIGN = "TAG_ALIGN"
    TASK_EXECUTE = "TASK_EXECUTE"
