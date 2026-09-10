"""Competition mission state definitions."""

from enum import Enum


class MissionState(str, Enum):
    """Top-level phases and terminal states of a RoboGame mission."""

    START = "START"
    FOLLOW_LINE = "FOLLOW_LINE"
    GET_ORANGE = "GET_ORANGE"
    GET_PURPLE = "GET_PURPLE"
    BUILD = "BUILD"
    FINISH = "FINISH"
    FAULT = "FAULT"
