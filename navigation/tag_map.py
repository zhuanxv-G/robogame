"""Semantic AprilTag map for the RoboGame field."""


# x, y and yaw are reserved for field calibration data added in a later stage.
TAG_MAP = {
    1: {
        "name": "start_zone",
        "purpose": "启动区",
        "x": None,
        "y": None,
        "yaw": None,
    },
    2: {
        "name": "orange_material",
        "purpose": "橙色材料区",
        "x": None,
        "y": None,
        "yaw": None,
    },
    3: {
        "name": "purple_material",
        "purpose": "紫色材料区",
        "x": None,
        "y": None,
        "yaw": None,
    },
    4: {
        "name": "build_zone",
        "purpose": "搭建区域",
        "x": None,
        "y": None,
        "yaw": None,
    },
    5: {
        "name": "right_area_reference",
        "purpose": "右侧区域辅助定位",
        "x": None,
        "y": None,
        "yaw": None,
    },
    6: {
        "name": "bottom_area_reference",
        "purpose": "底部区域辅助定位",
        "x": None,
        "y": None,
        "yaw": None,
    },
}
