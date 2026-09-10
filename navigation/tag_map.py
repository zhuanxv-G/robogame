"""RoboGame 比赛场地的 AprilTag 语义地图。"""


# x、y、yaw、tag_size 暂时留空，完成场地标定后可直接填入。
TAG_MAP = {
    1: {
        "name": "start_zone",
        "purpose": "启动区",
        "x": None,
        "y": None,
        "yaw": None,
        "tag_size": None,
    },
    2: {
        "name": "orange_material",
        "purpose": "橙色建筑材料区",
        "x": None,
        "y": None,
        "yaw": None,
        "tag_size": None,
    },
    3: {
        "name": "purple_material",
        "purpose": "紫色建筑材料区",
        "x": None,
        "y": None,
        "yaw": None,
        "tag_size": None,
    },
    4: {
        "name": "build_zone",
        "purpose": "搭建区域",
        "x": None,
        "y": None,
        "yaw": None,
        "tag_size": None,
    },
    5: {
        "name": "right_area_reference",
        "purpose": "右侧区域辅助定位",
        "x": None,
        "y": None,
        "yaw": None,
        "tag_size": None,
    },
    6: {
        "name": "bottom_area_reference",
        "purpose": "底部区域辅助定位",
        "x": None,
        "y": None,
        "yaw": None,
        "tag_size": None,
    },
}
