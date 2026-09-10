"""依据 RoboGame2026 竞技组规则手册 3.0 建立的场地世界模型。"""


# 数值单位为厘米，角度单位为度。坐标原点和方向是本项目约定，规则未定义坐标系。
FIELD_INFO = {
    "width_cm": 720.0,
    "height_cm": 480.0,
    "official_length_cm": 720.0,
    "official_width_cm": 480.0,
    "boundary_width_cm": 40.0,
    "boundary_height_cm": 40.0,
    "line_width_cm": 5.0,
    "line_width_tolerance_percent": 5.0,
    "tag_family": "tag36h11",
    "tag_size_cm": (15.0, 15.0),
    "tag_top_edge_height_cm": 40.0,
    "coordinate_unit": "cm",
    "angle_unit": "degree",
    "coordinate_system": {
        "origin": "full_field_outer_lower_left",
        "x_axis": "right_along_official_field_length",
        "y_axis": "up_along_official_field_width",
        "yaw_zero": "positive_x",
        "yaw_positive": "counterclockwise",
        "source": "project_convention_not_defined_by_rules",
    },
    "symmetry": "central_symmetry",
    "source": "rules_3.1.2_and_3.1.8",
}


def _unknown_position(description: str) -> dict[str, object]:
    """生成规则示意图中只有相对描述、没有绝对尺寸的位置。"""
    return {
        "x": None,
        "y": None,
        "description": description,
        "status": "TODO: official absolute coordinates are not dimensioned",
    }


ZONE_MAP = {
    "start_zone": {
        "position": _unknown_position("对应半场外侧角落，位置见图3.3"),
        "size": {"length_cm": 60.0, "width_cm": 60.0},
        "height": None,
        "height_status": "TODO: rules do not state a separate surface height",
        "task": "initialize",
        "purpose": "放置机器人并初始化比赛状态",
        "source": "rules_3.1.3",
    },
    "build_zone": {
        "position": _unknown_position("每个半场靠外侧边界的搭建区域，位置见图3.4"),
        "size": {
            "length_cm": 240.0,
            "width_cm": 60.0,
            "approximate": True,
        },
        "height": 10.0,
        "task": "build",
        "purpose": "有效搭建和计分区域",
        "source": "rules_3.1.4",
    },
    "central_highland": {
        "position": _unknown_position("场地中央，双方中心对称，范围见图3.5"),
        "size": {"length_cm": 180.0, "width_cm": 170.0, "per_side": True},
        "height": 20.0,
        "task": "transit_and_material_access",
        "purpose": "中央高台及材料区域的承载结构",
        "source": "rules_3.1.5",
    },
    "highland_ramp": {
        "position": _unknown_position("连接地面与中央高台，绝对位置见图3.5"),
        "size": {"length_cm": 180.0, "width_cm": 80.0},
        "height": None,
        "slope_deg": 14.0,
        "slope_approximate": True,
        "task": "highland_access",
        "purpose": "机器人上下中央高地区",
        "source": "rules_3.1.5",
    },
    "orange_material": {
        "position": _unknown_position("中央高地区的墙体建筑材料区，位置见图3.6"),
        "size": {"length_cm": 180.0, "width_cm": 30.0},
        "height": None,
        "height_status": "TODO: material-zone surface height is not stated separately",
        "task": "collect_orange",
        "purpose": "放置橙色墙体建筑方块",
        "slot": {
            "length_cm": 150.0,
            "width_cm": 11.0,
            "depth_cm": 5.0,
            "distance_to_short_edges_cm": 40.0,
            "distance_to_long_edges_cm": 9.5,
            "official_dimension_conflict": (
                "180 cm zone length, 150 cm slot length and 40 cm clearance to both "
                "short edges cannot all hold; confirm with organizers"
            ),
        },
        "block_count_per_side": 10,
        "block_count_total": 20,
        "minimum_initial_block_spacing_cm": 5.0,
        "source": "rules_3.1.6",
    },
    "purple_material": {
        "position": _unknown_position("中央高地区的屋顶建筑材料区，位置见图3.7"),
        "size": {"length_cm": 220.0, "width_cm": 40.0},
        "height": None,
        "height_status": "TODO: material-zone surface height is not stated separately",
        "task": "collect_purple",
        "purpose": "放置紫色屋顶建筑方块",
        "slot": {
            "count": 3,
            "length_cm": 11.0,
            "width_cm": 11.0,
            "depth_cm": 5.0,
            "spacing_cm": 20.0,
        },
        "block_count_total": 3,
        "source": "rules_3.1.7",
    },
    "gray_corridor": {
        "position": _unknown_position("图3.8中的灰色过道，横向巡线位于其中线"),
        "size": None,
        "size_status": "TODO: corridor dimensions are not stated",
        "height": None,
        "height_status": "TODO: corridor height is not stated",
        "task": "transit",
        "purpose": "连接启动区、材料区与搭建区的主要通道",
        "source": "rules_3.1.8_and_figure_3.8",
    },
    # 以下两项是项目使用的逻辑定位区域，不是规则命名的物理区域。
    "right_area_reference": {
        "position": _unknown_position("右侧外墙上的辅助定位参考，见图3.9 Tag 5"),
        "size": None,
        "height": None,
        "task": "localize",
        "purpose": "右侧区域辅助定位",
        "source": "project_semantics_with_official_figure_3.9_location",
    },
    "bottom_area_reference": {
        "position": _unknown_position("底部外墙上的辅助定位参考，见图3.9 Tag 6"),
        "size": None,
        "height": None,
        "task": "localize",
        "purpose": "底部区域辅助定位",
        "source": "project_semantics_with_official_figure_3.9_location",
    },
}


# 图3.9仅给出相对位置。x、y、yaw没有尺寸标注，禁止从示意图像素比例推算。
TAG_MAP = {
    1: {
        "name": "start_zone",
        "zone": "start_zone",
        "x": None,
        "y": None,
        "yaw": None,
        "relative_position": "left_outer_wall_on_gray_corridor_centerline",
        "purpose": "初始化机器人状态",
        "task": "initialize",
        "source": "figure_3.9_location_and_project_semantics",
    },
    2: {
        "name": "orange_material",
        "zone": "orange_material",
        "x": None,
        "y": None,
        "yaw": None,
        "relative_position": "left_wall_material_centerline_endpoint",
        "purpose": "橙色材料区定位",
        "task": "collect_orange",
        "source": "figure_3.9_location_and_project_semantics",
    },
    3: {
        "name": "purple_material",
        "zone": "purple_material",
        "x": None,
        "y": None,
        "yaw": None,
        "relative_position": "roof_material_centerline_endpoint",
        "purpose": "紫色材料区定位",
        "task": "collect_purple",
        "source": "figure_3.9_location_and_project_semantics",
    },
    4: {
        "name": "build_zone",
        "zone": "build_zone",
        "x": None,
        "y": None,
        "yaw": None,
        "relative_position": "right_wall_material_centerline_endpoint",
        "purpose": "搭建区对准",
        "task": "build",
        "source": "figure_3.9_location_and_project_semantics",
    },
    5: {
        "name": "right_area_reference",
        "zone": "right_area_reference",
        "x": None,
        "y": None,
        "yaw": None,
        "relative_position": "right_outer_wall_on_gray_corridor_centerline",
        "purpose": "右侧区域辅助定位",
        "task": "localize",
        "source": "figure_3.9_location_and_project_semantics",
    },
    6: {
        "name": "bottom_area_reference",
        "zone": "bottom_area_reference",
        "x": None,
        "y": None,
        "yaw": None,
        "relative_position": "bottom_outer_wall_on_right_vertical_line_extension",
        "purpose": "底部区域辅助定位",
        "task": "localize",
        "source": "figure_3.9_location_and_project_semantics",
    },
}

for tag_info in TAG_MAP.values():
    tag_info["size_cm"] = FIELD_INFO["tag_size_cm"]
    tag_info["top_edge_height_cm"] = FIELD_INFO["tag_top_edge_height_cm"]
    tag_info["coordinate_status"] = "TODO: measure x, y and inward-facing yaw on site"


# 拓扑来自规则3.1.8及图3.8/3.9，不包含未标注的线段长度和节点坐标。
LINE_MAP = {
    "line_width_cm": FIELD_INFO["line_width_cm"],
    "width_tolerance_percent": FIELD_INFO["line_width_tolerance_percent"],
    "segments": {
        "start_zone_perimeter": {
            "kind": "closed_boundary",
            "zone": "start_zone",
            "coordinates": None,
        },
        "start_centerline": {
            "orientation": "vertical_in_figure_3.8",
            "connects": ("start_zone", "start_t_junction"),
            "coordinates": None,
        },
        "left_wall_material_centerline": {
            "orientation": "vertical_in_figure_3.8",
            "connects": ("left_material_t_junction", "tag_2_endpoint"),
            "coordinates": None,
        },
        "right_wall_material_centerline": {
            "orientation": "vertical_in_figure_3.8",
            "connects": (
                "right_material_t_junction",
                "roof_material_t_junction",
                "tag_4_endpoint",
            ),
            "coordinates": None,
        },
        "roof_material_centerline": {
            "orientation": "horizontal_in_figure_3.8",
            "connects": ("tag_3_endpoint", "roof_material_t_junction"),
            "coordinates": None,
        },
        "gray_corridor_centerline": {
            "orientation": "horizontal_in_figure_3.8",
            "connects": (
                "tag_1_extension",
                "start_t_junction",
                "left_material_t_junction",
                "right_material_t_junction",
                "tag_5_extension",
            ),
            "coordinates": None,
        },
    },
    "nodes": {
        "start_t_junction": {"type": "T", "position": None},
        "left_material_t_junction": {"type": "T", "position": None},
        "right_material_t_junction": {"type": "T", "position": None},
        "roof_material_t_junction": {"type": "T", "position": None},
    },
    "routes": {
        "start_to_main_corridor": (
            "start_zone",
            "start_centerline",
            "start_t_junction",
            "gray_corridor_centerline",
        ),
        "main_corridor_to_orange_reference": (
            "gray_corridor_centerline",
            "left_material_t_junction",
            "left_wall_material_centerline",
            "tag_2_endpoint",
        ),
        "main_corridor_to_purple_reference": (
            "gray_corridor_centerline",
            "right_material_t_junction",
            "right_wall_material_centerline",
            "roof_material_t_junction",
            "roof_material_centerline",
            "tag_3_endpoint",
        ),
        "main_corridor_to_build_reference": (
            "gray_corridor_centerline",
            "right_material_t_junction",
            "right_wall_material_centerline",
            "tag_4_endpoint",
        ),
    },
    "coordinate_status": "TODO: measure all segment endpoints and T-junction coordinates",
    "source": "rules_3.1.8_and_figures_3.8_3.9",
}
