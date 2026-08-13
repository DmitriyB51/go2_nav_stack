"""Этап 3: планировщик Nav2 на записанном заезде, без езды (контроллера здесь нет
специально — проверяем один слой, планирование по 2D-карте).

Слои снизу вверх: bag play -> map_matcher_node (TF map->camera_init) -> tf_setup
-> map_server (/map) -> planner_server (NavFn) -> lifecycle_manager -> goal_to_planner
-> rviz2.

⚠️ Узлы Nav2 не работают сразу: unconfigured -> inactive -> active, переводит их
lifecycle_manager. "Всё запустилось, но ничего не происходит" -> смотри его лог.

  ros2 launch go2_navigation nav2_offline.launch.py [play_bag:=false]
Дождись 'Managed nodes are active', жми "Nav2 Goal", кликай в белом коридоре.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("go2_navigation")
    default_map = os.path.join(pkg, "maps", "building_old.yaml")
    default_params = os.path.join(pkg, "config", "nav2_params.yaml")
    default_loc_cfg = os.path.join(pkg, "config", "localization_offline.yaml")
    rviz_cfg = os.path.join(pkg, "rviz", "nav2.rviz")

    args = [
        DeclareLaunchArgument("bag", default_value="/home/dmitriyb51/maps/loc_test_2_1_plio",
                              description="записанный заезд для проигрывания"),
        DeclareLaunchArgument("map", default_value=default_map,
                              description="2D-карта (.yaml) для map_server"),
        DeclareLaunchArgument("params", default_value=default_params,
                              description="конфигурация Nav2"),
        DeclareLaunchArgument("localization_config", default_value=default_loc_cfg,
                              description="конфиг matcher (какая 3D-карта)"),
        DeclareLaunchArgument("play_bag", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("rate", default_value="1.0", description="скорость проигрывания"),
    ]

    # локализация: TF map -> camera_init (роль map->odom в Nav2)
    matcher = Node(
        package="go2_localization", executable="map_matcher_node", name="map_matcher_node",
        parameters=[LaunchConfiguration("localization_config")],
        output="screen",
    )

    # недостающее звено: aft_mapped -> base_link
    tf_setup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, "launch", "tf_setup.launch.py")),
    )

    # .pgm как /map, защёлкнуто
    map_server = Node(
        package="nav2_map_server", executable="map_server", name="map_server",
        parameters=[{"yaml_filename": LaunchConfiguration("map")}],
        output="screen",
    )

    # планировщик, внутри global_costmap (стены + inflation)
    planner = Node(
        package="nav2_planner", executable="planner_server", name="planner_server",
        parameters=[LaunchConfiguration("params")],
        output="screen",
    )

    # переводит узлы Nav2 в active, иначе они молчат
    lifecycle = Node(
        package="nav2_lifecycle_manager", executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        parameters=[{
            "autostart": True,
            # карта раньше планировщика, иначе costmap стартует без статического слоя
            "node_names": ["map_server", "planner_server"],
        }],
        output="screen",
    )

    # клик в RViz -> планировщик -> /plan
    goal_bridge = Node(
        package="go2_navigation", executable="goal_to_planner.py", name="goal_to_planner",
        output="screen",
    )

    rviz = Node(
        package="rviz2", executable="rviz2", name="rviz2",
        arguments=["-d", rviz_cfg],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    # с задержкой: matcher грузит 3D-карту несколько секунд, раньше слать сканы
    # бессмысленно
    bag = TimerAction(period=12.0, actions=[
        ExecuteProcess(
            cmd=["ros2", "bag", "play", LaunchConfiguration("bag"),
                 "--rate", LaunchConfiguration("rate")],
            condition=IfCondition(LaunchConfiguration("play_bag")),
            output="screen",
        ),
    ])

    return LaunchDescription(args + [matcher, tf_setup, map_server, planner,
                                     lifecycle, goal_bridge, rviz, bag])
