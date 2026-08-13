"""Этап 4: контроллер на записанном заезде, без реальной езды.

Этап 3 + controller_server (RPP, внутри него local_costmap) + goal_to_controller
вместо goal_to_planner. bt_navigator по-прежнему не поднимаем — изолируем слой.

  ros2 launch go2_navigation nav2_stage4.launch.py
  ros2 topic echo /cmd_vel

Дождись 'Managed nodes are active', жми 'Nav2 Goal'. На старте виден разворот на
месте (vx~0, vyaw!=0). Через ~10 с FollowPath завершится "застрял" — офлайн это
ожидаемо, робот в бэге не слушает /cmd_vel.
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

    # NavFn, внутри global_costmap
    planner = Node(
        package="nav2_planner", executable="planner_server", name="planner_server",
        parameters=[LaunchConfiguration("params")],
        output="screen",
    )

    # RPP + local_costmap: путь через FollowPath -> /cmd_vel
    controller = Node(
        package="nav2_controller", executable="controller_server", name="controller_server",
        parameters=[LaunchConfiguration("params")],
        output="screen",
    )

    # порядок важен: карта -> планировщик -> контроллер
    lifecycle = Node(
        package="nav2_lifecycle_manager", executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        parameters=[{
            "autostart": True,
            "node_names": ["map_server", "planner_server", "controller_server"],
        }],
        output="screen",
    )

    # клик -> путь -> контроллер -> /cmd_vel
    goal_bridge = Node(
        package="go2_navigation", executable="goal_to_controller.py", name="goal_to_controller",
        output="screen",
    )

    rviz = Node(
        package="rviz2", executable="rviz2", name="rviz2",
        arguments=["-d", rviz_cfg],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    # с задержкой: matcher грузит 3D-карту несколько секунд
    bag = TimerAction(period=12.0, actions=[
        ExecuteProcess(
            cmd=["ros2", "bag", "play", LaunchConfiguration("bag"),
                 "--rate", LaunchConfiguration("rate")],
            condition=IfCondition(LaunchConfiguration("play_bag")),
            output="screen",
        ),
    ])

    return LaunchDescription(args + [matcher, tf_setup, map_server, planner, controller,
                                     lifecycle, goal_bridge, rviz, bag])
