"""Offline visualization of localization against the prior map: map_matcher_node
(also latches the prior map on /localization/map) + bag play + rviz2.

  ros2 launch go2_localization offline_view.launch.py \
       bag:=~/maps/loc_test_2_1_plio start_offset:=0 rate:=1.0

The matcher is tracking-only, so the robot must start near the map origin —
otherwise set the pose in RViz with "2D Pose Estimate" (/initialpose).
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    pkg = get_package_share_directory('go2_localization')
    cfg = os.path.join(pkg, 'config', 'localization.yaml')
    rviz = os.path.join(pkg, 'rviz', 'localization.rviz')

    # resolve at runtime so ~ and relative paths work
    bag = os.path.abspath(os.path.expanduser(LaunchConfiguration('bag').perform(context)))
    rate = LaunchConfiguration('rate').perform(context)
    start_offset = LaunchConfiguration('start_offset').perform(context)

    matcher = Node(
        package='go2_localization', executable='map_matcher_node', name='map_matcher_node',
        parameters=[cfg], output='screen',
    )
    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2', arguments=['-d', rviz],
    )
    # let the matcher load the map before the bag streams
    bag_play = TimerAction(period=3.0, actions=[
        ExecuteProcess(cmd=[
            'ros2', 'bag', 'play', bag, '--rate', rate, '--start-offset', start_offset,
        ], output='screen'),
    ])
    return [matcher, rviz_node, bag_play]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('bag', default_value='/home/dmitriyb51/autonomy_stack_go2/plio_out'),
        DeclareLaunchArgument('start_offset', default_value='0'),
        DeclareLaunchArgument('rate', default_value='1.0'),
        OpaqueFunction(function=launch_setup),
    ])
