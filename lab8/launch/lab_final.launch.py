"""
lab_final.launch.py
====================
One-command launch for the full maze race sequence:
  1. SLAM Toolbox  (builds map in real-time)
  2. Nav2          (path planning + navigation)
  3. Brain Node    (state machine orchestrator, also spawns aruco_markers)

Usage:
  ros2 launch lab_final.launch.py
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # ── Package paths ──────────────────────────────────────────────────────
    tb4_nav_dir  = get_package_share_directory('turtlebot4_navigation')
    tb4_slam_dir = get_package_share_directory('turtlebot4_navigation')

    # ── 1. SLAM Toolbox (online async) ────────────────────────────────────
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb4_slam_dir, 'launch', 'slam.launch.py')
        ),
        launch_arguments={'sync': 'false'}.items(),
    )

    # ── 2. Nav2 (delayed 5s to give SLAM time to init) ────────────────────
    nav2 = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(tb4_nav_dir, 'launch', 'nav2.launch.py')
                ),
            )
        ]
    )

    # ── 3. Brain Node (delayed 10s to give Nav2 time to init) ────────────
    brain = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='lab8',               # ganti dengan nama package kamu
                executable='brain_node_auto', # sesuaikan dengan entry_point di setup.py
                name='robot_brain',
                output='screen',
                emulate_tty=True,
            )
        ]
    )

    return LaunchDescription([
        slam,
        nav2,
        brain,
    ])
