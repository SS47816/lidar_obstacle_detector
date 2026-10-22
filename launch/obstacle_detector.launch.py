"""Launch the obstacle detector (ROS 2).

ros2 launch lidar_obstacle_detector obstacle_detector.launch.py points:=/velodyne_points
ros2 launch lidar_obstacle_detector obstacle_detector.launch.py preset:=accurate rviz:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    share = get_package_share_directory("lidar_obstacle_detector")
    args = [
        DeclareLaunchArgument(
            "points", default_value="/points", description="input PointCloud2 topic"
        ),
        DeclareLaunchArgument(
            "params", default_value=os.path.join(share, "config", "obstacle_detector.yaml")
        ),
        DeclareLaunchArgument(
            "preset", default_value="fast", description="legacy | fast | accurate"
        ),
        DeclareLaunchArgument("target_frame", default_value=""),
        DeclareLaunchArgument("tracking_frame", default_value=""),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="false"),
    ]
    detector = Node(
        package="lidar_obstacle_detector",
        executable="obstacle_detector_node",
        name="obstacle_detector",
        output="screen",
        parameters=[
            LaunchConfiguration("params"),
            {
                "preset": LaunchConfiguration("preset"),
                "target_frame": LaunchConfiguration("target_frame"),
                "tracking_frame": LaunchConfiguration("tracking_frame"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            },
        ],
        remappings=[("points", LaunchConfiguration("points"))],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(share, "rviz", "obstacle_detector_ros2.rviz")],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )
    return LaunchDescription(args + [detector, rviz])
