"""MAI City demo (ROS 2): play a converted MAI City bag with `ros2 bag play --clock`.

    ros2 launch lidar_obstacle_detector mai_city.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    share = get_package_share_directory("lidar_obstacle_detector")
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, "launch", "obstacle_detector.launch.py")),
            launch_arguments={
                "points": "/velodyne_points",
                "use_sim_time": "true",
                "rviz": "true",
            }.items(),
        )
    ])
