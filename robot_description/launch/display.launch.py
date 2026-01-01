from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


#  Define the path to the URDF file
urdf_path = os.path.join(
    get_package_share_directory("robot_description"),
    "urdf",
    "mobile_manipulator.urdf.xacro",
)
