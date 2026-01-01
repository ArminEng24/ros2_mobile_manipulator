from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_path
import os


def generate_launch_description():
    #  Define the path to the URDF file
    urdf_path = os.path.join(
        get_package_share_path("robot_description"),
        "urdf",
        "mobile_manipulator.urdf.xacro",
    )

    # Define the path to the RViz configuration file
    rviz_config_path = os.path.join(
        get_package_share_path("robot_description"),
        "rviz",
        "mobile_manipulator.rviz",
    )
    # Generate xacro command to process the URDF file
    robot_description = ParameterValue(Command(["xacro ", urdf_path]), value_type=str)

    # Node to publish the robot state
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    # Node to publish joint states
    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
    )

    # Node to launch RViz2 with the specified configuration
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_path],
        output="screen",
    )

    return LaunchDescription(
        [robot_state_publisher_node, joint_state_publisher_node, rviz_node]
    )
