from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_path
import os


def generate_launch_description():
    # Define the path to the URDF file
    urdf_path = os.path.join(
        get_package_share_path("robot_description"),
        "urdf",
        "mobile_manipulator.urdf.xacro",
    )

    rviz_config_path = os.path.join(
        get_package_share_path("robot_description"),
        "rviz",
        "rviz_config.rviz",
    )

    gazebo_world_path = os.path.join(
        get_package_share_path("robot_bringup"),
        "world",
        "robot_world.sdf",
    )

    gazebo_bridge_config = os.path.join(
        get_package_share_path("robot_bringup"),
        "config",
        "gazebo_bridge.yaml",
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
    # RViz node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_path],
        output="screen",
    )

    # Node to publish joint states (uncomment to use GUI)
    # joint_state_publisher_node = Node(
    #     package="joint_state_publisher_gui",
    #     executable="joint_state_publisher_gui",
    #     name="joint_state_publisher_gui",
    # )
    # Include Gazebo launch file
    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(get_package_share_path("ros_gz_sim") / "launch" / "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": f"-r {gazebo_world_path}"}.items(),
    )

    # Node to spawn the robot into Gazebo
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "robot_description", "-name", "mobile_manipulator"],
        output="screen",
    )

    bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge",
        parameters=[{"config_file": gazebo_bridge_config}],
        output="screen",
    )

    return LaunchDescription(
        [
            gz_launch,
            robot_state_publisher_node,
            # Uncomment to use GUI
            # joint_state_publisher_node,
            rviz_node,
            spawn_entity,
            bridge_node,
        ]
    )
