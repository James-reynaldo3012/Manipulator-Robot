from launch import LaunchDescription
from launch_ros.actions import Node
import os
from pathlib import Path
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription

def generate_launch_description():
    # Declare a launch argument to allow overriding the URDF/Xacro path
    arduinobot_description_dir = get_package_share_directory("arduinobot_description")
    model_arg = DeclareLaunchArgument(
        name="model", 
        default_value=os.path.join(arduinobot_description_dir, "urdf", "arduinobot.urdf.xacro"),
        description="Absolute path to the robot URDF file"   
    )
    # Evaluate xacro at launch time to produce the URDF string for robot_description
    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]), value_type=str)

    gazebo_resourche_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            str(Path(arduinobot_description_dir).parent.resolve())
        ]
    )

    # Publishes TF from the kinematic tree defined by robot_description
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time": True}]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch"
            ), "/gz_sim.launch.py"]
        ),
        launch_arguments=[
            ("gz_args", [" -v 4 -r empty.sdf"])
        ]
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=("-topic", "robot_description",
                   "-name", "arduinobot")
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"
        ]
    )

    return LaunchDescription([
        model_arg,
        gazebo_resourche_path,
        robot_state_publisher,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge,
    ])