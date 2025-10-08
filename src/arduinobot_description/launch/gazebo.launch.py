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
    ros_distro = os.environ["ROS_DISTRO"]
    is_ignition = "True" if ros_distro == "humble" else "False"
    # Use ODE physics engine for better mimic joint support
    # physics_engine = "--physics-engine ode" if ros_distro != "humble" else ""
    # Evaluate xacro at launch time to produce the URDF string for robot_description
    robot_description = ParameterValue(Command([
        "xacro ", 
        LaunchConfiguration("model"),
        " is_ignition:=",
        is_ignition
        ]), 
        value_type=str)

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
            ("gz_args", [" -v 4 -r empty.sdf "])
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
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
            "/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"
        ]
    )

    # Path to ROS 2 control config file
    ros2_control_config = os.path.join(
        arduinobot_description_dir, "config", "arduinobot_controllers.yaml"
    )

    # Controller manager node for ros2_control
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},
            ros2_control_config
        ],
        output="screen"
    )

    return LaunchDescription([
        model_arg,
        gazebo_resourche_path,
        robot_state_publisher,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge,
        controller_manager,
    ])