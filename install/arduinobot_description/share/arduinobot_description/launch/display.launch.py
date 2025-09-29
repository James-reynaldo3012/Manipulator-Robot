from launch import LaunchDescription  # Container for launch entities
from launch_ros.actions import Node  # Action to start a ROS 2 node
from launch_ros.parameter_descriptions import ParameterValue  # Wraps parameter with substitutions
from launch.actions import DeclareLaunchArgument  # Declares a CLI/launch argument
from launch.substitutions import Command, LaunchConfiguration  # Substitutions evaluated at runtime
import os
from ament_index_python.packages import get_package_share_directory  # Finds package share directory

def generate_launch_description():
    # Declare a launch argument to allow overriding the URDF/Xacro path
    model_arg = DeclareLaunchArgument(
        name="model", 
        default_value=os.path.join(get_package_share_directory("arduinobot_description"), "urdf", "arduinobot.urdf.xacro"),
        description="Absolute path to the robot URDF file"   
    )
    # Evaluate xacro at launch time to produce the URDF string for robot_description
    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]))

    # Publishes TF from the kinematic tree defined by robot_description
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    # GUI to interactively move joints and publish joint states
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )

    # Launch RViz with a predefined configuration
    rviz_node =Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(get_package_share_directory("arduinobot_description"),"rviz","display.rviz")]
    )
    # Order matters only for declared arguments vs. their consumers
    return LaunchDescription([
        model_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz_node
    ])