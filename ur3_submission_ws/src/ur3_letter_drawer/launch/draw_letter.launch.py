import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def generate_launch_description():
    ur_sim_dir = get_package_share_directory("ur_simulation_gz")
    ur_moveit_dir = get_package_share_directory("ur_moveit_config")
    kinematics_yaml = load_yaml("ur_moveit_config", "config/kinematics.yaml")
    ur_type = LaunchConfiguration("ur_type")
    # ur_moveit_config intentionally loads the URDF from the robot_description topic.
    # A MoveGroupInterface client needs a local copy of that parameter as well.
    robot_description = {
        "robot_description": Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
            PathJoinSubstitution([FindPackageShare("ur_simulation_gz"), "urdf", "ur_gz.urdf.xacro"]), " ",
            "safety_limits:=true ", "safety_pos_margin:=0.15 ", "safety_k_position:=20 ",
            "name:=ur ", "ur_type:=", ur_type, " ", "tf_prefix:=\"\" ",
            "simulation_controllers:=",
            PathJoinSubstitution([FindPackageShare("ur_simulation_gz"), "config", "ur_controllers.yaml"]),
        ])
    }
    robot_description_semantic = {
        "robot_description_semantic": Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
            PathJoinSubstitution([FindPackageShare("ur_moveit_config"), "srdf", "ur.srdf.xacro"]),
            " name:=ur",
        ])
    }

    ur_sim_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ur_sim_dir, "launch", "ur_sim_control.launch.py")),
        launch_arguments={
            "ur_type": ur_type,
            "launch_rviz": "false",
        }.items(),
    )

    ur_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ur_moveit_dir, "launch", "ur_moveit.launch.py")),
        launch_arguments={
            "ur_type": ur_type,
            "use_sim_time": "true",
            "launch_rviz": "true",
        }.items(),
    )

    draw_letter_node = Node(
        package="ur3_letter_drawer",
        executable="draw_letter_n",
        name="ur3_letter_drawer",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            {
                "use_sim_time": True,
                "marker_topic": "/letter_path_n",
                "letter_height": 0.20,
                "letter_width": 0.16,
                "pen_lift": 0.04,
                "eef_step": 0.005,
                "min_fraction": 0.95,
                "robot_description_kinematics": kinematics_yaml,
            }
        ],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "ur_type", default_value="ur3e",
                choices=["ur3", "ur3e"],
                description="UR model used in Gazebo and MoveIt.",
            ),
            ur_sim_control_launch,
            ur_moveit_launch,
            # Gazebo, ros2_control and move_group need time to become ready.
            TimerAction(period=45.0, actions=[draw_letter_node]),
        ]
    )
