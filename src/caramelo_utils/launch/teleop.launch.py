import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    caramelo_utils_pkg = get_package_share_directory('caramelo_utils')

    use_sim_time_arg = DeclareLaunchArgument(name="use_sim_time", default_value="False", description="Use simulated time")

    joy_teleop = Node(
        package="joy_teleop",
        executable="joy_teleop",
        parameters=[
            os.path.join(
                caramelo_utils_pkg,
                "config",
                "joy_teleop.yaml",
            ),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        output="screen",
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joystick",
        parameters=[
            os.path.join(
                caramelo_utils_pkg,
                "config",
                "joy_config.yaml",
            ),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        output="screen",
    )
    
    teleop_keyboard_node = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        name="teleop_keyboard",
        output="screen",
        prefix="xterm -e",
        parameters=[],
        remappings=[
            # Remapeia /cmd_vel para /key_vel para funcionar com twist_mux
            ("/cmd_vel", "/cmd_vel_key"),
        ],
    )
    
    twist_mux_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("twist_mux"),
                "launch",
                "twist_mux_launch.py",
            )
        ),
        launch_arguments={
            "cmd_vel_out": "caramelo_controller/cmd_vel_unstamped",
            "config_locks": os.path.join(caramelo_utils_pkg, "config", "twist_mux_locks.yaml"),
            "config_topics": os.path.join(caramelo_utils_pkg, "config", "twist_mux_topics.yaml"),
            "config_joy": os.path.join(caramelo_utils_pkg, "config", "twist_mux_joy.yaml"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )

    twist_relay_node = Node(
        package="caramelo_utils",
        executable="twist_relay.py",
        name="twist_relay",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {"input_twist_topic": "caramelo_controller/cmd_vel_unstamped"},
            {"output_twist_stamped_topic": "/mecanum_controller/reference"},
            {"frame_id": "base_footprint"},
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            joy_teleop,
            joy_node,
            twist_mux_launch,
            teleop_keyboard_node,
            twist_relay_node
        ]
    )