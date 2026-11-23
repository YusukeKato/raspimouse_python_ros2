"""Start the Raspberry Pi Mouse node with the default parameters."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_params = (get_package_share_directory('raspimouse_python_ros2')
                      + '/config/raspimouse.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='YAML file with the node parameters'),
        DeclareLaunchArgument('initial_motor_power', default_value='false',
                              description='turn the motor power on at start'),
        DeclareLaunchArgument('use_mock_backend', default_value='false',
                              description='use fake hardware (to try the node on a PC)'),
        Node(
            package='raspimouse_python_ros2',
            executable='raspimouse_node',
            name='raspimouse',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {
                    'initial_motor_power': ParameterValue(
                        LaunchConfiguration('initial_motor_power'), value_type=bool),
                    'use_mock_backend': ParameterValue(
                        LaunchConfiguration('use_mock_backend'), value_type=bool),
                },
            ],
        ),
    ])
