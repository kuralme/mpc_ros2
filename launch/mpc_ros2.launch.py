import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description():
    
    mpc_parameters_file = LaunchConfiguration('mpc_ros2')
    
    default_mpc_parameter_file = os.path.join(
        get_package_share_directory('mpc_ros2'),
        'mpc_ros2_params.yaml'
    )
    mpc_parameters_cmd = DeclareLaunchArgument(
        'mpc_ros2',
        default_value=default_mpc_parameter_file,
        description='MPC / ROS2 configuration file'
    )

    mpc_ros2_node = Node(
        package='mpc_ros2',
        executable='mpc_ros2',
        output='screen',
        name='mpc_ros2_node',
        emulate_tty=True,
        remappings = [
            ('odom', '/wheel/odom'),
        ],
        parameters = [ParameterFile(mpc_parameters_file, allow_substs=True)]
    )

    ld = LaunchDescription()
    ld.add_action(mpc_parameters_cmd)
    ld.add_action(mpc_ros2_node)
    return ld

