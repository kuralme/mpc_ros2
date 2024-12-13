import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description():
    # map_frame_arg = DeclareLaunchArgument('map_frame', default_value='map',
    #                                      description='Map ros2 tf frame name')
    # odom_frame_arg = DeclareLaunchArgument('odom_frame', default_value='odom',
    #                                        description='Odometry ros2 tf frame name')
    # base_link_frame_arg = DeclareLaunchArgument('base_frame', default_value='base_link',
    #                                             description='Base link frame id')
    # odom_topic_arg = DeclareLaunchArgument('odom_topic_name', default_value='odom',
    #                                        description='Odometry ros2 topic name')
    # sim_control_rate_arg = DeclareLaunchArgument('control_rate', default_value='50',
    #                                             description='Simulation control loop update rate')
    # pub_odom_tf_arg = DeclareLaunchArgument('pub_odom_tf', default_value='false',
    #                                              description='Odometry TF')
    
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

