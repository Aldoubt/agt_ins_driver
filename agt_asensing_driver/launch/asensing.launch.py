from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(get_package_share_directory('agt_asensing_driver'), 'config', 'asensing.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('baudrate', default_value='230400'),
        Node(package='agt_asensing_driver', executable='asensing_node',
             name='asensing_driver', output='screen', parameters=[config, {
                 'port': LaunchConfiguration('port'),
                 'baudrate': ParameterValue(LaunchConfiguration('baudrate'), value_type=int),
             }]),
    ])
