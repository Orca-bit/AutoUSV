import launch
import launch_ros.actions
import launch.substitutions
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import ament_index_python
import os

def get_param(package_name, param_file):
    return os.path.join(ament_index_python.get_package_share_directory(package_name), param_file)


def generate_launch_description():

    gnss_param = DeclareLaunchArgument(
        'gnss_param',
        default_value=[
            get_param('gnss', 'param/gnss_interface.param.yaml')
        ],
        description='Path to config file for gnss_interface')

    # -------------------------------- Nodes-----------------------------------

    # gnss_interface node
    gnss_interface = launch_ros.actions.Node(
        package='gnss',
        executable='gnss_interface_exe',
        output='screen',
        parameters=[
            LaunchConfiguration('gnss_param'),
        ],
        remappings=[
            ("gnss_report", "/usv/gnss_report")
        ])

    ld = launch.LaunchDescription([
        gnss_param,
        gnss_interface])
    return ld
