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

    # --------------------------------- Params -------------------------------

    # In combination 'high_level', 'basic' control
    # in what mode of control commands to operate in,
    # only one of them can be active at a time with a value
    # ============================== now only 'basic' ==========================
    control_command_param = DeclareLaunchArgument(
        'control_command',
        default_value="basic",  # use "high_level", "basic"
        description='command control mode topic name')

    # Default joystick translator params
    vehicle_param = DeclareLaunchArgument(
        'vehicle_param',
        default_value=[
            get_param('vehicle_interface', 'param/vehicle_interface.param.yaml')
        ],
        description='Path to config file for vehicle_interface')

    # -------------------------------- Nodes-----------------------------------

    # vehicle_interface node
    vehicle_interface = launch_ros.actions.Node(
        package='vehicle_interface',
        executable='vehicle_interface_exe',
        output='screen',
        parameters=[
            LaunchConfiguration('vehicle_param'),
            # overwrite parameters from yaml here
            {"control_command": LaunchConfiguration('control_command')}
        ],
        remappings=[
            ("high_level_cmd", "/usv/high_level_cmd"),
            ("basic_cmd", "/usv/basic_cmd")
        ])

    ld = launch.LaunchDescription([
        control_command_param,
        vehicle_param,
        vehicle_interface])
    return ld
