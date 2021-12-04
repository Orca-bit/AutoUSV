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

    # In combination 'basic' and 'high_level' control
    # in what mode of control commands to operate in,
    # only one of them can be active at a time with a value
    # ============================== now only 'basic' ==========================
    control_command_param = DeclareLaunchArgument(
        'control_command',
        default_value="basic",  # use "basic" or "high_level"
        description='command control mode topic name')

    # Default joystick translator params
    joy_translator_param = DeclareLaunchArgument(
        'joy_translator_param',
        default_value=[
            get_param('joystick_interface_node', 'param/logitech_f310.default.param.yaml')
        ],
        description='Path to config file for joystick translator')

    # -------------------------------- Nodes-----------------------------------

    # joystick driver node
    joy = launch_ros.actions.Node(
        package='joy_linux',
        executable='joy_linux_node',
        output='screen')

    # joystick translator node
    joy_translator = launch_ros.actions.Node(
        package='joystick_interface_node',
        executable='joystick_interface_node_exe',
        output='screen',
        parameters=[
            LaunchConfiguration('joy_translator_param'),
            # overwrite parameters from yaml here
            {"control_command": LaunchConfiguration('control_command')}
        ],
        remappings=[
            ("basic_cmd", "usv/basic_cmd")
        ])

    ld = launch.LaunchDescription([
        control_command_param,
        joy_translator_param,
        joy,
        joy_translator])
    return ld
