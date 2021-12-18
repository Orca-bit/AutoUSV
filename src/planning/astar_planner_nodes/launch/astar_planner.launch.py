import ament_index_python
import launch
import launch_ros.actions

def generate_launch_description():
    """Launch astar_planner with default configuration."""
    # -------------------------------- Nodes-----------------------------------
    astar_planner_node = launch_ros.actions.Node(
        package='astar_planner_nodes',
        executable='astar_planner_node_exe',
        name='astar_planner',
        namespace='planning',
        output='screen',
        parameters=[
            "{}/param/defaults.param.yaml".format(
                ament_index_python.get_package_share_directory(
                    "astar_planner_nodes"
                )
            ),
        ]
    )

    ld = launch.LaunchDescription([astar_planner_node])
    return ld
