import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'cone_path_planner'
    
    config_path = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        'planning_params.yaml'
    )

    return LaunchDescription([
        Node(
            package=pkg_name,
            executable='path_planner_node',
            name='path_planner_node',
            parameters=[config_path, {'use_sim_time': False}], # for bag play (True)
            output='screen'
        )
    ])
