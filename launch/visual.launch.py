import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Define paths
    pkg_share = get_package_share_directory('dcm_walker')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'rviz.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    tf_publisher_node = Node(
        package='dcm_walker',
        executable='visual_node',
        name='dcm_walker_visual',
        output='screen'
    )

    nodes_to_start = [
        rviz_node,
        tf_publisher_node,
    ]

    return LaunchDescription(nodes_to_start)