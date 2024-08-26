# launch file to create a container and to load a component
import launch
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

this_pkg = 'frontal_free_pc'
pkg_dir = get_package_share_directory(this_pkg)

def generate_launch_description():
    container = ComposableNodeContainer(
        name='frontal_free_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='frontal_free_pc',
                plugin='free_pc_ns::FrontalFreePC',
                namespace='',
                remappings=[('cloud', 'camera/points')],
                parameters=[os.path.join( pkg_dir, 'params', 'parameters.yaml')],
                name='frontal_free_pc'
            ),
        ],
        output='screen',
    )

    return launch.LaunchDescription([container])