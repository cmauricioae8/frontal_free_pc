# launch to add a component to an existing container, if not exist, this launch will be waiting
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode

this_pkg = 'frontal_free_pc'
pkg_dir = get_package_share_directory(this_pkg)

def generate_launch_description():

    load_composable_nodes = LoadComposableNodes( #to load components into a given container
        target_container='frontal_free_container',
        composable_node_descriptions=[
            ComposableNode(
                package='frontal_free_pc',
                plugin='free_pc_ns::FrontalFreePC',
                name='frontal_free_pc2',
                # namespace='r1',
                remappings=[('cloud', 'camera/points')],
                parameters=[os.path.join( pkg_dir, 'params', 'parameters.yaml')],
            ),
        ],
    )

    return LaunchDescription([load_composable_nodes])