# launch file to run the 'frontal_free_pc' node.
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node

this_pkg = 'frontal_free_pc'
pkg_dir = get_package_share_directory(this_pkg)

def generate_launch_description():

    frontal_free_pc_node = Node(
        package=this_pkg,
        executable='frontal_free_pc',
        remappings=[('cloud', 'camera/camera/depth/color/points'), ], #RGBD camera topic
        	# realsense: camera/camera/depth/color/points,
            # simulation: camera/points,
        parameters=[os.path.join( pkg_dir, 'params', 'parameters.yaml')],
        # namespace='r1'
    )
    
    return LaunchDescription([
        frontal_free_pc_node,
        LogInfo(msg="\n\n---- node_launch.py, frontal_free_pc node running ----\n\n"),
    ])
