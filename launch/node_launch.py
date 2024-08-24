# launch file to run the 'frontal_free_pc' node.
from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node

def generate_launch_description():

    frontal_free_pc_node = Node(
        package='frontal_free_pc',
        executable='frontal_free_pc',
        remappings=[('cloud', 'camera/camera/depth/color/points'), ], #RGBD camera topic
        	# realsense: camera/camera/depth/color/points,
            # simulation (classic Gazebo): camera/points,
            # oakd-lite: stereo/points, NOT TESTED
        parameters=[{
            'max_depth_x': 4.0, # maximum depth distance to consider a possible crash (in front of the robot)
            'robot_width': 0.8,
            'min_height': 0.01, # minimum vertical distance to take into account an object in front
            'max_height': 1.70, # maximum vertical distance to take into account an object in front
            'target_frame': 'base_footprint',
            'debug_mode': True, # ROS param to publish the nearest point computed by the node
            'reduce_resolution': 1, # Resolution reducer to minimize the time to obtain the nearest point
            'alpha': 0.1 # Filter gain in (0,1)
        }],
    )
    
    return LaunchDescription([
        frontal_free_pc_node,
        LogInfo(msg="\n\n---- frontal_free_pc_node_launch.py, frontal_free_pc node running ----\n\n"),
    ])
