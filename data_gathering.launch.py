from launch import LaunchDescription
from launch_ros.actions import Node
import launch.actions
import datetime


def generate_launch_description():

    pkg = "data_gathering"
    ld = LaunchDescription()


    data_collector = Node(
        package=pkg,
        executable="data_collector",
    )

    acf_node = Node(
        package='ferrobotics_acf',
        executable='acf.py',
        parameters=[
            {'ip': '169.254.200.17'}
        ]
    )
    
    bag_directory = 'ros_bags'
    current_time = datetime.datetime.now()
    recorder = launch.actions.ExecuteProcess(
        cmd=['ros2', 'bag', 'record', 
             '-a',                      # Topics
             '-s', 'mcap',               # Set to MCAP storage format
             '-o', f'{bag_directory}/rosbag2_{current_time}'],            
        output='screen'
        )

    ld.add_action(data_collector)
    ld.add_action(acf_node)
    ld.add_action(recorder)
    return ld