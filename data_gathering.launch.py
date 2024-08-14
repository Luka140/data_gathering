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
        parameters=[
            {'force_desired':           '10',    # Force of the ACF (N)
             'desired_flowrate_scaled': '40',    # % of flowrate available for RPM
             'max_contact_time':        '5.',    # Duration to grind
             'timeout_time':            '20.',   # Duration before timout
             'timer_period':            '0.050', # Period between force and RPM calls 
             'time_before_extend':      '3'      # Duration between intial spin up of grinder and ACF extension
            }
        ]
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
             '-a',                                                  # All topics
             '-s', 'mcap',                                          # Set to MCAP storage format
             '-o', f'{bag_directory}/rosbag2_{current_time}'],            
        output='screen'
        )

    ld.add_action(data_collector)
    ld.add_action(acf_node)
    ld.add_action(recorder)
    return ld