from launch import LaunchDescription
from launch_ros.actions import Node
import launch.actions
import datetime
import pathlib
from itertools import product
import numpy as np 


def generate_launch_description():

    pkg = "data_gathering"
    ld = LaunchDescription()

    # ======================================================================================
    sample      = 0.5   #---------------- Change this! -------------------------------------
    # ======================================================================================
    grit        = 60
    force_settings = [5,6,7]
    rpm_settings = [7000, 8000, 9000]
    contact_times = [5]

    wear_threshold = 1e6

    _settings_array = np.array(list(product(force_settings, rpm_settings, contact_times)))
    _force_list, _rpm_list, _contact_list = [[float(val) for val in ar.flatten()] for ar in np.hsplit(_settings_array, 3)]

    # _desired_flowrate = 100 * (desired_rpm - 3400) / 7600
    
    data_collector = Node(
        package=pkg,
        executable="data_collector",
        parameters=[{
             'timeout_time':            '30.',                      # Duration before timout of a single test
             'timer_period':            '0.01',                     # Period between force and RPM calls 
             'time_before_extend':      '3',                        # Duration between intial spin up of grinder and ACF extension
             'grinder_enabled':         False                       # Enable/Disable the grinder with True/False
            }
        ]
    )

    test_coordinator = Node(
        package=pkg,
        executable="test_coordinator",
        parameters=[
            {'force_settings':           _force_list,          # Force of the ACF (N)     
             'rpm_settings':             _rpm_list,            # RPM of the grinder
             'contact_time_settings':    _contact_list,        # Duration to grind (s)
             'grit':                     grit,
             'sample_id':                sample,
             'wear_threshold':           wear_threshold,
             'wear_tracking_path':       "src/data_gathering/data/belt_data/beltid_2_grit_120.csv"
            }
        ]
    )

    acf_node = Node(
        package='ferrobotics_acf',
        executable='acf.py',
        parameters=[
            {'ip': '169.254.200.17',
             'ramp_duration': 0.0}
        ]
    )
        
    if any([rpm < 3400 for rpm in rpm_settings]):
        raise ValueError('The desired rpm must be above 3400')
    bag_directory = 'ros_bags'
    if len([file for file in pathlib.Path(bag_directory).iterdir() if f'sample{sample}' in str(file)]) > 0:
        raise ValueError(f"Sample {sample} already exists in directory {bag_directory}\ndon't forget to change the sample number")
        
    
    ld.add_action(data_collector)
    ld.add_action(acf_node)
    ld.add_action(test_coordinator)
    return ld