from launch import LaunchDescription
from launch_ros.actions import Node
import launch.actions
import datetime
import pathlib
from itertools import product
import numpy as np 

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg = "data_gathering"
    ld = LaunchDescription()

    ###################################################### TEST SETTINGS ########################################################

    # ---------------------------------------------- DO NOT FORGET TO CHANGE THESE ----------------------------------------------
    sample      = "wear_test_V2"   
    plate_thickness = 1.98 / 1000  # In meters 
    # ---------------------------------------------------------------------------------------------------------------------------

    # Main test settings 
    grit        = 120
    force_settings = [5] 
    rpm_settings = [10000]
    contact_times = [10] * 5 + [30] * 10
    
    # Set to true to test all combinations of force rpm and time settings. Otherwise they are paired elementwise.
    all_setting_permutations = True 
    
    # Prime the belt before starting the test.
    initially_prime_new_belt = False 

    # Belt wear tracking file path
    belt_wear_path = "src/data_gathering/data/belt_data/beltid_weartestV2_grit_120.csv"

    # Threshold of force * rpm * time after which the belt needs to be changed
    wear_threshold = 100e9    

    ##############################################################################################################################


    # Settings for priming a new belt. This is a single run that is performed if the belt is changed
    belt_prime_force    = 3
    belt_prime_rpm      = 9000
    belt_prime_time     = 5

    if all_setting_permutations:
        _settings_array = np.array(list(product(force_settings, rpm_settings, contact_times)))
        force_settings, rpm_settings, contact_times = [[float(val) for val in ar.flatten()] for ar in np.hsplit(_settings_array, 3)]

    # _desired_flowrate = 100 * (desired_rpm - 3400) / 7600
    
    data_collector = Node(
        package=pkg,
        executable="data_collector",
        parameters=[{
             'timeout_time':            float(max(contact_times) * 3),    # Duration before timeout of a single test
             'time_before_extend':      3.,     # Duration between initial spin up of grinder and ACF extension
             'grinder_enabled':         True,   # Enable/Disable the grinder with True/False
             'max_acf_extension':       35.5    # Extension of the acf before hitting its endstop in meters 
            }
        ]
    )

    test_coordinator = Node(
        package=pkg,
        executable="test_coordinator",
        parameters=[
            {'force_settings':          force_settings,    # Force of the ACF (N)     
             'rpm_settings':            rpm_settings,      # RPM of the grinder
             'contact_time_settings':   contact_times,     # Duration to grind (s)
             'grit':                    grit,              # Grit of the belt (only for logging purposes)
             'sample':                  sample,            # Sample number (only for logging purposes)
             'wear_threshold':          wear_threshold,    # Threshold of the belt wear metric before requiring a change
             'plate_thickness':         plate_thickness,   # The thickness of the tested plate (m)
             'wear_tracking_path':      belt_wear_path,
             "belt_prime_force":        belt_prime_force,
             "belt_prime_rpm":          belt_prime_rpm, 
             "belt_prime_time":         belt_prime_time,
             "initial_prime":           initially_prime_new_belt
            }
        ]
    )

    
    acf_node = Node(
        package='ferrobotics_acf',
        executable='acf.py',
        parameters=[
            {'ip': '169.254.200.17',
             'ramp_duration': 0.,
             'frequency':120,
             'payload': 1.6,
            #  'control_p': 0.9,
            #  'control_i': 9 / 1000,
            #  'control_d': 30  / 1000,
            #  'f_max': 20
            }
        ]
    )

        # Path to the Inner Launch File
    surface_scanner = os.path.join(
        get_package_share_directory('ur_trajectory_controller'),
        'launch',
        'ur_surface_measurement.launch.py' 
    )

    scanner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(surface_scanner),
        launch_arguments={
            'path_config': 'trajectory_test_plate_vertical.yaml',
            'autonomous_execution': 'false',  
            'loop_on_service': 'true',        
            'auto_loop': 'false',             
            'save_mesh': 'false',             
            'bbox_max': '0.0',
            'local_bbox_max': '0.17'                   
        }.items()
    )

    volume_calculator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('pcl_processing_ros2'), 'launch', 'scan_and_process_pcl.launch.py')
        )
    )
        
    if any([rpm < 3400 for rpm in rpm_settings]):
        raise ValueError('The desired rpm must be above 3400')
    bag_directory = 'src/data_gathering/data/test_data'
    if 'TEST' not in sample and len([file for file in pathlib.Path(bag_directory).iterdir() if f'sample{sample}' in str(file)]) > 0:
        raise ValueError(f"Sample {sample} already exists in directory {bag_directory}\ndon't forget to change the sample number")
        
    
    ld.add_action(data_collector)
    ld.add_action(acf_node)
    ld.add_action(test_coordinator)
    ld.add_action(scanner_launch)
    ld.add_action(volume_calculator)
    return ld
