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
    sample      = "..."   
    plate_thickness = 2.00 / 1000  # In meters 
    # ---------------------------------------------------------------------------------------------------------------------------

    # Main test settings 
    grit        = 120
    force_settings = [3, 5, 7, 9] 
    rpm_settings = [8000, 9500, 11000]
    contact_times = [5, 7.5, 12.5, 17.5] 
    
    # Set to true to test all combinations of force rpm and time settings. Otherwise they are paired elementwise.
    all_setting_permutations = True 
    # Skip the first few tests ...
    start_idx = 0
    
    # Prime the belt before starting the test. Recommended for a new belt, or a new plate.
    # The grinder behaves differently inside a groove compared to grinding a flat surface. 
    initially_prime_new_belt = False

    # Belt wear tracking file path  # TODO CHECK CURRENT INSTALLED BELT
    belt_wear_path = "src/data_gathering/data/belt_data/....csv"

    # This is approximately the maximum threshold up to which wear tests have been done. Higher values may still be fine 
    # but are not proven to be.
    wear_threshold = 5e7    

    ##############################################################################################################################

    local_bbox_max = 0.17   # Maximum distance the scanner. Any object further away is ignored

    # Settings for priming a new belt. This is a single run that is performed if the belt is changed
    # It is also recommended to start a groove when a plate is switched, as behaviour on a flat surface and inside a groove seems to differ
    belt_prime_force    = 5
    belt_prime_rpm      = 10000
    belt_prime_time     = 15

    if all_setting_permutations:
        _settings_array = np.array(list(product(force_settings, rpm_settings, contact_times)))
        force_settings, rpm_settings, contact_times = [[float(val) for val in ar.flatten()] for ar in np.hsplit(_settings_array, 3)]

    # _desired_flowrate = 100 * (desired_rpm - 3400) / 7600
    
    time_before_extend = 3.
    data_collector = Node(
        package=pkg,
        executable="data_collector",
        parameters=[{
             'timeout_time':            float((max(contact_times) + time_before_extend) * 1.5),    # Duration before timeout of a single test
             'time_before_extend':      time_before_extend,     # Duration between initial spin up of grinder and ACF extension
             'grinder_enabled':         True,   # Enable/Disable the grinder with True/False
             'max_acf_extension':       35.5    # Extension of the acf before hitting its endstop in mm 
            }
        ]
    )

    test_coordinator = Node(
        package=pkg,
        executable="test_coordinator",
        parameters=[
            {'force_settings':          force_settings[start_idx:],    # Force of the ACF (N)     
             'rpm_settings':            rpm_settings[start_idx:],      # RPM of the grinder
             'contact_time_settings':   contact_times[start_idx:],     # Duration to grind (s)
             'grit':                    grit,              # Grit of the belt (only for logging purposes)
             'sample':                  sample,            # Sample number (only for logging purposes)
             'wear_threshold':          wear_threshold,    # Threshold of the belt wear metric before requiring a change
             'plate_thickness':         plate_thickness,   # The thickness of the tested plate (m)
             'wear_tracking_path':      belt_wear_path,             # Path to wear tracking file
             "belt_prime_force":        belt_prime_force,           # Force to use for priming the belt
             "belt_prime_rpm":          belt_prime_rpm,             # RPM to use for priming the belt
             "belt_prime_time":         belt_prime_time,            # Duration to prime the belt 
             "initial_prime":           initially_prime_new_belt    # Prime the belt before any test is run. 
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
            'local_bbox_max': str(local_bbox_max)'                   
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
