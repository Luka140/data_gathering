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
    sample      = "moving_robot_feed_rate_investigation_2"   # TODO BEFORE ANY SERIEUS TEST - CHANGE WEAR BACK!!!
    plate_thickness = 2.0 / 1000  # In meters 
    # ---------------------------------------------------------------------------------------------------------------------------

    grit        = 120
    belt_width  = 25. / 1000    
    pass_length = 75. / 1000

    # Main test settings 
    force_settings = [7, 8, 9] 
    rpm_settings = [11000]
    feed_rate_settings = [10]
    pass_count_settings = [4]
    repeat_test_count = 1      # Repeat a grind x times, and only scan afterwards. Then lost vol = lost vol / x. Detects lower volume. Set to 1 to scan after every grind 
    
    # Set to true to test all combinations of force rpm and time settings. Otherwise they are paired elementwise.
    all_setting_permutations = True 
    # Skip the first few tests ...
    # start_idx = len(force_settings) * len(rpm_settings) * len(contact_times)  - 15
    start_idx = 0
    
    # Prime the belt before starting the test. Recommended for a new belt, or a new plate.
    # The grinder behaves differently inside a groove compared to grinding a flat surface. 
    initially_prime_new_belt = True    

    # Belt wear tracking file path  
    belt_wear_path = "src/data_gathering/data/belt_data/beltid_10_grit_120.csv"

    # This is approximately the maximum threshold up to which wear tests have been done. Higher values may still be fine 
    # but are not proven to be.
    wear_threshold = 10e7    # TODO CHANGE BACK!!!!!!!

    ##############################################################################################################################

    # local_bbox_max = 0.17   # Maximum distance the scanner. Any object further away is ignored
    local_bbox_max = 0.35   # Maximum distance the scanner. Any object further away is ignored

    # Settings for priming a new belt. This is a single run that is performed if the belt is changed
    # It is also recommended to start a groove when a plate is switched, as behaviour on a flat surface and inside a groove seems to differ
    belt_prime_force    = 7
    belt_prime_rpm      = 11000
    belt_prime_time     = 15

    if all_setting_permutations:
        _settings_array = np.array(list(product(force_settings, rpm_settings, feed_rate_settings, pass_count_settings)))
        force_settings, rpm_settings, feed_rate_settings, pass_count_settings = [[float(val) for val in ar.flatten()] for ar in np.hsplit(_settings_array, 4)]

    # _desired_flowrate = 100 * (desired_rpm - 3400) / 7600


    
    time_before_extend = 3.
    data_collector = Node(
        package=pkg,
        executable="data_collector",
        parameters=[{
             'timeout_time':            float(((pass_length*1000 * max(pass_count_settings))/min(feed_rate_settings) + time_before_extend) * 1.5),    # Duration before timeout of a single test
             'time_before_extend':      time_before_extend,     # Duration between initial spin up of grinder and ACF extension
             'grinder_enabled':         False,   # Enable/Disable the grinder with True/False
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
             'feed_rate_settings':      feed_rate_settings[start_idx:],     # grinder feed rate (mm/s)
             'pass_count_settings':     pass_count_settings[start_idx:],    # number of passes
             'grit':                    grit,               # Grit of the belt (only for logging purposes)
             'sample':                  sample,             # Sample number (only for logging purposes)
             'wear_threshold':          wear_threshold,     # Threshold of the belt wear metric before requiring a change
             'plate_thickness':         plate_thickness,    # The thickness of the tested plate (m)
             'belt_width':              belt_width,         # The width of the belt (m)
             'pass_length':             pass_length,        # feed length of each pass
             'wear_tracking_path':      belt_wear_path,             # Path to wear tracking file
             "belt_prime_force":        belt_prime_force,           # Force to use for priming the belt
             "belt_prime_rpm":          belt_prime_rpm,             # RPM to use for priming the belt
             "belt_prime_time":         belt_prime_time,            # Duration to prime the belt 
             "initial_prime":           initially_prime_new_belt,   # Prime the belt before any test is run. 
             "repeat_test_count":       repeat_test_count,          # Repeat every test x times before scanning to make the volume loss more detectable
             "recorded_topics": ['/data_collector/rpm',
                                 '/acf/force',
                                 '/rws_motion_client/grind_settings',
                                 '/acf/telem',
                                 '/data_collector/timesync',
                                 '/test_coordinator/volume',
                                 '/test_coordinator/belt_wear_history',
                                 '/test_coordinator/test_failure',
                                 '/test_coordinator/grind_area']
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
             'payload': 2.0,
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
            'path_config': 'trajectory_abb_grind.yaml',
            # 'path_config': 'trajectory_corner_grind.yaml',
            'autonomous_execution': 'false',  
            'loop_on_service': 'true',        
            'auto_loop': 'false',             
            'save_mesh': 'false',             
            'bbox_max': '0.0',
            'local_bbox_max': str(local_bbox_max)                   
        }.items()
    )

    volume_calculator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('pcl_processing_ros2'), 'launch', 'scan_and_process_pcl.launch.py')),
    )
        
    if any([rpm < 3400 for rpm in rpm_settings]):
        raise ValueError('The desired rpm must be above 3400')
    bag_directory = 'src/data_gathering/data/test_data'
    if 'TEST' not in sample and len([file for file in pathlib.Path(bag_directory).iterdir() if f'sample{sample}' in str(file)]) > 0:
        raise ValueError(f"Sample {sample} already exists in directory {bag_directory}\ndon't forget to change the sample number")
        
    
    ld.add_action(data_collector)
    ld.add_action(acf_node)
    ld.add_action(test_coordinator)
    #ld.add_action(scanner_launch)
    ld.add_action(volume_calculator)
    return ld
