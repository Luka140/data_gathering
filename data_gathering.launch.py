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

    # =========================================================================================
    sample      = "TEST"   #---------------- Change this! -------------------------------------
    plate_thickness = 2. / 1000  # In meters 
    # =========================================================================================
    
    grit        = 120
    force_settings = [3,5,7]
    rpm_settings = [7000, 9000, 11000]
    contact_times = [10]
    
    
    # TODO Test for what wear threshold should be 
    # TODO that bastard timer is still getting fired twice for some reason 

    # Set to true to test all combinations of force rpm and time settings. Otherwise they are paired elementwise.
    all_setting_permutations = True 

    wear_threshold = 20e6    # Threshold of force * rpm * time after which the belt needs to be changed

    if all_setting_permutations:
        _settings_array = np.array(list(product(force_settings, rpm_settings, contact_times)))
        force_settings, rpm_settings, contact_times = [[float(val) for val in ar.flatten()] for ar in np.hsplit(_settings_array, 3)]

    # _desired_flowrate = 100 * (desired_rpm - 3400) / 7600
    
    data_collector = Node(
        package=pkg,
        executable="data_collector",
        parameters=[{
             'timeout_time':            30.,    # Duration before timeout of a single test
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
             'wear_tracking_path':      "src/data_gathering/data/belt_data/beltid_1_grit_120.csv",
            }
        ]
    )

    acf_node = Node(
        package='ferrobotics_acf',
        executable='acf.py',
        parameters=[
            {'ip': '169.254.200.17',
             'ramp_duration': 0.0,
             'frequency': 120,
             'payload': 1.6}
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
            'local_bbox_max': '0.2'                   
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
    if sample != 'TEST' and len([file for file in pathlib.Path(bag_directory).iterdir() if f'sample{sample}' in str(file)]) > 0:
        raise ValueError(f"Sample {sample} already exists in directory {bag_directory}\ndon't forget to change the sample number")
        
    
    ld.add_action(data_collector)
    ld.add_action(acf_node)
    ld.add_action(test_coordinator)
    ld.add_action(scanner_launch)
    ld.add_action(volume_calculator)
    return ld
