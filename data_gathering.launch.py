from launch import LaunchDescription
from launch_ros.actions import Node
import launch.actions
import datetime
import pathlib


def generate_launch_description():

    pkg = "data_gathering"
    ld = LaunchDescription()

    # ======================================================================================
    sample      = 0.5   #---------------- Change this! -------------------------------------
    # ======================================================================================
    force       = 7
    desired_rpm = 10000
    grit        = 60
    grind_time  = 10.
    
    # _desired_flowrate = 100 * (desired_rpm - 3400) / 7600
    
    data_collector = Node(
        package=pkg,
        executable="data_collector",
        parameters=[{
            # 'force_desired':           f'{force}',                 # Force of the ACF (N)
            # 'desired_flowrate_scaled': f'{_desired_flowrate}',     # % of flowrate available for RPM
            # 'max_contact_time':        f'{grind_time}',            # Duration to grind
             'timeout_time':            '30.',                      # Duration before timout
             'timer_period':            '0.050',                    # Period between force and RPM calls 
             'time_before_extend':      '3',                        # Duration between intial spin up of grinder and ACF extension
             'grinder_enabled':         False                       # Enable/Disable the grinder with True/False
            }
        ]
    )

    test_coordinator = Node(
        package=pkg,
        executable="test_coordinator",
        parameters=[
            {'force_settings':           [5,6,7],                 # Force of the ACF (N)
             'rpm_settings':             [7000, 8000, 9000],     # % of flowrate available for RPM
             'contact_time_settings':    [5],            # Duration to grind
             'grit':                     grit,
             'sample_id':                sample,
             'wear_threshold':           10e6,
             'wear_tracking_file':       "src/data_gathering/belt_data/beltid_1_grit_120.csv"
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
    
    bag_directory = 'ros_bags'
    # current_time = datetime.datetime.now()
    # timestamp = f'{str(current_time.date()).strip()}_{str(current_time.time()).strip().split(".")[0]}'
    
    # recorder = launch.actions.ExecuteProcess(
    #     cmd=['ros2', 'bag', 'record', 
    #          '-a',                                                  # All topics
    #          '-s', 'mcap',                                          # Set to MCAP storage format
    #          '-o', f'{bag_directory}/rosbag2_{timestamp}_sample{sample}__f{force}_rpm{desired_rpm}_grit{grit}_t{grind_time}'],            
    #     output='screen'
    #     )
    
    if desired_rpm < 3400:
        raise ValueError('The desired rpm must be above 3400')
    if len([file for file in pathlib.Path(bag_directory).iterdir() if f'sample{sample}' in str(file)]) > 0:
        raise ValueError(f"Sample {sample} already exists in directory {bag_directory}\ndon't forget to change the sample number")
        
    
    ld.add_action(data_collector)
    ld.add_action(acf_node)
    ld.add_action(test_coordinator)
    return ld