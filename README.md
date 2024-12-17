# Grinding model data gatherer -- Moving grinder
An overview of the interactions of all nodes in this environment is shown [below](#interaction-overview).
## Overview
Code to perform automated testing of material removal using a pneumatic grinder. 
It uses a laser line scanner on a UR16 to create a scan of the test plate. A grinder which is mounted on an ABB IRB 1200 is then engaged and makes a set number of grinding passes at a certain feed rate, force and RPM. 
A second scan is then performed to measure the amount of material removed. The test data is recorded using rosbags.

The recorded rosbags can be processed into .csv files for modelling using the [bag_converter](https://github.com/Luka140/bag_converter) package. 

**Note that other packages this package depends on should also be set to the 'moving_grinder' branch if available.**

## Installation and dependencies
Clone this repository. 
The following are the dependencies: 
- [`rws_motion_client`](https://github.com/Luka140/rws_motion_client): Interacts with the ABB robot, and coordinates its moves with the grinder and ACF.
- [`ferrobotics_acf`](https://github.com/Luka140/ferrobotics_acf/tree/humble): Controls the ACF
- [`stamped_std_msgs`](https://github.com/Luka140/stamped_std_msgs/tree/main): Stamped standard messages for data storage
- [`data_gatherin_msgs`](https://github.com/Luka140/data_gathering_msgs/tree/moving_grinder): Additional msgs
- [`ur_trajectory_controller`](https://github.com/Luka140/ur_trajectory_controller): Controls the UR16e
- [`scancontrol`](https://github.com/Luka140/scancontrol/tree/ros2-devel): ROS2 Driver for Scancontrol laser line scanners
- [`lls_processing`](https://github.com/Luka140/lls_processing): Compiles 3D pointcloud reconstructions based on TF transfers
- [`pcl_processing_ros2`](https://github.com/panin-anan/pcl_processing_ros2/tree/moving_grinder): Used to calculate volume loss between two scans
- [`Universal_Robots_ROS2_Driver`](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/humble): Driver for the UR16e
- [`abb_ros2`](https://github.com/PickNikRobotics/abb_ros2/tree/humble): Driver for the ABB robot
 
- `ros-humble-rosbag2-storage-mcap`: Enabled MCAP storage format for rosbags
- `open3d`: for pointcloud operations. Note that the used (and currently latest) version requires Numpy < 1.25. Used in `pcl_processing_ros2` and `lls_processing`.
- [`pyads`](https://github.com/stlehmann/pyads): A Python wrapper for TwinCAT ADS library. Used in `data_gathering`
- [`concave_hull`](https://github.com/panin-anan/concave_hull): A Python library to calculate concave hulls. Used in `pcl_processing_ros2`
- [`pyransac3d`](https://github.com/leomariga/pyRANSAC-3D): A python library for the RANSAC algorithm. Used in`pcl_processing_ros2`

```bash
git clone git@github.com:Luka140/rws_motion_client.git
git clone git@github.com:Luka140/data_gathering.git -b moving_grinder
git clone git@github.com:Luka140/ferrobotics_acf.git -b humble
git clone git@github.com:Luka140/stamped_std_msgs.git
git clone git@github.com:Luka140/data_gathering_msgs.git -b moving_grinder
git clone git@github.com:Luka140/ur_trajectory_controller.git
git clone git@github.com:Luka140/scancontrol.git -b ros2-devel
git clone git@github.com:Luka140/lls_processing.git
git clone git@github.com:panin-anan/pcl_processing_ros2.git -b moving_grinder
git clone git@github.com:PickNikRobotics/abb_ros2.git -b humble
sudo apt-get install ros-humble-ur
sudo apt-get install ros-humble-rosbag2-storage-mcap

pip install pyads==3.4.2
pip install open3d==0.18.0
pip install numpy==1.24.0
pip install pyransac3d==0.6.0
```

## Launch
To launch use 
```bash
ros2 launch data_gathering data_gathering.launch.py
```
This launch file is used to set all the test settings. 


## Nodes
### grinder_node
This node acts as an interface to a Beckhoff PLC using pyads, controlling the RPM of the grinder. 

Parameters:
- `plc_target_ams`: AMS ID of the PLC.
- `plc_target_ip`: IP address of the PLC.
- `rpm_control_var`: Name of the TWINCAT variable which controls the RPM.
- `grinder_on_var`: Name of the TWINCAT variable for turning the grinder on/off.
- `grinder_enabled`: Bool that enables or disables the grinder.
- `time_var`: Name of the TWINCAT variable for PLC timestamps.

Topics:
- `~/rpm` [Int32Stamped](https://github.com/Luka140/stamped_std_msgs/blob/main/msg/Int32Stamped.msg): Publishes the grinder's actual RPM. Solely for logging purposes.
- `~/requested_rpm` [Int32Stamped](https://github.com/Luka140/stamped_std_msgs/blob/main/msg/Int32Stamped.msg): Publishes the requested RPM. Solely for logging purposes.
- `~/timesync` [TimeSync](https://github.com/Luka140/stamped_std_msgs/blob/main/msg/TimeSync.msg): Publishes time synchronization messages between ROS and PLC.

Services:
- `~/enable_grinder` [StartGrinder](https://github.com/Luka140/data_gathering_msgs/blob/moving_grinder/srv/StartGrinder.srv): Sets a grinder RPM and enables it. **Note that the actual PLC variable controlling the RPM in the grinder is a scaled flowrate. The RPM is converted to this scaled flowrate with the `rpm_to_flowrate` method. This method contains the inverse formula for how the scaled flowrate is defined in the PLC. If this formula changes in the PLC, this method should be adjusted as well.**
- `~/disble_grinder` [StopGrinder](https://github.com/Luka140/data_gathering_msgs/blob/moving_grinder/srv/StopGrinder.srv): Disables the grinder.

### test_coordinator
A node that cycles through the tests specified in the launch file. It coordinates the grinds, scans, volume calculations, and data recording. 

Parameters:
- `force_settings`: list of force settings for the queued tests
- `rpm_settings`: list of RPM settings for the queued tests
- `feed_rate_settings`: list of TCP feed rate settings for the queued tests
- `pass_count_settings`: list of how many griding passes each queued test should perform
- `grit`: the grit of the sanding belt - for logging purposes (default: 120)
- `sample`: handle/name by which to identify the tests - for logging purposes
- `plate_thickness`: the thickness of the plate in mm. Sent in the request to another node and used to calculate the removed volume
- `belt_width`: The width of the grinder belt [m]
- `pass_length`: The length of the grinder pass [m]. Used to calculate total belt contact time
- `feed_rate_threshold`: The maximum accepted feed rate setting threshold. Used to ensure no unsafe feed rate is set accidentally (default: 10.)

- `belt_prime_force`: the force setting at which to prime a new belt (default: 5)
- `belt_prime_rpm`: the RPM setting at which to prime a new belt (default: 9000)
- `belt_prime_feedrate`: the time setting at which to prime a new belt (default: 10)
- `belt_prime_passes`: The number of passes during belt priming (default: 2)
- `initial_prime`: bool to indicate whether a prime run needs to be performed before the first queued test (default: False)
- `wear_threshold`: the threshold of the belt wear indicator after which the belt needs to be changed (default: 5e7)

- `data_path`: path to the data storage location
- `wear_tracking_path`: path to the storage location of the belt run history to calculate wear. In launch file, check that .csv exists at the path and the name format should be: beltid_IDNum_grit_GritSize
- `test_tracker_path`: path to file which tracks all tests that have been run
- `record_path`: path to rosbag storage
- `recorded_topics`: The topics which are recorded to the rosbag (default: ['-a']  <- all topics)

Topics:
- `user/continue_testing` [Empty]: Send Empty msg to this topic to continue testing when prompted
- `user/ignore_error` [Empty]: If the "/rws_motion_client/start_grind_move" service returns a failure, this node will hang until the user sends a message to this topic, to prevent the scanning setup from running in an unsafe situation. 
- `user/changed_belt` [Empty]: Send Empty msg to this topic to confirm you have changed the belt when prompted

- `~/test_failure` [String]: Publishes a message indicating that a test has failed and why for logging purposes.
- `~/belt_wear_history` [BeltWearHistory](https://github.com/Luka140/data_gathering_msgs/blob/moving_grinder/msg/BeltWearHistory.msg): Publishes the belt wear on the currently tracked belt for logging purposes
- `~/grind_area` [GrindArea](https://github.com/Luka140/data_gathering_msgs/blob/moving_grinder/msg/GrindArea.msg): Publishes the grinder contact area size for logging purposes
- `~/volume`[Float32Stamped](https://github.com/Luka140/stamped_std_msgs/blob/main/msg/Float32Stamped.msg): Publishes the removed volume in a belt-width-sized section in the middle of the test plate. For logging purposes
- `~/full_volume`[Float32Stamped](https://github.com/Luka140/stamped_std_msgs/blob/main/msg/Float32Stamped.msg): Publishes the removed volume over the full size of the test plate. For logging purposes
  
Clients:
- `execute_loop` [RequestPCL](https://github.com/Luka140/data_gathering_msgs/blob/main/srv/RequestPCL.srv): Requests a scan of the test object
- `calculate_volume_lost` [RequestPCLVolumeDiff](https://github.com/Luka140/data_gathering_msgs/blob/moving_grinder/srv/RequestPCLVolumeDiff.srv): Requests the comparison of two pointclouds and the calculation of lost volume.
- `/rws_motion_client/start_grind_move` [StartGrindTest](https://github.com/Luka140/data_gathering_msgs/blob/moving_grinder/srv/StartGrindTest.srv): Requests a grind test with the next queued set of grinder settings. 

# Interaction Overview
Normal arrows are topics and the double-sided arrows are services
![data_gathering_moving_grinder drawio](https://github.com/user-attachments/assets/3abc280e-4294-447f-9d55-bc641d076220)
