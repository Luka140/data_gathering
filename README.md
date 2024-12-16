# Grinding model data gatherer
This branch contains code to perform tests on a stationary grinder. For the version with the robot-arm mounted grinder, see the `moving_grinder` branch.
## Overview
Code to perform automated testing of material removal using a pneumatic grinder. 
It uses a laser line scanner on a UR16 to create a scan of the test plate. The grinder is then engaged and grinds away material for a set force, rpm, and duration. A second scan is then performed to measure the amount of material removed. The test data is recorded using rosbags.

The recorded rosbags can be processed into .csv files for modelling using the [bag_converter](https://github.com/Luka140/bag_converter) package. 

## Installation and dependencies
Clone this repository. 
The following are the dependencies: 
- [`ferrobotics_acf`](https://github.com/Luka140/ferrobotics_acf/tree/humble): Controls the ACF
- [`stamped_std_msgs`](https://github.com/Luka140/stamped_std_msgs/tree/main): Stamped standard messages for data storage
- [`data_gatherin_msgs`](https://github.com/Luka140/data_gathering_msgs): Additional msgs
- [`ur_trajectory_controller`](https://github.com/Luka140/ur_trajectory_controller): Controls the UR16e
- [`scancontrol`](https://github.com/Luka140/scancontrol/tree/ros2-devel): ROS2 Driver for Scancontrol laser line scanners
- [`lls_processing`](https://github.com/Luka140/lls_processing): Compiles 3D pointcloud reconstructions based on TF transfers
- [`pcl_processing_ros2`](https://github.com/panin-anan/pcl_processing_ros2/tree/main): Used to calculate volume loss between two scans
- [`Universal_Robots_ROS2_Driver`](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/humble): Driver for the UR16e
  
- `ros-humble-rosbag2-storage-mcap`: Enabled MCAP storage format for rosbags
- `open3d`: for pointcloud operations. Note that the used (and currently latest) version requires Numpy < 1.25. Used in `pcl_processing_ros2` and `lls_processing`.
- [`pyads`](https://github.com/stlehmann/pyads): A Python wrapper for TwinCAT ADS library. Used in `data_gathering`
- [`concave_hull`](https://github.com/panin-anan/concave_hull): A Python library to calculate concave hulls. Used in `pcl_processing_ros2`
- [`pyransac3d`](https://github.com/leomariga/pyRANSAC-3D): A python library for the RANSAC algorithm. Used in`pcl_processing_ros2`

```bash
git clone git@github.com:Luka140/data_gathering.git
git clone git@github.com:Luka140/ferrobotics_acf.git -b humble
git clone git@github.com:Luka140/stamped_std_msgs.git
git clone git@github.com:Luka140/data_gathering_msgs.git
git clone git@github.com:Luka140/ur_trajectory_controller.git
git clone git@github.com:Luka140/scancontrol.git -b ros2-devel
git clone git@github.com:Luka140/lls_processing.git
git clone git@github.com:panin-anan/pcl_processing_ros2.git
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
### data_collector
A node that connects to the PLC with grinder, and sends commands to the ACF node. It performs a grind for a requested force, RPM, and contact duration.
Parameters:
- `plc_target_ams`: AMS ID of the PLC.
- `plc_target_ip`: IP address of the PLC.
- `timeout_time`: Maximum time allowed before timeout.
- `time_before_extend`: Time delay before ACF extension to allow the grinder to spin up.
- `rpm_control_var`: Name of variable which controls the RPM through PLC.
- `grinder_on_var`: Name of variable for turning the grinder on/off.
- `grinder_enabled`: Bool that enables or disables the grinder.
- `time_var`: Name of variable for PLC timestamps.
- `max_acf_extension`: Maximum allowed ACF extension.

Topics:
- `/acf/force` [Float32Stamped](https://github.com/Luka140/stamped_std_msgs/blob/main/msg/Float32Stamped.msg): Publishes the force applied during grinding.
- `~/rpm` [Int32Stamped](https://github.com/Luka140/stamped_std_msgs/blob/main/msg/Int32Stamped.msg): Publishes the grinder's actual RPM. Solely for logging purposes.
- `~/requested_rpm` [Int32Stamped](https://github.com/Luka140/stamped_std_msgs/blob/main/msg/Int32Stamped.msg): Publishes the requested RPM. Solely for logging purposes.
- `~/timesync` [TimeSync](https://github.com/Luka140/stamped_std_msgs/blob/main/msg/TimeSync.msg): Publishes time synchronization messages between ROS and PLC.

 - `/acf/telem` [ACFTelemStamped](https://github.com/Luka140/ferrobotics_acf/blob/humble/msg/ACFTelemStamped.msg): Subscribes to ACF telemetry, handling force and position data.

Services:
- `~/execute_test` [TestRequest](https://github.com/Luka140/data_gathering_msgs/blob/main/srv/TestRequest.srv): Starts a test by setting force, RPM, and contact duration. It handles RPM control, ACF engagement, monitoring grinder performance, and managing shutdown sequences on test completion or failure.


### test_coordinator
A node that cycles through the tests specified in the launch file. It coordinates the grinds, scans, volume calculations, and data recording. 

Parameters:
- `force_settings`: list of force settings for the queued tests
- `rpm_settings`: list of RPM settings for the queued tests
- `contact_time_settings`: list of contact time settings for the queued tests
- `grit`: the grit of the sanding belt - for logging purposes
- `sample`: handle/name by which to identify the tests - for logging purposes
- `plate_thickness`: the thickness of the plate in mm. 

- `belt_prime_force`: the force setting at which to prime a new belt
- `belt_prime_rpm`: the RPM setting at which to prime a new belt
- `belt_prime_time`: the time setting at which to prime a new belt
- `initial_prime`: bool to indicate whether a prime run needs to be performed before the first queued test
- `wear_threshold`: the threshold of the belt wear indicator after which the belt needs to be changed

- `data_path`: path to the data storage location
- `wear_tracking_path`: path to the storage location of the belt run history to calculate wear
- `test_tracker_path`: path to file which tracks all tests that have been run
- `record_path`: path to rosbag storage


Topics:
- `user/continue_testing` [Empty]: Send Empty msg to this topic to continue testing when prompted
- `user/changed_belt` [Empty]: Send Empty msg to this topic to confirm you have changed the belt when prompted
  
- `~/test_failure` [String]: Publishes a message indicating that a test has failed and why for logging purposes.
- `~/belt_wear_history` [BeltWearHistory](https://github.com/Luka140/data_gathering_msgs/blob/main/msg/BeltWearHistory.msg): Publishes the belt wear on the currently tracked belt for logging purposes
- `~/grind_area` [GrindArea](https://github.com/Luka140/data_gathering_msgs/blob/main/msg/GrindArea.msg): Publishes the grinder contact area size for logging purposes
- `~/volume`[Float32Stamped](https://github.com/Luka140/stamped_std_msgs/blob/main/msg/Float32Stamped.msg): Publishes the removed volume for logging purposes
    
Clients:
- `execute_loop` [RequestPCL](https://github.com/Luka140/data_gathering_msgs/blob/main/srv/RequestPCL.srv): Requests a scan of the test object
- `calculate_volume_lost` [RequestPCLVolumeDiff](https://github.com/Luka140/data_gathering_msgs/blob/main/srv/RequestPCLVolumeDiff.srv): Requests the comparison of two pointclouds and the calculation of lost volume.
- `data_collector/execute_test` [TestRequest](https://github.com/Luka140/data_gathering_msgs/blob/main/srv/TestRequest.srv): Requests a test from data_collector.

# Overview
Normal arrows are topics, the double-sided arrows are services, and the dotted lines are only for data storage or monitoring purposes. 
![data_gathering_architecture drawio](https://github.com/user-attachments/assets/81f2189e-e6ac-4c56-98fc-8f3556a72aa4)
