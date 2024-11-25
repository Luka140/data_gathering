# Grinding model data gatherer

## Overview
Code to perform automated testing of material removal using a pneumatic grinder. 
It uses a laser line scanner on a UR16 to create a scan of the test plate. The grinder is then engaged and grinds away material for a set force, rpm, and duration. A second scan is then performed to measure the amount of material removed. The test data is recorded using rosbags.

The recorded rosbags can be processed into .csv files for modelling using the [bag_converter](https://github.com/Luka140/bag_converter) package. 

## Installation

#### Dependencies (INCOMPLETE)
- ROS2 Humble
- Universal Robots ROS2 Driver
- pyads
- open3d



```bash
pip install pyads
pip install open3d==0.18.0
pip install numpy==1.24.0

sudo apt-get update
sudo apt install ros-humble-rosbag2-storage-mcap
```

#### Building
To build from source, clone the latest version from this repository into your workspace along with the following repositories

- `ferrobotics_acf`: Controls the ACF
- `stamped_std_msgs`: Stamped standard messages for data storage
- `data_gatherin_msgs`: Additional msgs
- `ur_trajectory_controller`: Controls the UR16
- `scancontrol`: ROS Driver for Scancontrol laser line scanners
- `lls_processing`: Compiles 3D pointcloud reconstructions based on TF transfers
- `pcl_processing_ros2`: Used to calculate volume loss between two scans
- `Universal_Robots_ROS2_Driver`: Driver for the UR16

```bash
git clone git@github.com:Luka140/data_gathering.git
git clone git@github.com:Luka140/ferrobotics_acf.git -b humble
git clone git@github.com:Luka140/stamped_std_msgs.git
git clone git@github.com:Luka140/data_gathering_msgs.git
git clone git@github.com:Luka140/ur_trajectory_controller.git
git clone git@github.com:Luka140/lls_processing.git
git clone git@github.com:Luka140/scancontrol.git -b ros2-devel
git clone git@github.com:panin-ananwa/pcl_processing_ros2.git
sudo apt-get install ros-humble-ur
```
and compile all of the packages:
To build, after cloning all the packages:
```bash
colcon build --symlink-install
install/setup.bash
```


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
- `/grinder/rpm` [Int32Stamped](https://github.com/Luka140/stamped_std_msgs/blob/main/msg/Int32Stamped.msg): Publishes the grinder's actual RPM. Solely for logging purposes.
- `/grinder/requested_rpm` [Int32Stamped](https://github.com/Luka140/stamped_std_msgs/blob/main/msg/Int32Stamped.msg): Publishes the requested RPM. Solely for logging purposes.
- `/timesync` [TimeSync](https://github.com/Luka140/stamped_std_msgs/blob/main/msg/TimeSync.msg): Publishes time synchronization messages between ROS and PLC.

 - `/acf/telem` [ACFTelemStamped](https://github.com/Luka140/ferrobotics_acf/blob/humble/msg/ACFTelemStamped.msg): Subscribes to ACF telemetry, handling force and position data.

Services:
- `execute_test` [TestRequest](https://github.com/Luka140/data_gathering_msgs/blob/main/srv/TestRequest.srv): Starts a test by setting force, RPM, and contact duration. It handles RPM control, ACF engagement, monitoring grinder performance, and managing shutdown sequences on test completion or failure.


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
- `/stop_testing` [Empty]: Send Empty msg to this topic to stop testing when prompted (Currently does not do anything...)
- `/continue_testing` [Empty]: Send Empty msg to this topic to continue testing when prompted
- `/changed_belt` [Empty]: Send Empty msg to this topic to confirm you have changed the belt when prompted

- `/test_failure` [String]: Publishes a message indicating that a test has failed and why.
- `/belt_wear_history` [BeltWearHistory](https://github.com/Luka140/data_gathering_msgs/blob/main/msg/BeltWearHistory.msg): Publishes the belt wear on the currently tracked belt for logging purposes
  
Clients:
- `execute_loop` [RequestPCL](https://github.com/Luka140/data_gathering_msgs/blob/main/srv/RequestPCL.srv): Requests a scan of the test object
- `calculate_volume_lost` [RequestPCLVolumeDiff](https://github.com/Luka140/data_gathering_msgs/blob/main/srv/RequestPCLVolumeDiff.srv): Requests the comparison of two pointclouds and the calculation of lost volume.
- `execute_test` [TestRequest](https://github.com/Luka140/data_gathering_msgs/blob/main/srv/TestRequest.srv): Requests a test from data_collector.

# Overview
Normal arrows are topics, the double-sided arrows are services, and the dotted lines are only for data storage or monitoring purposes. 
![data_gathering_architecture drawio](https://github.com/user-attachments/assets/81f2189e-e6ac-4c56-98fc-8f3556a72aa4)
