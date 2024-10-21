# SAMXL_GrinderRobot

## Overview
Code to perform automated testing of material removal using a pneumatic grinder. 
It uses a laser line scanner on a UR16 to create a scan of the test plate. The grinder is then engaged and grinds away material for a set force, rpm, and duration. A second scan is then performed to measure the amount of material removed. The test data is recorded using rosbags.

## Installation

#### Dependencies
- ROS2 Humble
- pyads
- ...................................... so many 


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

```bash
git clone git@github.com:Luka140/data_gathering.git
git clone git@github.com:Luka140/ferrobotics_acf.git -b humble
git clone git@github.com:Luka140/stamped_std_msgs.git
git clone git@github.com:Luka140/data_gathering_msgs.git
git clone git@github.com:Luka140/ur_trajectory_controller.git
git clone git@github.com:Luka140/lls_processing.git
git clone git@github.com:Luka140/scancontrol.git -b ros2-devel
git clone git@github.com:panin-ananwa/pcl_processing_ros2.git

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
- plc_target_ams: AMS ID of the PLC.
- plc_target_ip: IP address of the PLC.
- timeout_time: Maximum time allowed before timeout.
- time_before_extend: Time delay before ACF extension.
- rpm_control_var: Controls the RPM through PLC.
- grinder_on_var: Boolean for turning the grinder on/off.
- grinder_enabled: Enables or disables the grinder.
- time_var: Variable for PLC timestamps.
- max_acf_extension: Maximum allowed ACF extension.

Topics:
- /acf/force [Float32Stamped](https://github.com/Luka140/stamped_std_msgs/blob/main/msg/Float32Stamped.msg): Publishes the force applied during grinding.
- /grinder/rpm [Int32Stamped](https://github.com/Luka140/stamped_std_msgs/blob/main/msg/Int32Stamped.msg): Publishes the grinder's actual RPM. Solely for logging purposes.
- /grinder/requested_rpm [Int32Stamped](https://github.com/Luka140/stamped_std_msgs/blob/main/msg/Int32Stamped.msg): Publishes the requested RPM. Solely for logging purposes.
- /timesync [TimeSync](https://github.com/Luka140/stamped_std_msgs/blob/main/msg/TimeSync.msg): Publishes time synchronization messages between ROS and PLC.

 - /acf/telem [ACFTelemStamped](https://github.com/Luka140/ferrobotics_acf/blob/humble/msg/ACFTelemStamped.msg): Subscribes to ACF telemetry, handling force and position data.

Services:
- execute_test [TestRequest](https://github.com/Luka140/data_gathering_msgs/blob/main/srv/TestRequest.srv): Starts a test by setting force, RPM, and contact duration. It handles RPM control, ACF engagement, monitoring grinder performance, and managing shutdown sequences on test completion or failure.


### test_coordinator
A node that cycles through the tests specified in the launch file. It coordinates the grinds, scans, volume calculations, and data recording. 

Parameters:
- test_duration: Time (in seconds) that the material test will run.
- wear_threshold: Maximum allowable wear percentage before stopping the test.
- bag_file_directory: Directory for storing recorded ROS bag files.
- initial_belt_condition: Initial state of the belt (for example, 100%).
- test_name_prefix: Prefix for naming the test files or recordings.
- scan_service_timeout: Timeout duration for the scan service calls.
- volume_calculation_service_timeout: Timeout for volume calculation service calls.

Topics:
- ...
- ...
- ...
  
Services:
- /scan_belt: Service client to trigger a scan of the material being tested.
- /calculate_volume_loss: Service client to compute volume loss during the test.
- /prime_belt: Service client for priming the belt.
- /stop_test: Handles external requests to stop the current test.
- /save_rosbag: Triggers the saving of test data in a ROS bag file.
- /change_belt: Handles the process when the belt must be replaced.
