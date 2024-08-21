# SAMXL_GrinderRobot

## Overview
Automated grinder robot control code

## Installation

#### Dependencies
- ROS2 Humble
- pyads
- mcap data format

To install ROS2 workspace under BrightSky Project, follow the instruction from: https://gitlab.tudelft.nl/samxl/projects/brightsky/brightsky-ws

Otherwise, install ROS2 humble according to https://docs.ros.org/en/humble/Installation.html.

To install pyads from Python:
```bash
pip install pyads

sudo apt-get update
sudo apt install ros-humble-rosbag2-storage-mcap
```

#### Building
To build from source, clone the latest version from this repository into your workspace along with the following repositories

- ferrobotics_acf
- bag_converter
- stamped_std_msgs

```bash
mkdir -p $workspace/src
git clone git@github.com:Luka140/data_gathering.git
git clone git@github.com:Luka140/ferrobotics_acf.git
git clone git@github.com:Luka140/bag_converter.git
git clone git@github.com:Luka140/stamped_std_msgs.git
```
and compile all of the packages:
To build, after installing all the packages:
```bash
colcon build --symlink-install
```
And source the installation
```bash
. install/setup.bash
```

## Nodes
### data_collector
To launch grinding sequence when ACF and PLC are turned on, connected, and in RUN mode
```bash
ros2 launch data_gathering data_gathering.launch.py
```

#### Parameters
Grinder Settings
* **`force_desired`** (int)

	Define force applied by ACF.

* **`_desired_flowrate`** (int)

	Define the RPM of the sandbelt. The variable **`_desired_flowrate`** is scaled but RPM can be set by variable **`desired_rpm`** in data_gathering.launch.py

* **`grind_time`** (int)

	Define total grinding time, ramp up/down time of approximately 3 seconds are included.

### acf_node

Establish connection with ACF. Also define ramp duration to achieve desired force by parameter **`ramp_duration`**

#### Published Topics

* **`force`** ([acf/force])(float32)

	Record force value by ROS2

* **`RPM`** ([grinder/rpm])(int32)

	Record RPM values by ROS2

* **`time_sync`** ([acf/force])(float32)

	Record time, synced to the same time stamp, by ROS2.


#### Services
TBD


## Bugs & Feature Requests

Please report bugs and request features through paninananwatan@tudelft.nl.
