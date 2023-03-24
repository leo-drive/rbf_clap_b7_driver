# Clap-B7 Test Drivers and Basic Usage
This repository include Clap B7 ROS1 and ROS2 test drivers.


### Wiring for USB Connection

Clap-B7 Wiring for USB Connection      |
:-------------------------:|
![ClapB7Wiring](Docs/clapb7_usb_wiring.png)


>**Input voltage must be between 3.3V and 5V!**

* `CLAP-TX1 --> USB-TTL RX`
* `CLAP-RX1 --> USB-TTL TX`
* `CLAP-GND --> USB-TTL GND`

* `CLAP-VCC --> VCC (3.3 - 5V DC)`
* `CLAP-GND --> GND`


```
After completing the wiring, you can run the driver by plugging the usb cable into your computer.
```
## Installation
#### Dependencies
* [Robot Operating System (ROS-ROS2)]
* Autoware Messages | [autoware_msgs](https://github.com/autowarefoundation/autoware_msgs) 

#### Building

1. Clone the repository (use a Release version) (Put repo into autoware messages built)
2. Build using the normal ROS2 colcon build system

## Usage
To run the default Ros2 node with the default configuration

```
ros2 launch rbf_clap_b7_driver rbf_clap_b7_driver.launch.py
```
## Config files

Parameters of the driver can be modified from `clap_b7_driver.param.yaml` 

### Parameters

Parameter | Type | Detailed Description | Default |
--- | --- | --- | -- |
serial_name | String | Serial device name of port communicates with ClapB7 |"/dev/ttyACM0"|
parse_type | String | Parsing type of serial reading | "BINARY"|
baud_rate | Integer | Baud rate of serial communication | 460800|
clap_data | String | Clap specific topic name that provides all raw data | "/clap_b7_data"|
clap_imu_data | String | Topic name that provides all raw imu data | "/clap_imu"|
clap_ins_data | String | Topic name that provides all raw gnss_ins data | "/clap_ins"|
imu_topic | String | ROS standard [sensor_msgs/IMU](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Imu.html) topic  | "/gnss/imu"|
nav_sat_fix_topic | String | ROS standard [sensor_msgs/NavSatFix](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/NavSatFix.html) | "/gnss/gps"|
autoware_orientation_topic | String | Autoware [autoware_sensing_msgs/GnssInsOrientationStamped](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_sensing_msgs/msg/GnssInsOrientationStamped.msg) | "/gnss/autoware_orientation"|
twist_topic | String | ROS standard [geometry_msgs/TwistWithCovarianceStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/TwistWithCovarianceStamped.html) | "/gnss/twist"|
time_system_selection | Integer | GNSS-INS Header/TimeStamp time system reference selection 0 -- Global GPS clock time 1 -- ROS RCLCPP clock time | 0 |
enu_ned_transform | String | GNSS data comes NED frame as default, "true" -- convert NED frame to ENU | "false"|
gnss_frame | String | Frame of `nav_sat_fix_topic` | "gnss"|
imu_frame | String | Frame of `imu_topic` | "gnss"|
autoware_orientation_frame | String | Frame of `autoware_orientation_topic` | "gnss"|
twist_frame | String | Frame of `twist_topic` | "gnss"|
debug | String | true -- Prints out required some values to test working of GNSS | "true"|

### NTRIP Settings

For using NTRIP, you have to uncomment `NTRIP Settings lines` and `NTRIP_client_start()` function line in driver.

After this step, you need to enter the relevant places NTRIP settings in `clap_b7_driver.param.yaml` . These settings are NTRIP ip, NTRIP username, NTRIP password, NTRIP mount point and NTRIP port.

Parameter | Type | Detailed Description |
--- | --- | --- |
ntrip_ip | String | IP number of NTRIP |
ntrip_user_name | String | Username | 
ntrip_password | Integer | NTRIP password| 
ntrip_mount_point | String | NTRIP mount point | 
ntrip_port | Integer | NTRIP port number | 
activate_ntrip | String | "true" -- if you want to connect RTK otherwise "false" |

