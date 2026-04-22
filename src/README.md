# Unitree GO2 Robot ROS 2

<p align="center">
<img width="1280" height="420" src="https://github.com/IntelligentRoboticsLabs/go2_robot/assets/44479765/da616d77-cf4d-4acf-af2f-adc99f4f72d7)" alt='Go2 point cloud'>
</p>

[![License](https://img.shields.io/badge/license-BSD--3-yellow.svg)](https://opensource.org/licenses/BSD-3-Clause)
![distro](https://img.shields.io/badge/Ubuntu%2022-Jammy%20Jellyfish-green)
![distro](https://img.shields.io/badge/ROS2-Humble-blue)
[![humble](https://github.com/IntelligentRoboticsLabs/go2_robot/actions/workflows/humble.yaml/badge.svg)](https://github.com/IntelligentRoboticsLabs/go2_robot/actions/workflows/humble.yaml)
[![humble-devel](https://github.com/IntelligentRoboticsLabs/go2_robot/actions/workflows/humble_devel.yaml/badge.svg)](https://github.com/IntelligentRoboticsLabs/go2_robot/actions/workflows/humble_devel.yaml)

In this package is our integration for the Unitee Go2 robot.

## Checklist

- [x] robot description
- [x] odom
- [x] pointcloud
- [x] joint_states
- [x] Visualization in rviz
- [x] cmd_vel
- [x] go2_interfaces
- [x] Change modes
- [x] Change configuration for robot
- [ ] SLAM (working in progress)
- [ ] Nav2 (working in progress)
- [ ] Ros2cli
- [ ] Hardware interface
- [ ] Gazebo simulation

## Installation
You need to have previously installed ROS2. Please follow this [guide](https://docs.ros.org/en/humble/Installation.html) if you don't have it.

```bash
source /opt/ros/humble/setup.bash
```

Create workspace and clone the repository

```bash
mkdir ~/go2_ws/src
cd ~/go2_ws/src
git clone https://github.com/IntelligentRoboticsLabs/go2_robot.git -b humble
```

Install dependencies and build workspace
```bash
cd ~/go2_ws
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install 
```

Setup the workspace
```bash
source ~/ros2_ws/install/setup.bash
```

## Sensor installation
If you have purchased a hesai lidar 3d, or a realsense d435i, follow the following steps inside the robot.

### Hesai Lidar

```bash
sudo apt-get install libboost-all-dev
sudo apt-get install -y libyaml-cpp-dev
git clone --recurse-submodules https://github.com/HesaiTechnology/HesaiLidar_ROS_2.0.git
cd ..
source /opt/ros/$ROS_DISTRO/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Set the lidar IP to `config/config.yaml`
```yaml
lidar:
- driver:
    udp_port: 2368                                       #UDP port of lidar
    ptc_port: 9347                                       #PTC port of lidar
    device_ip_address: <Device IP>                       #IP address of lidar
    pcap_path: "<Your PCAP file path>"                   #The path of pcap file (set during offline playback)
    correction_file_path: "<Your correction file path>"  #LiDAR angle file, required for offline playback of pcap/packet rosbag
    firetimes_path: "<Your firetime file path>"          #The path of firetimes file
    source_type: 2                                       #The type of data source, 1: real-time lidar connection, 2: pcap, 3: packet rosbag
    pcap_play_synchronization: true                      #Pcap play rate synchronize with the host time
    x: 0                                                 #Calibration parameter
    y: 0                                                 #Calibration parameter
    z: 0                                                 #Calibration parameter
    roll: 0                                              #Calibration parameter
    pitch: 0                                             #Calibration parameter
    yaw: 0                                               #Calibration parameter
ros:
    ros_frame_id: hesai_lidar                            #Frame id of packet message and point cloud message
    ros_recv_packet_topic: /lidar_packets                #Topic used to receive lidar packets from ROS
    ros_send_packet_topic: /lidar_packets                #Topic used to send lidar packets through ROS
    ros_send_point_cloud_topic: /lidar_points            #Topic used to send point cloud through ROS
    send_packet_ros: true                                #true: Send packets through ROS 
    send_point_cloud_ros: true                           #true: Send point cloud through ROS 
```

### Realsense d435i

```bash
sudo apt install ros-humble-realsense2-camera
```

## Tutorial: Running Go2 on ROS2 Foxy

This tutorial guides you through bringing up the Unitree Go2 robot and enabling Nav2 navigation on Ubuntu 20.04.

### 1. Build the Workspace
Ensure all dependencies are installed and build the workspace:
```bash
cd ~/argo_go2_ws
source /opt/ros/foxy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 2. Robot Bringup
The bringup command initializes the hardware communication and the robot's state.

**Command:**
```bash
ros2 launch go2_bringup go2.launch.py lidar:=True rviz:=True
```

**What this runs:**
*   **`go2_driver`**: Communicates with the Unitree Go2 via their low-level API. It publishes Odometry (`/odom`) and Joint States (`/joint_states`).
*   **`robot_state_publisher`**: Uses the URDF from `go2_description` to publish the robot's TF tree.
*   **`hesai_ros_driver`**: Connects to the Hesai 3D Lidar and publishes the point cloud (`/lidar_points`).
*   **`rviz2`**: Opens a pre-configured visualization window.

**Verification:**
*   Run `ros2 topic list`. You should see `/odom`, `/lidar_points`, and `/joint_states`.
*   In Rviz, you should see the 3D model of the Go2 and the lidar point cloud.
*   Run `ros2 run tf2_tools view_frames.py` (if available) to verify the TF tree from `map` -> `odom` -> `base_link`.

### 3. Navigation Stack
Once the robot is localized, you can send goals for autonomous movement.

**Command:**
```bash
ros2 launch go2_nav navigation.launch.foxy.py
```

**What this runs:**
*   **AMCL**: Localization system that uses lidar data to find the robot's position on a map.
*   **Planner Server**: Computes a path from the current position to the goal.
*   **Controller Server (DWB)**: Follows the path while avoiding obstacles detected by the lidar.
*   **Behavior Tree (BT) Navigator**: Orchestrates the navigation logic (retries, recoveries).

**Verification:**
*   In Rviz, use the **"2D Pose Estimate"** tool to set the robot's initial position on the map.
*   Use the **"2D Goal Pose"** tool to set a destination. The robot should plan a path (visualized as a line) and start moving.
*   Check `/cmd_vel` to see the velocity commands being sent to the robot.

### 4. Useful Commands
*   **Change to Stand Up mode:**
    ```bash
    ros2 service call /mode go2_interfaces/srv/Mode "{mode: 'stand_up'}"
    ```
*   **Check Lidar Health:**
    ```bash
    ros2 topic echo /lidar_points --count 1
    ```

## Acknowledgment
Thanks to [unitree](https://github.com/unitreerobotics/unitree_ros2) for providing the support and communication interfaces with the robot.

## About
This is a customized version of the Go2 ROS 2 integration, tailored for ROS2 Foxy on Ubuntu 20.04.

Maintainer:
* giovanna

## License
This project is licensed under the BSD 3-clause License.
