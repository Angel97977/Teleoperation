# 🦾 Teleoperation and Peg-in-Hole with xArm (Physical Evidence)

Welcome to the **PhysicalEvidence** branch. This repository showcases the successful implementation of a **physical hardware** master-slave teleoperation system using UFactory xArm (Lite 6) robots.

## 🏆 Project Achievements
The main goal of this project was to achieve real-time teleoperation of a slave arm using a master arm to perform precision tasks, specifically the **Peg-in-Hole** insertion. 

**The result was successfully achieved!** The system faithfully replicates movements and features a critical safety protocol:
A **load cell (HX711)** processed by an **ESP32** microcontroller publishes real-time force data via **micro-ROS**. If the robot detects a collision force exceeding the safe limit (e.g., 9810 Newtons) during the physical Peg-in-Hole task, the system triggers an Emergency Stop (State 4), instantly locking the motors to protect the hardware.

---

## 🛠️ Prerequisites & Installation

### 1. Official xArm Packages
To run this project, you first need the official xArm ROS 2 packages. **These are not included in this repository.**
Navigate to your ROS 2 workspace `src` folder (e.g., `~/ros2_ws1/src`) and clone the official repository to get the robot descriptions and drivers:

```bash
git clone [https://github.com/xArm-Developer/xarm_ros2.git](https://github.com/xArm-Developer/xarm_ros2.git) --recursive -b humble


# When this first clone is already satisfied, the way to execute this on each command prompt:

## Terminal 1 Slave Robot Initialization 

cd ~/ros2_ws1
source ~/dev_ws/install/setup.bash
ros2 launch xarm_api lite6_driver.launch.py robot_ip:=192.168.1.179 hw_ns:=ufactory

#Terminal 2 Master Robot Initialization
cd ~/ros2_ws1
source ~/dev_ws/install/setup.bash
ros2 launch xarm_api lite6_driver.launch.py robot_ip:=192.168.1.154 hw_ns:=ufactory2

## Terminal 3 Main Controller (Communication between master & slave)

cd ~/ros2_ws1 && colcon build --packages-select imu_xarm_controller
source ~/dev_ws/install/setup.bash
source ~/ros2_ws/install/setup.bash
source install/setup.bash
ros2 launch imu_xarm_controller master_slave.launch.py


## Terminal 4 micro-ROS Agent (Optional / Sensor Connection)
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0


## Terminal 5 Data Monitor (Optional / Debugging)
ros2 topic echo /load_cell_weight

# Notice: on each command prompt use your current path when you clone the first repo (ROS2 humble && you can use this as well). But be aware of your version of ROS2.