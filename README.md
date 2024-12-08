# <div align="center">Need title</div>
![image](https://github.com/WinstonHChou/winter-2024-final-project-team-7/assets/68310078/0ba1c6cb-c9e0-4cf7-905a-f5f16e6bb2ca)
### <div align="center"> MAE 148 Final Project </div>
#### <div align="center"> Team 11 Fall 2024 </div>

<div align="center">
    <img src="images\ucsdyellow-car.jpg" height="300"> <img src="images\ucsdcart.png" height="300"><br>

</div>

## Table of Contents
  <ol>
    <li><a href="#team-members">Team Members</a></li>
    <li><a href="#abstract">Abstract</a></li>
    <li><a href="#what-we-promised">What We Promised</a></li>
    <li><a href="#accomplishments">Accomplishments</a></li>
    <li><a href="#challenges">Challenges</a></li>
    <li><a href="#final-project-videos">Final Project Videos</a></li>
    <li><a href="#software">Software</a></li>
        <ul>
            <li><a href="#slam-simultaneous-localization-and-mapping">SLAM (Simultaneous Localization and Mapping)</a></li>
            <li><a href="#obstacle-avoidance">Obstacle Avoidance</a></li>
        </ul>
    <li><a href="#hardware">Hardware</a></li>
    <li><a href="#gantt-chart">Gantt Chart</a></li>
    <li><a href="#course-deliverables">Course Deliverables</a></li>
    <li><a href="#project-reproduction">Project Reproduction</a></li>
    <li><a href="#acknowledgements">Acknowledgements</a></li>
    <li><a href="#contacts">Contacts</a></li>
  </ol>

<hr>

## Team Members
- Liem Le - MAE-Ctrls & Robotics (MC34) - Class of 2025
- Jusung Park- MAE (MC81) - Class of 2025

<hr>
## Abstract
* NEED IT EDIT

<hr>

## What We Promised
### Must Have
* NEED TO EDIT

### Nice to Have
* NEED to EDit
* NEED to EDIT
<hr>

## Accomplishments
* ODOM data achieved
  * Successfully implemented Fast LIO for real-time lidar odometry and mapping which enables the robot to accurately locate itself while generating a 3D map of the environment.
* Laserscan 2D visualization achieved
  * Configured visualization tools such as Rviz2 for odometry data and 3D point cloud representation, including tuning parameters for LaserScan and PointCloud2 to enhance data clarity.
<hr>

## Challenges
* Nav2 Stack is a complex but useful system for developing an autonomous robot.
* Futher Actions:
  * PointCloud Dynamic Obstacle Detection:  
    - Develop an algorithm to mark down position of obstacle group, and add them to Nav2 obstacle layer
  * Nav2 Path Planning & ROS 2 Control:  
    - Path Planning Server Development, and communication to ROS 2 Control System
<hr>

## Software

### Overall Architecture
The project was successfully completed using the **Slam-Toolbox** and **ROS2 Navigation 2 Stack**, with a significant adaptation to the [djnighti/ucsd_robocar container](https://hub.docker.com/r/djnighti/ucsd_robocar). The adaptation allowed for seamless integration and deployment of the required components, facilitating efficient development and implementation of the robotic system.

### SLAM (Simultaneous Localization and Mapping)
- The **Slam Toolbox** proved indispensable in our project, enabling us to integrate the LD19 Lidar – firmware-compatible with the LD06 model – into the ROS2 framework. This integration allowed us to implement SLAM, empowering our robot to autonomously map its environment while concurrently determining its precise location within it. Additionally, we enhanced this capability by incorporating nav2 amcl localization, further refining the accuracy and dependability of our robot's localization system. By combining these technologies, our robot could navigate confidently, accurately mapping its surroundings and intelligently localizing itself within dynamic environments.<br>

- The **Online Async Node** from the Slam Toolbox is a crucial component that significantly contributes to the creation of the map_frame in the project. This node operates asynchronously, meaning it can handle data processing tasks independently of other system operations, thereby ensuring efficient utilization of resources and enabling real-time performance. The map_frame is a fundamental concept in SLAM, representing the coordinate frame that defines the global reference frame for the environment map being generated. The asynchronous online node processes Lidar data, and fuses this information together to construct a coherent and accurate representation of the surrounding environment.<br>

- The **VESC Odom Node** plays a pivotal role in supplying vital odometry frame data within the robotics system. This node is responsible for gathering information from the VESC (Vedder Electronic Speed Controller), and retrieves essential data related to the robot's motion, such as wheel velocities and motor commands. The odometry frame, often referred to as the "odom_frame," is a critical component in localization and navigation tasks. It represents the robot's estimated position and orientation based on its motion over time. This information is crucial for accurately tracking the robot's trajectory and determining its current pose within the environment. By utilizing the data provided by the VESC Odom Node, the system can update the odometry frame in real-time, reflecting the robot's movements and changes in its position. This dynamic updating ensures that the odometry frame remains synchronized with the robot's actual motion, providing an accurate representation of its trajectory.

<br>
<hr>

## Hardware 

* __3D Printing:__ Camera Case & Stand, Jetson Nano Case, Jetson Nano Case, Base Mounts
* __Laser Cut:__ Base with 3mm holes to mount electronics and other components.

__Parts List__

* Traxxas Chassis with steering servo and sensored brushless DC motor
* Jetson Nano
* LIVOX MID 360
* Livox three-wire aviation connector
* Aviation connector power network port cable
* 64 GB Micro SD Card
* Adapter for micro SD card
* Wifi Antenna
* Logitech Controller (F710)
* OAK-D Lite Camera 
* SparkFun OpenLog Artemis (IMU)
* VESC
* XeRUn 3660 G2 Sensored Motor
* Anti-Spark Switch with Power Switch
* DC-DC Converter
* 3 cell LIPO Battery
* Battery Voltage Checker
* DC Barrel to XT30 Connector
* XT60, XT30, MR60 connectors

*For Testing:*

*Car Stand
*5V, 4A power supply for Jetson Nano
*USB-C to USB-A cable
*Micro USB to USB cable


### __Mechanical Designs__

__Base Plate with 3mm Holes__

<img src="images\baseplate.jpg" height="350">

__Adjustable Camera Stand__

<img src="images\cameram1.png" height="160"> <img src="images\cameram2.png" height="160"><br>

__GPS Mount/Stand__

<img src="images\gpsstand.jpg" height="300">

__Circuit Diagram__

<img src="images\circuitdiagram.png" height="300">

<hr>

## Gantt Chart
<div align="center">
    <img src="images\ganttchart.png" height="500">
</div>
<hr>

## Project Reproduction



<hr>


## Acknowledgements
Special thanks to Professor Jack Silberman, TA Winston Chou, and Ta Alexander   for delivering the course!  
Thanks to Raymond on Triton AI giving suggestions on our project!  
Thanks to Nikita on Triton AI providing support on razorIMU_9dof repo for IMU usage!

**Programs Reference:**
* [UCSD Robocar Framework](https://gitlab.com/ucsd_robocar2)
* [Slam Toolbox](https://github.com/SteveMacenski/slam_toolbox.git)
* [DepthAI_ROS_Driver](https://github.com/luxonis/depthai-ros)
* [razorIMU_9dof](https://github.com/NikitaB04/razorIMU_9dof)
* [Foxglove Studio](https://app.foxglove.dev/)


README.md Format, reference to [spring-2023-final-project-team-5](https://github.com/UCSD-ECEMAE-148/spring-2023-final-project-team-5)

<hr>

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

## Contacts

* Liem Le - ltle@ucsd.edu
* Jusung Park - jup018@ucsd.edu
* 















ssh -X jetson@ucsd-yellow.local

Pulling docker images(files)
- `cd projects/ `
-  `ll `
-  `cd robocar// `
-  `docker ps -a `
-  `docker images `
-  `cat docker.sh `

To enter container:
- `docker start team11 `
-  `docker exec -it team11 bash `
To activate ros2:
-  `source /opt/ros/foxy/setup.bash `
-  `ros2 <command lines>  `

# All the code has to be in the ros2_ws directory.

Git.clone https://github.com/hku-mars/FAST_LIO/tree/ROS2
  - into src in the container (...) 
- `cd ~/path/to/your/container/workspace/src `
- `git clone <url to git hub> `
- `git clone -b ROS2 --single-branch https://github.com/hku-mars/FAST_LIO.git `

OR

vcs import < livox.repox & vcs import < racer.repos
- `cd ~/path/to/your/container/workspace/src `
- `vcs import < livox.repos`
- `vcs import < racer.repos`

# To get Nav2 stack:
Update package index
 `sudo apt update `

# Install nav2 packages
```
sudo apt install ros-<ros-distro>-navigation2 
ros-<ros-distro>-nav2-bringup
source /opt/ros/<ros-distro>/setup.bash
```

## Install livox_ros_driver2 and Livox-SDK/Livox-SDK2 (sent in discord)

Develop our own code for 
Check out robocar 

robocar/repos/racer.repos:

# Pointcloud->Laser scan
- https://github.com/ros-perception/pointcloud_to_laserscan/tree/foxy
#
In this code change the Triton AI to hku-mars/FAST LIO… 
`Git.clone https://github.com/hku-mars/FAST_LIO/tree/ROS2`
  - Change IP address of livox_ros_driver2/config/MID360_config.json
  - Change line 28 to 192.168.1.124
## Launching Lidar
`ros2 launch livox_ros_driver2 rviz_MID360_launch.py`

## Installing Livox-Pointcloud2
Launch `ros2 run livox_to_pointcloud2 livox_to_pointcloud2_node`

## Pointcloud-Laserscan


