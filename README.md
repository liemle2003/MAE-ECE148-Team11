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
    <li><a href="#How To Run">Project Reproduction</a></li>
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
* Need to edit your guys accomplisments 
<hr>

## Challenges
* Need to edit challenges 
<hr>

## Software

### Laserscan
- Laserscan was another key component to our project which helped enable efficient processing of lidar data for environmental mapping and localization within the ROS2 framework. By using LaserScan data, we were able to detect and visualize the robot’s surroundings in real time, providing a clear 2D representation of spatial layouts. This was also visualized using Rviz2 which optimized visualization settings for clarity.
## Fast_Lio and PointCloud
- The Fast LIO was integral to our project, which helped enable integration of lidar odometry and mapping into the ROS2 framework. This setup allowed the robot to process lidar data in real time and generate accurate odometry for localization  while simultaneously creating detailed point cloud representations of the environment. These point clouds were visualized using the tool, Rviz2 and this helped assess the environment in 3D which also helped the robot to map and detect obstacles.
### Odometry
- The odometry system was crucial for our project, which provided the foundation for accurate localization and navigation within the ROS2 framework. The system allowed the robot to determine its precise position and orientation relative to its environment. This was also visualized using Rviz2, which helped provide a clear and smooth tracking of the robot’s movement.

## Need to edit the gazebo/more softwares on

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

## How To Run



<hr>


## Acknowledgements
Special thanks to Professor Jack Silberman for delivering the course!  
Thanks to TA Winston Chou and Ta Alexander for giving suggestions to our project!  


**Programs Reference:**
* [UCSD Robocar Framework](https://gitlab.com/ucsd_robocar2)
* [Slam Toolbox](https://github.com/SteveMacenski/slam_toolbox.git)
* [Triton AI](https://github.com/Triton-AI)


README.md Format, reference to [spring-2024-final-project-team-7](https://github.com/UCSD-ECEMAE-148/winter-2024-final-project-team-7)

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


