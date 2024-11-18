# MAE-ECE148-Team11 Final Project
ssh -X jetson@ucsd-yellow.local
Pw:  jetsonucsd

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
 `cd ~/path/to/your/container/workspace/src `
 `git clone <url to git hub> `
 `git clone -b ROS2 --single-branch https://github.com/hku-mars/FAST_LIO.git `

# To get Nav2 stack:
Update package index
 `sudo apt update `

# Install nav2 packages
 `sudo apt install ros-<ros-distro>-navigation2 ros-<ros-distro>-nav2-bringup
source /opt/ros/<ros-distro>/setup.bash `

## Install livox_ros_driver2 and Livox-SDK/Livox-SDK2 (sent in discord)

Develop our own code for 
Check out robocar 

robocar/repos/racer.repos:
In this code change the Triton AI to hku-mars/FAST LIO… 
`Git.clone https://github.com/hku-mars/FAST_LIO/tree/ROS2`
  - Change IP address of livox_ros_driver2/config/MID360_config.json
  - Change line 28 to 192.168.1.124
## Launching Lidar
ros2 launch livox_ros_driver2 rviz_MID360_launch.py
