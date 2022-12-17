# VBRM-DR

Project : GRASPING OF UNKNOWN OBJECTS USING TOP SURFACES FROM A TABLE TOP
              
Contributors: Ashwij Kumbla (akumbla@wpi.edu);
              Anoushka Baidya (abaidya@wpi.edu);
              Purvang Patel (pppatel@wpi.edu);
              Krutarth Ambarish Trivedi (ktrivedi@wpi.edu)

# Requirements
ROS2Humble
Gazebo
Rviz2
pcl-dev
pcl-tools

# Instructions for Setting Project Environment

**Clone/download the OSRF gazebo_models git repository**
```
git clone https://github.com/osrf/gazebo_models
```
Extract it into ~/.gazebo/models

.gazebo is a hidden folder, you can see those by pressing Ctrl + H when using **Files** application.


**Install pcl-conversion lib**
```
sudo DEBIAN_FRONTEND=noninteractive apt-get install ros-humble-pcl-conversions -y
```

**Install pcl-tools**
```
sudo apt-get install -y pcl-tools
```

# Running the project

Clone the ROS2 package in your local folder. Make sure to source the setup.bash

Open <WORKSPACE>/vbm_project_env/src/main.cpp -> Change line number --- to the directory where you want to save the processed point cloud data.

Open a terminal and run:

**Build the ROS2 Package**
``` 
colcon build --packages-select vbrm-dr
```

**Launch the enviornment**
```
ros2 launch vbrm-dr simulation.launch.py
```

**Run the ROS2 Node to process the data**
```
ros2 run vbrm-dr PointCloudProcessor 
```

