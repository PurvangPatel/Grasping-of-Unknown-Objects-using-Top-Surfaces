# Directed Research - Fall B-term 2022 - Vision Based Robotic Manipulation 

Project : GRASPING OF UNKNOWN OBJECTS USING TOP SURFACES FROM A TABLE TOP
              
Contributors: Ashwij Kumbla (akumbla@wpi.edu);
              Anoushka Baidya (abaidya@wpi.edu);
              Purvang Patel (pppatel@wpi.edu);
              Krutarth Ambarish Trivedi (ktrivedi@wpi.edu)
              
Mentor/Advisor: Dr. Berk Calli(bcalli@wpi.edu)

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

Open <workspace>/src/Grasping-of-Unknown-Objects-using-Top-Surfaces/src/main.cpp -> Change the line number 196 to /<workspace>/Grasping-of-Unknown-Objects-using-Top-Surfaces/Media/

Open a terminal and run:

**Build the ROS2 Package**
``` 
colcon build --packages-select Grasping-of-Unknown-Objects-using-Top-surfaces
```

**Launch the enviornment**
```
ros2 launch Grasping-of-Unknown-Objects-using-Top-Surfaces simulation.launch.py
```

**Run the ROS2 Node to process the data**
```
ros2 run Grasping-of-Unknown-Objects-using-Top-Surfaces PointCloudProcessor 
```

# ROS2 Workspace Structure

**Media**
  
Contains all the PCDs files including the point cloud data received from the simulator, clustered objects, estimnated boundaries and grasp points.

**Algorithms/Cavity_Detection**
  
To run the cavity detection algorithm approach - Perform the below minor read & write path change and follow the build instructions. 

Change directory in line numbers 225-237 to: /<workspace>/Grasping-of-Unknown-Objects-using-Top-Surfaces/Algorithms/Cavity_Detection/Media/<filename>.pcd

```
cd <workspace>/src/Grasping-of-Unknown-Objects-using-Top-Surfaces/Algorithms/Cavity_Detection
```

```
rm -rf build
```

```
mkdir -p build
```

```
cd build
```

```
cmake ../
```

```
make -j16 
```
(note: -j<number of preferred core>)

**rviz2**
  
When the simulation and Point cloud processor are running, you may want to run this visualization file to see all the objects with their respective grasp points.
