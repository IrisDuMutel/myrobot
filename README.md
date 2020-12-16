# myrobot
This project intends to simulate the behaviour of a Devastator ground robot.
It includes homemade plugins and also some from gazebo_ros_pkgs (https://github.com/ros-simulation/gazebo_ros_pkgs).

## To use this repo follow the next steps:

-Clone the repo:
    
    git clone https://github.com/IrisDuMutel/myrobot.git
    
    
-Build the package to  get plugin executables and so on:

    
    cd catkin_ws
    catkin_make
    
    
-Check if it's working by launching any of the launch files included:

    
    roslaunch myrobot myrobot.launch



## Models

This repo contains more than one model. However, the main scope is to obtain a model from a real-life ground robot. For such purposes, the two existing formats (SDF & URDF) have been used to create to very similar models. Progress is being made with the URDF model due to ROS packages requirements. To launch either model, there are two launch files:

        roslaunch myrobot myrobot.launch # launches the SDF model in an empty world
        roslaunch myrobot myroboturdf.launch # launches the URDF model in an empty world
The rest of launch files use such models and a wide variety of maps, environmnet objects and spawn positions. Feel free to play with them.


## SLAM Gmapping
To perform SLAM, one must launch:
        roslaunch myrobot myroboturdf.launch
        roslaunch myrobot slam_gmapping.launch
To use this function, the following dependencies are required:
- slam_gmapping (ROS package)
- robot_state_publisher (ROS package)
  
## Machine learning

The machine learning scripts used in this project belong to :
    https://emanual.robotis.com/docs/en/platform/turtlebot3/machine_learning/#machine-learning

Minimal changes have been done to allow its correct functioning in ROS noetic.

    


    
