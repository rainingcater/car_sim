# ROS Packages for Scout Mobile Robot

## Packages

This repository contains minimal packages to control the scout robot using ROS. 

* scout_bringup: launch and configuration files to start ROS nodes 
* scout_base: a ROS wrapper around [ugv_sdk](https://github.com/westonrobot/ugv_sdk) to monitor and control the scout robot
* scout_description: URDF model for the mobile base, a sample urdf (scout_description/sample/scout_v2_nav.xacro) is provided for customized robot with addtional sensors
* scout_msgs: scout related message definitions

## Supported Hardware

* Scout
* Scout Mini
* Scout Mini Omni

**Note:** Both V1 and V2 protocols are supported by this package (only with CAN interface). You don't have to specify the protocol version when launching the base node but you need to use the right launch file for your specific Scout model.  

## Communication interface setup

Please refer to the [README](https://github.com/westonrobot/ugv_sdk/blob/next/README.md) of the "ugv_sdk" package for setup of communication interfaces.

**Note on CAN interface on Nvidia Jetson Platforms:** Nvidia Jeston TX2/Xavier/XavierNX have CAN controller(s) integrated in the main SOC. If you're using a dev kit, you need to add a CAN transceiver for proper CAN communication. 

## Basic usage of the ROS packages

1. Install dependent libraries

    ```
    $ sudo apt install -y ros-$ROS_DISTRO-teleop-twist-keyboard
    ```

2. Clone the packages into your catkin workspace and compile

    (the following instructions assume your catkin workspace is at: ~/catkin_ws/src)

    ```
    $ cd ~/catkin_ws/src
    $ git clone https://github.com/westonrobot/ugv_sdk.git
    $ git clone https://github.com/westonrobot/scout_ros.git
    $ cd ..
    $ catkin_make
    ```

3. Launch ROS nodes
 
* Start the base node for the real robot

    ```
    $ roslaunch scout_bringup scout_minimal.launch
    ```

    The [scout_bringup/scout_minimal.launch](scout_bringup/launch/scout_minimal.launch) has 4 parameters:

    - port_name: specifies the port used to communicate with the robot, default = "can0"
    - simulated_robot: indicates if launching with a simulation, default = "false"
    - model_xacro: specifies the target ".xacro" file for the publishing of tf frames, default = [scout_v2.xacro](scout_base/description/scout_v2.xacro)
    - odom_topic_name: sets the name of the topic which calculated odometry is published to, defaults = "odom"

* Start the keyboard tele-op node

    ```
    $ roslaunch scout_bringup scout_teleop_keyboard.launch
    ```

**SAFETY PRECAUSION**: 

The default command values of the keyboard teleop node are high, make sure you decrease the speed commands before starting to control the robot with your keyboard! Have your remote controller ready to take over the control whenever necessary. 

## Update the packages for your customized robot

**Additional sensors**

It's likely that you may want to add additional sensors to the scout mobile platform, such as a Lidar for navigation. In such cases, a new ".xacro" file needs to be created to describe the relative pose of the new sensor with respect to the robot base, so that the sensor frame can be reflected in the robot tf tree. 

A [sample](scout_description/sample/scout_v2_nav.xacro) ".xacro" file is present in this repository, in which the base ".xacro" file of an empty scout platform is first included, and then additional links are defined. 

The nodes in this ROS package are made to handle only the control of the scout base and publishing of the status. Additional nodes may need to be created by the user to handle the sensors.

**Alternative odometry calculation**

By default the scout_base package will publish odometry message to topic "/odom". In case you want to use a different approach to calculate the odometry, for example estimating the position together with an IMU, you could rename the default odometry topic to be something else.
 
```
$ scout_bringup scout_minimal.launch odom_topic_name:="<custom_name>"
```