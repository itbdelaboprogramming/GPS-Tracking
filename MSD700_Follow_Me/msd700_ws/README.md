# ROS Workspace for MSD700  
  
This folder contain all the file for ROS Workspace of MSD700 "Follow Me" project. Before reading this, you may learn about [ROS Workspace](http://wiki.ros.org/ROS/Tutorials) and it's structure.
  
## Prerequisite
  
1. Installed [ROS](http://wiki.ros.org/ROS/Installation). In this project, we use [ROS Noetic](http://wiki.ros.org/noetic/Installation) distribution.
2. Installed [rosserial_arduino](http://wiki.ros.org/rosserial_arduino). With this package we can connect our machine with Arduino and communicate with it using ROS.  

If you haven't installed `rosserial_arduino`, here are the step to install it.  
Open the terminal and run the following command.
```bash
sudo apt-get install ros-$ROS_DISTRO-rosserial-arduino
sudo apt-get install ros-$ROS_DISTRO-rosserial
```
For our development, because we use ROS Noetic the command that we use are 
```bash
sudo apt-get install ros-noetic-rosserial-arduino
sudo apt-get install ros-noetic-rosserial
```
  
After installing `rosserial` for our machine, we installed `rosserial` arduino library. 
1. Open the Arduino IDE, in this case we use Arduino IDE 1.8.19 but this may work in other version as well.
2. Open **Sketch** tab then select **Include Library** and click **Manage Libraries**.
3. This will open **Library Manager** Window. Search for *rosserial* and install the library.

# How to Use  

To check the system, we will move the servo based on the position of human detected by the camera.

## Setup on Arduino Board  

Just upload [this](/MSD700_Follow_Me/Arduino/control_servo_with_ros/control_servo_with_ros.ino) sketch into the Arduino Board. In this case we will use Arduino Uno R3 Board.  

Connect the servo to the Arduino Board.  

![Arduino Board](/MSD700_Follow_Me/img/servo_wiring.png?raw=true "Arduino Wiring")

## Setup on Your Machine
  
To use ROS, we need Ubuntu installed on our machine. In this case we will use itbdelabof3 in Lab, but this code should work on any machine that use Ubuntu. Next we may test it in Jetson Xavier NX.  
  
1. Download all the code from the `msd700_ws` or just pull all the code from this repo.
2. Go to the `msd700_ws` directory. For this case we go to `/home/itbdelabof3/allproject/GPS-Tracking/MSD700_Follow_Me/msd700_ws`
3. Run `catkin_make` command in terminal.

```bash
catkin_make
```

4. Source the setup file. Check the output of `catkin_make` from previous step. See the *devel space*.

```bash
Base path: /home/itbdelabof3/allproject/GPS-Tracking/MSD700_Follow_Me/msd700_ws
Source space: /home/itbdelabof3/allproject/GPS-Tracking/MSD700_Follow_Me/msd700_ws/src
Build space: /home/itbdelabof3/allproject/GPS-Tracking/MSD700_Follow_Me/msd700_ws/build
Devel space: /home/itbdelabof3/allproject/GPS-Tracking/MSD700_Follow_Me/msd700_ws/devel
Install space: /home/itbdelabof3/allproject/GPS-Tracking/MSD700_Follow_Me/msd700_ws/install
...
```

The `setup.bash` file is located in this `Devel` folder.

```bash
source /home/itbdelabof3/allproject/GPS-Tracking/MSD700_Follow_Me/msd700_ws/devel/setup.bash
```

Everytime we open a terminal, we need to source that `setup.bash` file. To make it easier, source that file into `.bashrc` file in the root directory so everytime we open terminal, it automatically do it for us.

5. Run the `roscore`.

```bash
roscore
```

6. Run the launch file.

```bash
roslaunch human_detector_pkg follower.launch
```

If the file is successfully launch, the output at the terminal will be like this.

```bash
... logging to /home/itbdelabof3/.ros/log/a9a53544-f395-11ed-b034-7bcaf4a26fc2/roslaunch-itbdelabof3-OptiPlex-7470-AIO-210559.log
Checking log directory for disk usage. This may take a while.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://itbdelabof3-OptiPlex-7470-AIO:36521/

SUMMARY
========

PARAMETERS
 * /rosdistro: noetic
 * /rosversion: 1.15.15

NODES
  /
    camera_control (human_detector_pkg/follow_me.py)
    serial_node (rosserial_python/serial_node.py)

ROS_MASTER_URI=http://localhost:11311

process[camera_control-1]: started with pid [210596]
process[serial_node-2]: started with pid [210606]
[ WARN:0@1.673] global net_impl.cpp:174 setUpNet DNN module was not built with CUDA backend; switching to CPU
```

The video stream will pop up and display the video stream of human detector.

**Please Note** that in this test we use the default camera. If we use another device as camera (for example IntelRealsense D435i), we may change the device id at [this](/MSD700_Follow_Me/msd700_ws/src/human_detector_pkg/src/follow_me.py) file. From experince, when using IntelRealsense D435i at itbdelabof3's computer, the `device id` for camera is 8 and at Jetson Xavier NX is 4.

# Code Explanation

## Arduino Code
