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

1. Setup on Arduino Board  

Just upload [this](/MSD700_Follow_Me/Arduino/control_servo_with_ros/control_servo_with_ros.ino) sketch into the Arduino Board. In this case we will use Arduino Uno R3 Board.  

Connect the servo to the Arduino Board.  

![Arduino Board](/MSD700_Follow_Me/img/servo_wiring.png?raw=true "Arduino Wiring")

2. Setup on Your Machine
  
To use ROS, we need Ubuntu installed on our machine. In this case we will use itbdelabof3 in Lab, but this code should work on any machine that use Ubuntu. Next we may test it in Jetson Xavier NX.  
  
