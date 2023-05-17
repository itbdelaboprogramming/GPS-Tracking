# Servo Control with Serial (Discontinued)

This code is used to test if direct serial communication from computer is suitable to control the movement of the rover. For this test we try to control the movement of the servo. **From the test we concluded that communication using direct serial communication isn't suitable for the project.** When using direct serial, the human detection algorithm flow is disturbed. ecause of this, we will use ROS for communicating between computer and Arduino.  

# How to Use

We will need these file to show the direct serial communication:

1. [Arduino Sketch](/MSD700_Follow_Me/Arduino/control_servo_with_serial/control_servo_with_serial.ino)
2. [Python Script](/MSD700_Follow_Me/serial_control/follow_me.py)

## Setup on Arduino Board

Just upload [this](/MSD700_Follow_Me/Arduino/control_servo_with_serial/control_servo_with_serial.ino) sketch into the Arduino Board. In this case we will use Arduino Uno R3 Board.  

Connect the servo to the Arduino Board.  

![Arduino Board](/MSD700_Follow_Me/img/servo_wiring.png?raw=true "Arduino Wiring")

## Setup on Your Machine

For this exposition, there is no need to specific setup for the machine. We just follow the steps from *[human_detection](/MSD700_Follow_Me/human_detection/README.md#how-to-use)*.

# Code Explanation

