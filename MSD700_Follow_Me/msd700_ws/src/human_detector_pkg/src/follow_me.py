#!/usr/bin/env python3

import rospy
import time
from std_msgs.msg import UInt8
from darknet_yolo import *
from device_camera import *

#Initialize Darknet and Camera
net = DarknetDNN()
camera = DeviceCamera(0)
start_time = time.time()
frequency = 10 #in Hz

rospy.init_node('follow_me_node')
pub = rospy.Publisher('rover_command', UInt8, queue_size=10)

while not rospy.is_shutdown():
    #Get frame from camera
    frame = camera.get_frame()

    #Detect human from the frame
    net.detect_object(frame)
    
    #Draw bounding box of the human detected
    net.draw_object(frame)

    #Publish the command
    if time.time() - start_time >= 1/frequency:
        direct = net.get_command()
        if direct == 'Right':
            command = 1
        elif direct == 'Left':
            command = 2
        elif direct == 'Center':
            command = 3
        else:
            command = 0
        
        rospy.loginfo(command)
        pub.publish(command)

        start_time = time.time()

    #Draw grid
    camera.create_grid()

    #Display the image
    camera.show()

    #exit condition
    key = cv2.waitKey(1)
    if key == 27:
        print(f"Key {key} is pressed.")
        break

camera.release()
