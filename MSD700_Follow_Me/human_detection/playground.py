import cv2
import pyrealsense2 as rs
import numpy as np
from realsense_camera import *

realsense = RealsenseCamera()

tick_frequency = cv2.getTickFrequency()
start_time = cv2.getTickCount()
frame_count = 0

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
pipeline.start(config)

while True:
    frame = pipeline.wait_for_frames()
    color_frame = frame.get_color_frame()
    depth_frame = frame.get_depth_frame()

    frame_count += 1
    current_time = cv2.getTickCount()
    elapsed_time = (current_time - start_time)/tick_frequency

    if elapsed_time > 1.0:
        fps = frame_count / elapsed_time
        print(fps)
        start_time = current_time
        frame_count = 0

    
    cv2.imshow("BGR", color_frame)
    cv2.imshow("Depth", depth_frame)

    #exit condition
    key = cv2.waitKey(1)
    if key == 27:
        print(f"Key {key} is pressed.")
        break

pipeline.stop()
cv2.destroyAllWindows()
#camera.release()