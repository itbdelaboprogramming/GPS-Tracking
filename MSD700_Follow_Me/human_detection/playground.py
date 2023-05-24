import cv2
import pyrealsense2 as rs
import numpy as np
#from realsense_camera import *

#realsense = RealsenseCamera()

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 20)
pipeline.start()

print("Starting ...")
tick_frequency = cv2.getTickFrequency()
start_time = cv2.getTickCount()
frame_count = 0

try:
    while True:
        # Wait for a coherent pair of frames: color and depth
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        if not color_frame:
            continue

        # Convert the color frame to a numpy array
        color_image = np.asanyarray(color_frame.get_data())

        # Convert color ordering from RGB to BGR
        color_image = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)

        frame_count += 1
        current_time = cv2.getTickCount()
        elapsed_time = (current_time - start_time)/tick_frequency

        if elapsed_time >= 1.0:
            fps = frame_count / elapsed_time
            print(fps)
            start_time = current_time
            frame_count = 0

        # Display the resulting frame
        cv2.imshow('Video Stream', color_image)

        #exit condition
        key = cv2.waitKey(1)
        if key == 27:
            print(f"Key {key} is pressed.")
            break
    
except Exception as e:
    print(e)
    pass
finally:
    pipeline.stop()
    cv2.destroyAllWindows()

