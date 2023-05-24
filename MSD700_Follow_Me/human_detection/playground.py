import pyrealsense2 as rs
import numpy as np
import cv2

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

try:
    while True:
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        if not depth_frame or not color_frame:
            continue

        # Convert depth frame to a numpy array
        depth_image = np.asanyarray(depth_frame.get_data())

        # Normalize the depth values for visualization
        depth_image = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8UC1)

        # Convert color frame to a numpy array
        color_image = np.asanyarray(color_frame.get_data())

        # Display the depth image
        cv2.imshow('Depth Image', depth_image)

        # Display the color image
        cv2.imshow('Color Image', color_image)

        key = cv2.waitKey(1)

        # Exit loop if 'q' is pressed
        if key == ord('q'):
            break

finally:
    # Stop streaming
    pipeline.stop()

# Close all windows
cv2.destroyAllWindows()
