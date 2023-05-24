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

# Create a variable to store the depth scale
depth_scale = None

# Retrieve the depth scale from the pipeline
profile = pipeline.get_active_profile()
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()

# Create a variable to store the depth intrinsics
depth_intrinsics = None

# Retrieve the depth intrinsics from the pipeline
depth_stream = profile.get_stream(rs.stream.depth)
depth_intrinsics = depth_stream.as_video_stream_profile().get_intrinsics()

# Create a variable to store the clicked point
clicked_point = None

# Mouse callback function
def mouse_callback(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        global clicked_point
        clicked_point = (x, y)

# Set the mouse callback function
cv2.namedWindow('Color Image')
cv2.setMouseCallback('Color Image', mouse_callback)

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

        # Check if a point has been clicked
        if clicked_point is not None:
            # Get the depth value at the clicked point
            x, y = clicked_point
            depth_value = depth_frame.get_distance(x, y)

            # Convert the depth value to meters
            depth_value_meters = depth_value * depth_scale

            # Print the depth value
            print(f"Depth value at clicked point: {depth_value_meters:.3f} meters")

            # Reset the clicked point
            clicked_point = None

        key = cv2.waitKey(1)

        # Exit loop if 'q' is pressed
        if key == ord('q'):
            break

finally:
    # Stop streaming
    pipeline.stop()

# Close all windows
cv2.destroyAllWindows()
