import pyrealsense2 as rs
import numpy as np
import cv2

class RealsenseCamera:
    def __init__(self):
        # Configure depth and color streams
        print("Loading Intel RealSense Camera")
        self.pipeline = rs.pipeline()

        config = rs.config()
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)

        # Start streaming
        self.pipeline.start(config)
        align_to = rs.stream.color
        self.align = rs.align(align_to)


    def get_frame_stream(self):
        # Wait for a coherent pair of frames: depth and color
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        if not depth_frame or not color_frame:
            # If there is no frame, probably the camera is not connected, return False
            print("Error: Unable to get the frame. Make sure that the Intel RealSense camera is correctly connected.")
            return False, None, None

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        return True, color_image, depth_image

    def release(self):
        self.pipeline.stop()


# Create an instance of the RealsenseCamera class
camera = RealsenseCamera()

while True:
    # Get the color and depth frames from the camera
    success, color_frame, depth_frame = camera.get_frame_stream()

    if not success:
        break

    # Display the depth image
    cv2.imshow("Depth Image", depth_frame)

    # Press 'q' to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera resources
camera.release()

# Close all OpenCV windows
cv2.destroyAllWindows()
