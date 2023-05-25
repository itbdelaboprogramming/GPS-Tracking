import cv2
import numpy as np
import time

class DeviceCamera:
    """docstring for DeviceCamera."""
    def __init__(self, device_id = None, realsense = True):
        print("Loading camera")

        # Check if pyrealsense2 is available
        self.realsense, self.rs = self.check_pyrealsense2() if realsense else (False, None)
        ''' 
        try:
            import pyrealsense2 as rs
            self.realsense = True
            print("Realsense is used")
            pass
        except Exception as e:
            self.realsense = False
            print("pyrealsense2 can't be opened")
        '''

        # Camera parameter initialization
        self.device_id = device_id
        self.device_ids = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
        self.capture = None
        self.pipeline = None
        self.winname = None

        # Initialize device
        print(self.realsense)
        print(self.rs)
        if self.realsense:
            print("Starting realsense")
            self.winname = "Realsense"
            self.stream_realsense()
        else:
            print("Starting regular stream")
            self.winname = "Regular Stream"
            self.stream_regular()
    
    def check_pyrealsense2(self):
        try:
            import pyrealsense2
            rs = pyrealsense2
            print("Pyrealsense2 is available")
            return True, rs
        except ImportError:
            print("Pyrealsense2 is not available")
            return False, None

    def stream_realsense(self):
        # Configure depth and color streams
        self.pipeline = self.rs.pipeline()
        self.config = self.rs.config()
        self.config.enable_stream(self.rs.stream.color, 1280, 720, self.rs.format.bgr8, 30)
        self.config.enable_stream(self.rs.stream.color, 640, 480, self.rs.format.bgr8, 30)
        #self.config.enable_stream(self.rs.stream.depth, 1280, 720, self.rs.format.z16, 30)

        # Start streaming
        self.pipeline.start(self.config)
        align_to = self.rs.stream.color
        self.align = self.rs.align(align_to)

    def stream_regular(self):
        # Searching for the first available device id if not specified
        if self.device_id is None:
            self.device_id = self.search_available_device_id()
        
        # Start streaming
        self.capture = cv2.VideoCapture(self.device_id)
        print("Starting on device ", self.device_id)

    def get_frame(self):
        if self.realsense:
            # Read the incoming frame from Realsense
            frames = self.pipeline.wait_for_frames()

            # Aligned the color frame and depth frame
            aligned_frames = self.align.process(frames)
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()

            # Check if the stream is success or not
            if not color_frame or not depth_frame:
                print("Error, impossible to get the frame, make sure that the Intel Realsense camera is correctly connected")
                return None, None
            
            # Convert the frame into Matrix
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())

            return color_image, depth_image
            pass
        else:
            # Read the incoming frame from Regular Camera
            retval, frame = self.capture.read()

            return frame, None
            pass
        pass

    def show_color(self):
        color, depth = self.get_frame()
        cv2.imshow(self.winname, color)
    
    def show_depth(self):
        color, depth = self.get_frame()
        cv2.imshow(self.winname, depth)

    def validate_device_id(self, device_id):
        try:
            cap = cv2.VideoCapture(device_id)
            is_opened = cap.isOpened()
            cap.release()
            return is_opened
        except Exception as e:
            return False
    
    def search_available_device_id(self):
        for device_id in self.device_ids:
            if self.validate_device_id(device_id):
                return device_id
        return None
    
    def available_device_id(self):
        available_device = [device_id for device_id in self.device_ids if self.validate_device_id(device_id)]
        return available_device

def main():
    camera = DeviceCamera()
    
    while True:
        color, depth = camera.get_frame()

        cv2.imshow("Color", color)
        #cv2.imshow("Depth", depth)

        key = cv2.waitKey(1)
        if key == ord('q') or key == 27:
            print(f"Key {key} is pressed.")
            break

if __name__ == "__main__":
    main()


        