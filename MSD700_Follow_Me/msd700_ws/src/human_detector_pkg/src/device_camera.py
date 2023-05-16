import cv2
import time

class DeviceCamera:
    def __init__(self, device_id = 0, winname = 'Device camera output'):
        print("Loading camera ...")
        #Device Camera initialization
        self.device_id = device_id
        self.winname = winname
        self.capture = cv2.VideoCapture(self.device_id)
        print("Starting on device ", self.device_id)
        
        #FPS Calculation
        self.previous_frame_time = 0
        self.current_frame_time = 0
        self.fps = 0

        #Font parameters
        self.font_face = cv2.FONT_HERSHEY_SIMPLEX
        self.org = (0, 50)
        self.font_scale = 1
        self.font_color = (90, 252, 3)
        self.font_thickness = 2
        self.font_line_type = cv2.LINE_AA
        self.font_bottom_left_origin = False

        #Miscellaneous
        self.execute_once = True


    def get_frame(self):
        #Frame reading
        self.retval, self.frame = self.capture.read()
        self.frame_height, self.frame_width, self.frame_channel = self.frame.shape

        #Print info about the frame
        if self.execute_once:
            print("Frame width: ", self.frame_width)
            print("Frame height: ", self.frame_height)
            print("Frame channel: ", self.frame_channel)
            self.execute_once = False

        #FPS Calculation
        self.current_frame_time = time.time()
        self.fps = round(1/(self.current_frame_time - self.previous_frame_time), 2)
        self.previous_frame_time = self.current_frame_time

        #Put the FPS on the frame
        cv2.putText(self.frame, f"FPS: {self.fps}", self.org, self.font_face, self.font_scale, self.font_color, self.font_thickness, self.font_line_type, self.font_bottom_left_origin)

        return self.frame
    
    def create_grid(self):
        self.frame = cv2.line(self.frame, (int(self.frame_width/3), 0), (int(self.frame_width/3), self.frame_height), (0, 255, 0), 1)
        self.frame = cv2.line(self.frame, (int(2*self.frame_width/3), 0), (int(2*self.frame_width/3), self.frame_height), (0, 255, 0), 1)
    
    def show(self):
        if self.retval:
            cv2.imshow(self.winname, self.frame)
        else:
            print("No frame detected")

    def release(self):
        self.capture.release()
        print("Stream end here")

