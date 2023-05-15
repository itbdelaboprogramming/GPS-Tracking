# Human detection

This folder contain all the code to detect human from the frame image of a camera. 

## How to Use

1. Download/Pull the code from GitHub
2. Go to GPS-Tracking/MSD700_Follow_Me/human_detection directory

If you run this in Jetson device or any device that have Python installed globally (not recommended), then you can skip this step and jum to step 6

3. Install [virtual environment](https://python.land/virtual-environments/virtualenv) for Python
4. Activate the virtual environment
```powershell
venv\Scripts\Activate.ps1
```
5. Install [opencv-python](https://pypi.org/project/opencv-python/)

6. Go back to GPS-Tracking/MSD700_Follow_Me/human_detection directory
7. Run `follow_me.py` script
If succcess, the terminal will print the information about darknet model and the device camera used. A pop up will appear in the screen and show the frame from the camera. To exit the script just press `ESC` key in your keyboard.

These are the example of the script's output when running in Windows Machine.
```powershell
Initiating Darknet ...
Loading model from  D:\Nakayama Iron Works\GPS-Tracking\MSD700_Follow_Me\human_detection\weights/yolov3-tiny.weights
Loading config from  D:\Nakayama Iron Works\GPS-Tracking\MSD700_Follow_Me\human_detection\cfg/yolov3-tiny.cfg
Loading names from  D:\Nakayama Iron Works\GPS-Tracking\MSD700_Follow_Me\human_detection\coco.names
Loading camera ...
Starting on device  0
Frame width:  640
Frame height:  480
Frame channel:  3
[ WARN:0@5.379] global net_impl.cpp:174 cv::dnn::dnn4_v20221220::Net::Impl::setUpNet DNN module was not built with CUDA backend; switching to CPU
Key 27 is pressed.
Stream end here
```

If you want to use another camera and not your default one, try to run the script with `-c <device-id>` flag. 
```powershell
python follow_me.py -c 4
```
This will use camera with device id 4. For reference when using Intel Realsense camera in itbdelabof3 computer, the device id is 8 while using it in Jetson Xavier NX the device id is 4

## Files and Scripts Explanation

### Files and Folders

These are the important files and folders that being used for human detection. All the files that not mention is not that important and only used for experiment and testing.

1. [cfg](/MSD700_Follow_Me/human_detection/cfg/) and [weights](/MSD700_Follow_Me/human_detection/weights/)

These folders contain the configuration and weights of the model for [Darknet-YOLO](https://pjreddie.com/darknet/yolo/) framework.

2. [coco.names](/MSD700_Follow_Me/human_detection/coco.names)

The YOLO framework not only can detect human but all sort of things. This file contain the label for those objects. But in the code we will use only `Person`.

3. [darknet_yolo.py](/MSD700_Follow_Me/human_detection/darknet_yolo.py)

This file contain the class for handling the Neural Network for detecting human. The code use [Darknet](https://pjreddie.com/darknet/yolo/) framework for the model of the Deep Neural Network. For more explanation read below.

4. [device_camera.py](/MSD700_Follow_Me/human_detection/device_camera.py)

This file contain the class for handling camera, frame and fps. For more explanation read below.

5. [follow_me.py](/MSD700_Follow_Me/human_detection/follow_me.py)

This file is the main script for human detection. Run this script to open the camera and detect the human in the camera frame.

### Main Script

The main script to run the human detector is [follow_me.py](/MSD700_Follow_Me/human_detection/follow_me.py). The file follows this flowchart.

![Human_detection_flowchart](/MSD700_Follow_Me/human_detection/img/Flowchart%20human-detection.png?raw=true "Human detection flowchart")

Now open the file [follow_me.py](/MSD700_Follow_Me/human_detection/follow_me.py) and see these parts of script.
```python
import argparse

parser = argparse.ArgumentParser(
    description= 'This program will detect human',
    epilog= 'Hope this works'
)
group = parser.add_mutually_exclusive_group()
group.add_argument('-c', '--camera', type=int, default=0, help='Camera device id')
group.add_argument('-i', '--image', action='store_true', help='Image directory')
args = parser.parse_args()
#print(args)

if args.image:
    image_path = input('Enter the path to the image: ')
    print(image_path)
```

In this part of the script, `argparse` is used to handle when the script is runnning in Command Line Interface (CLI). This part code just to make it easier to change the device id of the camera that we used so we don't have to make a change in the main script. For more about `argparse` you can learn it [here](https://docs.python.org/3/library/argparse.html).

```python
from device_camera import *
from darknet_yolo import *

net = DarknetDNN()
camera = DeviceCamera(device_id=args.camera)
```

In this part of the script, we imnport `DarknetDNN()` and `DeviceCamera()` from [darknet_yolo.py](/MSD700_Follow_Me/human_detection/darknet_yolo.py) and [device_camera.py](/MSD700_Follow_Me/human_detection/device_camera.py). After that we create an object for each classes. For `device id` the default value is `0` but we can change it when running the script by using `-c <device-id>` flag.

```python
while True:
    #Get frame from camera
    frame = camera.get_frame()

    #Detect human from the frame
    net.detect_object(frame)
    
    #Draw bounding box of the human detected
    net.draw_object(frame)

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
```

This part is the main loop of the code, first we get one `frame` from the camera. Then we feed that frame into our DNN model. The DNN model then will get the position of the object that we want to detected in the frame. Then we draw a bounding box around the object that we detected. This will help us visualize the object detection alghorithm.

We want to know if the object that we detected is located in the center, left or right side relative to our camera. The frame then divided into 3 area. For visualize the area, we use `camera.create_grid()` method.

After all the processing of the frame, we show the final result of the object detection.

For exit condition, we chcekc if the keyh `ESC` is being pressed or not. The key number for `ESC` is `27`.

After exiting the loop, we then turn-off the camera using `camera.release()`.

### Device Camera Class

Now open the file [device_camera.py](/MSD700_Follow_Me/human_detection/device_camera.py).

```python
import cv2
import time

class DeviceCamera:
    def __init__(self, device_id = 0, winname = 'Device camera output'):
        pass

    def get_frame(self):
        return self.frame
    
    def create_grid(self):
        pass
    
    def show(self):
        pass

    def release(self):
        pass
```

This class has constructor and 1 function and 3 methods. We use `OpenCV` for handling the frame from camera and `time` for calculating the Frame Per Second (FPS) of the resulting video stream. Each description of the function and methods can be seen below.

```python
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
```

This is the constructor of the DeviceCamera class. It takes 2 parameter which are `device_id` and `winname`. By default the value of each are `device_id = 0` and `winname = 'Device camera output'`. THese value than be used to call `cv2.VideoCapture()` to accessing the camera.

The constructor will alse initialize variable that will be used for FPS calculation and to set the fonts for displaying the FPS info into the frame. ([More about font parameter](https://docs.opencv.org/4.x/d6/d6e/group__imgproc__draw.html#ga5126f47f883d730f633d74f07456c576))

Variable `self.execute_once` is a variable that we used as a flag to execute some function once when in the loop.

```python
class DeviceCamera:
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
```

The function `get_frame()` is used to capture frame from video stream. We then extract the information about the stream and print it once.

To calculate the FPS of the video stream, we track the time of the frame that we got then calculate the difference of that frame relative to the frame that we get previously. The FPS is the inverse of that differenc in time. We then put the FPS that we got into the frame of the video stream.

After that return that frame.

