# Human detection

This folder contain all the code to detect human from the frame image of a camera. 

* [How to Use](#how-to-use)  
    * [Files and Scripts Explanation](#files-and-scripts-explanation)  
    * [Files and Folders](#files-and-folders)  
    * [Main Script](#main-script)  
    * [Device Camera Class](#device-camera-class)  
    * [DarknetDNN Class](#darknetdnn-class)  
    * [Darknet](#darknet)  
        * [YOLO](#yolo)  
            * [Class](#class)  
                * [Constructor](#constructor)  
                * [Methods and Function](#methods-and-function)  
    
## How to Use

1. Download/Pull the code from GitHub
2. Go to GPS-Tracking/MSD700_Follow_Me/human_detection directory

If you run this in Jetson device or any device that have Python installed globally (not recommended), then you can skip this step and jump to step 6

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

For exit condition, we check if the key `ESC` is being pressed or not. The key number for `ESC` is `27`.

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
        pass
    
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

```python
class DeviceCamera:
    def create_grid(self):
        self.frame = cv2.line(self.frame, (int(self.frame_width/3), 0), (int(self.frame_width/3), self.frame_height), (0, 255, 0), 1)
        self.frame = cv2.line(self.frame, (int(2*self.frame_width/3), 0), (int(2*self.frame_width/3), self.frame_height), (0, 255, 0), 1)
```

The method `create_grid()` is simply just create 2 lines that will divide the frame into 3 area: `Left`, `Center`, and `Right`.

```python
class DeviceCamera:
    def show(self):
        if self.retval:
            cv2.imshow(self.winname, self.frame)
        else:
            print("No frame detected")
    
    def release(self):
        self.capture.release()
        print("Stream end here")
```

The method `show()` is used to show the frame from the camera stream.
The method `release()` is used to close the camera.

### DarknetDNN Class

#### Darknet
Darknet is an open source neural network framework written in C and CUDA. It is fast, easy to install, and supports CPU and GPU computation. You can find the source on [GitHub](https://github.com/pjreddie/darknet) or you can read more about what Darknet can do right [here](https://pjreddie.com/darknet/).

#### YOLO
YOLO is an abbreviation for the term ‘You Only Look Once’. This is an algorithm that detects and recognizes various objects in a picture (in real-time). Object detection in YOLO is done as a regression problem and provides the class probabilities of the detected images.

YOLO algorithm employs convolutional neural networks (CNN) to detect objects in real-time. As the name suggests, the algorithm requires only a single forward propagation through a neural network to detect objects.

This means that prediction in the entire image is done in a single algorithm run. The CNN is used to predict various class probabilities and bounding boxes simultaneously.

The YOLO algorithm consists of various variants. Some of the common ones include tiny YOLOv3 and YOLOv3.

[More about YOLO](https://pjreddie.com/darknet/yolo/)

##### Class

```python
import os
import cv2
import numpy as np

ROOT_DIR = os.path.dirname(__file__)

class DarknetDNN:
    def __init__(self, dnn_model = "weights/yolov3-tiny.weights", dnn_config = "cfg/yolov3-tiny.cfg"):
        pass

    def detect_object(self, image):
        pass

    def draw_object(self, frame):
        pass

    def get_command(self):
        pass
```

These are the structure of the DarknetDNN class. Some method or function may not be shown here because it is not being used right now. Those method or function may be used in debugging or in the future.

###### Constructor
```python
class DarknetDNN:
    def __init__(self, dnn_model = "weights/yolov3-tiny.weights", dnn_config = "cfg/yolov3-tiny.cfg"):
        #Initiate DNN model using Darknet framework
        print("Initiating Darknet ...")
        self.dnn_model = os.path.join(ROOT_DIR, dnn_model)
        self.dnn_config = os.path.join(ROOT_DIR, dnn_config)
        self.dnn_name_lists = os.path.join(ROOT_DIR, "coco.names")
        print("Loading model from ", self.dnn_model)
        print("Loading config from ", self.dnn_config)
        print("Loading names from ", self.dnn_name_lists)
        self.net = cv2.dnn.readNet(self.dnn_model, self.dnn_config)
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)

        ...
```
The constructor takes 2 argument which are `dnn_model` and `dnn_config`. By default those arguments values are `dnn_model = "weights/yolov3-tiny.weights"` and `dnn_config = "cfg/yolov3-tiny.cfg"`. Those values are the location of the model weights and configuration for the DNN.

Another file that might be interesting is `coco.names` which are the list of object that can be detected, but for this case, we only want to detect human.

```python
class DarknetDNN:
    def __init__(self, dnn_model = "weights/yolov3-tiny.weights", dnn_config = "cfg/yolov3-tiny.cfg"):
        ...

        self.classes = []

        with open(self.dnn_name_lists, "r") as f:
            self.classes = [line.strip() for line in f.readlines()]

        ...
```

This part of the script is used to open `coco.names` and save the list. But for this case we actually don't need this part of the code.

```python
class DarknetDNN:
    def __init__(self, dnn_model = "weights/yolov3-tiny.weights", dnn_config = "cfg/yolov3-tiny.cfg"):
        ...

        self.layer_names = self.net.getLayerNames()
        self.output_layers = [self.layer_names[i - 1] for i in self.net.getUnconnectedOutLayers()]
        self.colors = np.random.uniform(0, 255, size=(len(self.classes), 3))

        ...
```
The first line initializes a variable called self.layer_names to a list of all the layer names in the neural network. The second line initializes a variable called self.output_layers to a list of the output layer names in the neural network. These are the layers that produce the final output of the network, which may be the predicted class probabilities or the bounding boxes of detected objects. The code uses the getUnconnectedOutLayers method to obtain the indices of the output layers and subtracts one from each index to get the corresponding layer name from self.layer_names.

The third line initializes a variable called self.colors to an array of random RGB values that will be used to draw bounding boxes or labels for the detected objects. The array has a shape of (num_classes, 3), where num_classes is the number of classes that the neural network is trained to detect or classify. The np.random.uniform method is used to generate random values between 0 and 255 for each RGB channel.

In our case, we actually don't need this third line because we only detect one object which is `Person`.

```python
class DarknetDNN:
    def __init__(self, dnn_model = "weights/yolov3-tiny.weights", dnn_config = "cfg/yolov3-tiny.cfg"):
        ...

        #Blob parameter
        self.blob_scalefactor = 0.00392
        self.blob_size = (320, 320)
        self.blob_scalar = (0, 0, 0)
        self.blob_swapRB = True
        self.blob_crop = False
        self.blob_ddepth = cv2.CV_32F

        ...
```

In this part, we initialize the value for Blob parameter. Blob refers to a multi-dimensional array of data that is used as input to a neural network. A blob is typically created by pre-processing an input image to meet the requirements of the neural network's input layer.

A blob can be thought of as a higher-level representation of an image, where the pixel values are transformed into a format that can be efficiently processed by the neural network. The pre-processing steps involved in creating a blob can include resizing the input image to a fixed size, scaling the pixel values to a certain range, and reordering the color channels.

The first parameter, `self.blob_scalefactor`, is a scale factor used to normalize the pixel values of the input image. This value is multiplied by each pixel value to scale them to a range between 0 and 1. The value of 0.00392 is equivalent to dividing each pixel value by 255, which is the maximum value that a pixel can have in an 8-bit color image.

The second parameter, `self.blob_size`, specifies the size of the input image that the neural network expects. In this case, the input images are resized to a square shape with a width and height of 320 pixels. There are 5 common size for blob.
1. 224x224
2. 256x256
3. 320x320
4. 416x416
5. 512x512

The optimal value for blob may be differ for each case. In this case I just pick randomly. In the future we may be use anotbher value to get the most optimal human detector.

The third parameter, `self.blob_scalar`, is a tuple that specifies a scalar value to subtract from each pixel value in the input image. This is used to center the pixel values around zero and to help normalize the input data.

The fourth parameter, `self.blob_swapRB`, is a Boolean value that determines whether the color channels of the input image should be swapped. In OpenCV, the default order of color channels is BGR, but some neural networks may expect RGB color channels. Setting this parameter to True swaps the first and last color channels of the input image from BGR to RGB.

The fifth parameter, `self.blob_crop`, is a Boolean value that determines whether the input image should be cropped to the specified self.blob_size after resizing. If this parameter is set to True, the center region of the resized image will be cropped to the desired size.

The sixth parameter, `self.blob_ddepth`, specifies the data type of the output blob that will be fed into the neural network. In this case, the data type is cv2.CV_32F, which is a 32-bit floating point format.

```python
class DarknetDNN:
    def __init__(self, dnn_model = "weights/yolov3-tiny.weights", dnn_config = "cfg/yolov3-tiny.cfg"):
        ...

        #Threshold for detecting object
        self.confidence_threshold = 0.3
        self.nms_threshold = 0.4
```

The confidence_threshold and nms_threshold (Non-Maximum Suppression) parameters are used for object detection tasks and it affect the accuracy and reliability of the detections.

`confidence_threshold`, specifies the minimum confidence score required for an object detection to be considered valid. The confidence score represents the probability that a detected object belongs to a particular class. A higher confidence threshold will result in fewer detections, but they will be more reliable. Conversely, a lower confidence threshold will result in more detections, but some of them may be false positives. The confidence_threshold value is typically set in the range of 0.1 to 0.5.

`nms_threshold`, is used to remove redundant detections for the same object. After object detection, multiple bounding boxes may be assigned to the same object, resulting in duplicate detections. The NMS algorithm removes the redundant detections by selecting the bounding box with the highest confidence score and suppressing the others. The nms_threshold parameter controls the overlap between bounding boxes that are considered redundant. A higher nms_threshold will result in more suppression and fewer detections, while a lower nms_threshold will result in fewer suppression and more detections. The nms_threshold value is typically set in the range of 0.1 to 0.9.

The value for these two variable is chosen randomly for now, but in the future for improvement can be used another value.

###### Methods and Function

1. `detect_object()`

This method will take frame from camera and feed it into the DNN that we create.

```python
class DarknetDNN:
    def detect_object(self, image):
        #Set image into blob
        height, width, channels = image.shape
        blob = cv2.dnn.blobFromImage(image, self.blob_scalefactor, self.blob_size, self.blob_scalar, self.blob_swapRB, self.blob_crop, self.blob_ddepth)

        #Add blob as input and wait for output
        self.net.setInput(blob)
        outs = self.net.forward(self.output_layers)

        ...
```

The first part of the method is to create a blob from the input frame and feed it to the Deep Neural Network (DNN). The neural network is then run using the `self.net.forward()` method, which returns the output of the network for the specified output layers (`self.output_layers`). The output is stored in the `outs` variable for object detection.

```python
class DarknetDNN:
    def detect_object(self, image):
        ...

        #Object informations
        self.object_boxes = []
        self.object_classes = []
        self.object_centers = []
        self.object_contours = []
        self.object_positions = []
        self.object_confidences = []
        self.object_area = []

        ...
```

The code creates empty lists to store information about the objects detected in the input image. These lists later will be populated with information obtained from the output of the neural network.

* `self.object_boxes`: This list will store the bounding boxes of the objects detected in the input image. A bounding box is defined by four values: the x-coordinate and y-coordinate of the top-left corner of the box, and the width and height of the box.

* `self.object_classes`: This list will store the class labels of the objects detected in the input image. Each object is assigned a label that represents the type of object detected, such as "person", "car", or "dog".

* `self.object_centers`: This list will store the center coordinates of the bounding boxes of the objects detected in the input image. The center coordinates are calculated as the midpoint of the x-coordinate and y-coordinate of the bounding box.

* `self.object_contours`: This list will store the contour points of the objects detected in the input image. A contour is a curve that connects points of equal intensity in an image. The contour points are used to draw the boundaries of the objects in the output image.

* `self.object_positions`: This list will store the position of the objects detected in the input image. The position is wether the object is in `Left`, `Right` or `Center` of the frame.

* `self.object_confidences`: This list will store the confidence scores of the objects detected in the input image. The confidence score represents the probability that the object detected belongs to a particular class.

* `self.object_area`: This list will store the area of the objects detected in the input image. The area is calculated as the product of the width and height of the bounding box.

```python
class DarknetDNN:
    def detect_object(self, image):
        ...

        for out in outs:
            for detection in out:
                scores = detection[5:]

                class_id = np.argmax(scores)
                confidance = scores[class_id]

                # To filter object confidance
                if confidance < self.confidence_threshold:
                    continue
                
                # To filter out other than 'Person' object
                if class_id != 0:
                    continue
                
                cx = int(detection[0] * width)
                cy = int(detection[1] * height)
                w = int(detection[2] * width)
                h = int(detection[3] * height)
                area = int(w * h)

                x1 = int(cx - w/1.8)
                y1 = int(cy - h/1.8)
                x2 = int(cx + w/1.8)
                y2 = int(cy + h/1.8)

                self.object_boxes.append([x1, y1, x2, y2])
                self.object_classes.append(class_id)
                self.object_centers.append([cx, cy])
                self.object_confidences.append(confidance)
                self.object_area.append(area)
                
                if cx <= width/3:
                    self.object_positions.append('Left')
                elif cx >= 2*width/3:
                    self.object_positions.append('Right')
                else:
                    self.object_positions.append('Center')
```

This code is responsible for extracting information about the objects detected in the input image from the output of the neural network.

For each output layer in `outs`, the code iterates through the detections in that layer. For each detection, the code extracts the scores for each class, and finds the index of the class with the highest score using `np.argmax(scores)`. The confidence score for that class is then obtained from `scores[class_id]`.

The code then checks if the confidence score for the detected object is greater than the `confidence_threshold`. If it is not, the object is skipped and the code proceeds to the next detection.

The code also filters out objects that are not labeled as "Person" by checking if the `class_id` is equal to 0.

If the confidence score is greater than the `confidence_threshold` and the object is labeled as "Person", the code calculates the coordinates of the bounding box for the detected object using the values in `detection` and the dimensions of the input image (`height` and `width`). The code also calculates the area of the bounding box as `w*h`.

Finally, the code appends the bounding box coordinates, class label, center coordinates, confidence score, and area to the corresponding lists for each object detected (`self.object_boxes`, `self.object_classes`, `self.object_centers`, `self.object_confidences`, and `self.object_area`). For `self.object_positions`, just check the position of the center of the object detected.

2. `draw_object()`

```python
class DarknetDNN:
    def draw_object(self, frame):
        indexes = cv2.dnn.NMSBoxes(self.object_boxes, self.object_confidences, self.confidence_threshold, self.nms_threshold)
        for i, box, class_id, center, position in zip(range(len(self.object_boxes)), self.object_boxes, self.object_classes, self.object_centers, self.object_positions):
            if i not in indexes:
                continue

            x1, y1, x2, y2 = box
            cx, cy = center
            name = self.classes[class_id]
            #color = self.colors[int(class_id)]
            #color = (int(color[0]), int(color[1]), int(color[2]))
            color = (0, 255, 0)
            this_position = position
            
            cv2.circle(frame, (cx, cy), 10, color, 2)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 1)
            cv2.putText(frame, name.capitalize(), (x1 + 5, y1 + 25), 0, 0.8, (255, 255, 255), 2)
            cv2.putText(frame, this_position, (x1+5, y1+50), 0, 0.8, (255, 255, 255), 2)
```

This method will takes in a frame as an argument and draws rectangles and labels around the detected objects in the frame. It uses the information stored in the instance variables of the class `object_boxes`, `object_classes`, `object_centers`, and `object_positions` to draw the rectangles and labels.

First, it calls `cv2.dnn.NMSBoxes()` function to apply non-maximum suppression (NMS) to the detected object bounding boxes. This function removes overlapping bounding boxes and only keeps the ones with the highest confidence scores.

Then, it iterates through the object information stored in the instance variables and checks if the current object's index is in the `indexes` returned by NMS. If it's not, it skips the object and moves on to the next one.

For each object, it retrieves its bounding box coordinates, center point, class label, and position. It sets the color of the rectangle and the label based on the class label. It draws a circle around the center point, a rectangle around the object, and a label with the class name and position of the object. Finally, it returns the modified frame with the objects detected.

3. `get_command`

```python
class DarknetDNN:
    def get_command(self):
        #for box, class_id, contours in zip():
        if not self.object_area:
            return 'Hold'
        else:
            return self.object_positions[self.object_area.index(max(self.object_area))]
```

This method returns the position of the largest object detected in the frame, which represents the command for the vehicle to execute.

If there are no objects detected in the frame, it returns 'Hold' indicating that the drone should maintain its current position.

Otherwise, it finds the index of the largest object in `self.object_area` list and returns the corresponding value in `self.object_positions` list, which represents the position of the object. The largest object means that the object that we will follow is the nearest from the vehicle assumming that the size of the object is the same because the nearest object will appear larger in the camera while the opposite for the farthest object.

Currently, this method is used only when the human detectkion is pair up with ROS.
