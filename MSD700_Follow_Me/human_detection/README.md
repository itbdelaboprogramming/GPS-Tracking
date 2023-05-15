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

1. cfg and weights

These folders contain the configuration and weights of the model for [Darknet-YOLO](https://pjreddie.com/darknet/yolo/) framework.

2. coco.names

The YOLO framework not only can detect human but all sort of things. This file contain the label for those objects. But in the code we will use only `Person`.

3. darknet_yolo.py

This file contain the class for handling the Neural Network for detecting human. The code use [Darknet](https://pjreddie.com/darknet/yolo/) framework for the model of the Deep Neural Network. For more explanation read below.

4. device_camera.py

This file contain the class for handling camera, frame and fps. For more explanation read below.

5. follow_me.py

This file is the main script for human detection. Run this script to open the camera and detect the human in the camera frame.

### Main Script

The main script to run the human detector is `follow_me.py`. The file follows this flowchart.

![Human_detection_flowchart](/MSD700_Follow_Me/human_detection/img/Flowchart%20human-detection.png?raw=true "Human detection flowchart")


