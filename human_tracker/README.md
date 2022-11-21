# Human Tracker

## HOG Method
Using histogram representation of an image (extracting features from image data) and use neural network for object identification and classification. Run the script below to run the program,

```sh
python3 hog.py
```

## YOLO Method
implements a real time human detection via video or webcam detection using yolov3-tiny algorithm. YOLO is an object detection algorithm which stand for You Only Look Once. Implemented using pre-trained weights. Run the script below to run the program,

```sh
python3 yolo.py
```

to change input from webcam to video,

```python
# replace this line
cap = cv2.VideoCapture(0)

# into this one (*** is the name of the video file)
cap = cv2.VideoCapture("videos/***")
```

## YOLO Method with circle and color detection
Combined the regular yolo method and filter the result to only show object that contain a certain color or circle. un the script below to run the program,

```sh
python3 yolo_circle.py
```