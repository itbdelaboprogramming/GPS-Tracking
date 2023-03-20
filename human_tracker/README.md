# YOLO

YOLO is an abbreviation for the term ‘You Only Look Once’. This is an algorithm that detects and recognizes various objects in a picture (in real-time). Object detection in YOLO is done as a regression problem and provides the class probabilities of the detected images.

YOLO algorithm employs convolutional neural networks (CNN) to detect objects in real-time. As the name suggests, the algorithm requires only a single forward propagation through a neural network to detect objects.

This means that prediction in the entire image is done in a single algorithm run. The CNN is used to predict various class probabilities and bounding boxes simultaneously.

The YOLO algorithm consists of various variants. Some of the common ones include tiny YOLOv3 and YOLOv3.

## Technical Implementation

To implement YOLO algorithm we use OpenCV's DNN module and YOLO pre-trained configuration and weight because of YOLO's easy integration with OpenCV program and the robustness of OpenCV's DNN module.

### YOLO Implementation

#### Loading the model, classes, and other preparation

The first thing to do is loading the YOLO model and classes to use for our model. We use `cv2.dnn.readNet()` to load our YOLO model with pre-trained configuration and weight. For the classes we use `coco.names` from coco dataset that contains 80 class. 

Other preparation that we need to do is generating random color for each of object class listed in coco.names . In this section we also load our video input using `cv2.VideoCapture()`, insert integer as the parameter to use camera input or video's path to use video input. We use `time` module to mark the program starting time and using it to calculate fps.

Loading the model, classes, and other preparation,
```python
# Load Yolo using cv2 Deep Neural Network module
net = cv2.dnn.readNet("weights/yolov3-tiny.weights", "cfg/yolov3-tiny.cfg")
classes = []

# Load coco using cv2 Deep Neural Network module
with open("lib/coco.names", "r") as f:
    classes = [line.strip() for line in f.readlines()]
layer_names = net.getLayerNames()
output_layers = [layer_names[i - 1] for i in net.getUnconnectedOutLayers()]

# generate random color for each detected object type
colors = np.random.uniform(0, 255, size=(len(classes), 3))

# Loading video input
cap = cv2.VideoCapture(0)

font = cv2.FONT_HERSHEY_PLAIN
starting_time = time.time()
frame_id = 0
```

#### Processing each frame

The input image to a neural network needs to be in a certain format called a blob. After a frame is read from the input image or video stream, it is passed through the `cv2.dnn.blobFromImage()` function to convert it to an input blob for the neural network. In this process, it scales the image pixel values to a target range of 0 to 1 using a scale factor of 1/255 (0.00392). It also resizes the image to the given size of (416, 416) without cropping. 

The output blob is then passed into the network as its input, and a forward pass is run to get a list of predicted bounding boxes as the network’s output.

Process each frame,
```python
# main loop
while True:
    _, frame = cap.read()
    frame_id += 1

    height, width, channels = frame.shape

    blob = cv2.dnn.blobFromImage(
        frame, 0.00392, (416, 416), (0, 0, 0), True, crop=False)

    net.setInput(blob)

    # outs: nested list containing centre coordinates, width and height, confidence and scores 
    outs = net.forward(output_layers)
```

#### Post-processing the output

The network outputs represent 5 elements + confidence of each class. first 4 elements represent the center_x, center_y, width, and height. The fifth element represents the confidence that the bounding box encloses an object.

The rest of the elements are the confidence associated with each class (i.e., object type). The box is assigned to the class corresponding to the highest score for the box. To get the class with highest score we use `np.argmax()`.

The highest score for a box is also called its confidence. If the confidence of a box is less than the given threshold (0.2), the bounding box is dropped and not considered for further processing.

The boxes with confidence equal to or greater than the confidence threshold are then subjected to Non Maximum Suppression. This would reduce the number of overlapping boxes. In this program we use DNN's implementation of NMS `cv2.dnn.NMSBoxes()`.

Post-processing the output,
```python
class_ids = []
confidences = []
boxes = []
for out in outs:
    for detection in out:
        scores = detection[5:]

        # the predicted class of an object is the class with highest score
        class_id = np.argmax(scores)
        confidence = scores[class_id]

        # gives a valid pass for an object with 20% or more confidence score
        # raise 0.2 for more stringent detection, lower for the other way around
        if confidence > 0.2:
            # Object detected
            center_x = int(detection[0] * width)
            center_y = int(detection[1] * height)
            w = int(detection[3] * width)
            h = int(detection[3] * height)

            # Rectangle coordinates 
            # (1.8 correcting factor to the width and height) lower to get smaller boundary box
            x = int(center_x - w / 1.8)
            y = int(center_y - h / 1.8)

            # cropped frame using the coordinates of the rectangle surrounding an object
            # roi = frame[y:y+h, x:x+w]

            # if check_circle(roi):
            boxes.append([x, y, w, h])
            confidences.append(float(confidence))
            class_ids.append(class_id)

# removes redundant overlapping bounding boxes
indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.4, 0.3)
```

#### Drawing the boxes

In this section we already got the properties of each detected object's boundary box (coordinates, width, height). Now what we need to do is to show or visualize those boxes in the given frame. To do that first we register each box properties and the corresponding class type, confidence score, and color. To draw the rectangle itself we use `cv2.rectangle()` with each box properties as the parameter. We also label each boxes with class type and confidence score using `cv2.putText()`.

Drawing the boxes,
```python
# draw the registered rectangle
for i in range(len(boxes)):
    if i in indexes:
        x, y, w, h = boxes[i]
        label = str(classes[class_ids[i]])
        confidence = confidences[i]
        color = colors[class_ids[i]]
        cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)

        # putText(img, text, org, fontFace, fontScale, color, thickness)
        cv2.putText(frame, label + " " + str(round(confidence, 2)),
                    (x, y + 30), font, 2, color, 2)
```

### The Filters

#### Human filter

For the human filter what we do is simple. We just filter out the class that we consider valid for further processing. We do that by adding `class_id == 0` in the if statement at post processing section.

```python
class_ids = []
confidences = []
boxes = []
for out in outs:
    for detection in out:
        scores = detection[5:]

        # the predicted class of an object is the class with highest score
        class_id = np.argmax(scores)
        confidence = scores[class_id]

        # gives a valid pass for an object with 20% or more confidence score
        # raise 0.2 for more stringent detection, lower for the other way around
        if confidence > 0.2 and class_id == 0: # class_id 0 is human
            # Object detected
            center_x = int(detection[0] * width)
            center_y = int(detection[1] * height)
            w = int(detection[3] * width)
            h = int(detection[3] * height)

            # Rectangle coordinates 
            # (1.8 correcting factor to the width and height) lower to get smaller boundary box
            x = int(center_x - w / 1.8)
            y = int(center_y - h / 1.8)

            # cropped frame using the coordinates of the rectangle surrounding an object
            # roi = frame[y:y+h, x:x+w]

            # if check_circle(roi):
            boxes.append([x, y, w, h])
            confidences.append(float(confidence))
            class_ids.append(class_id)

# removes redundant overlapping bounding boxes
indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.4, 0.3)
```

#### Color filter

Color filtering is widely used in OpenCV for identifying specific objects/regions having a specific color. Regular camera usually use RGB color space. However, filtering/thresholding color in RGB colorspace is hard because of the way it represents color. 

In this program we use HSV colorspace for color filtering/thresholding over RGB because HSV is more robust towards external lighting changes. This means that in cases of minor changes in external lighting (such as pale shadows, etc). Hue values vary relatively lesser than RGB values. Hence, setting the ranges of color in HSV space is more robust. We convert the colorspace of an image using `cv2.cvtColor()` and threshold the color with `cv2.inRange()`.

First we threshold the image to only show certain colors, then we look for [contour](https://docs.opencv.org/3.4/d4/d73/tutorial_py_contours_begin.html) by using `cv2.findContours()` and if the area of the contour is big enough (not some random particle), we register it as an object with certain color.

color checker,
```python
# check_colour : used to check if there is a certain color within a certain frame of video
# output : true if there is a certain color detected and if not return false
def check_colour(roi):
    if 0 in np.shape(roi):
        return False
    else:
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
        # define range of color in HSV
        lower_hsv = np.array([0, 50, 50])
        upper_hsv = np.array([10, 255, 255])

        # Threshold the HSV image to get only a certain color
        mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
        cnts = get_centered_contours(mask)
        if cnts != []:
            return True
        else:
            return False
```
contour getter,
```python
# get_centered_contours: get the contours of a masked image
# output : filterd_contours
def get_centered_contours(mask):
    cntrs = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cntrs = cntrs[0] if len(cntrs) == 2 else cntrs[1]

    # sort countour by its area (largest to smallest)
    sorted_contours = sorted(cntrs, key=cv2.contourArea, reverse=True)
    filterd_contours = []

    if sorted_contours != []:

        # return countours with area bigger than 1000
        # raise or lower 1000 to change the minimum threshold for "valid" countour
        # raise to avoid random countour
        for k in range(len(sorted_contours)):
            if cv2.contourArea(sorted_contours[k]) < 1000.0:
                filterd_contours = sorted_contours[0:k]
                return filterd_contours

    return filterd_contours
```

#### Circle filter

The circle filter used in this program is using [Hough Circle Transform](https://docs.opencv.org/3.4/d4/d70/tutorial_hough_circle.html) to detect circles within an image. Basically this method split into two stages. The first stage involves edge detection and finding the possible circle centers and the second stage finds the best radius for each candidate center. But we can automate those process using `cv2.HoughCircles()`.

The first step is to convert our image to grayscale to avoid shape misreading because of color difference. Then, we smooth all the edges within our image by bluring the image. After that, we apply the Hough Circle Transform to return a list of detected circles.

Using the list of all detected circles, we iterate through all the circle and register a circle as the chosen circle (the one we are going to draw) if the distance between the circle in current state is close to the chosen circle from the previous state. After we get the chosen circle, we draw the circle using ` cv2.circle()`.


circle filter,
```python
# check_circle : used to tell either there's a circle or not within  a frame of a video
# output: return true if the circle exist return false if circle doesn't exist
def check_circle(roi):
    global prevCircle

    # precaution if roi is a frame with 0 width or 0 height (either will make the cvtColor catch
    # an error)
    if 0 in np.shape(roi):
        return False
    else:
        print(np.shape(roi))
        
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

        # blur to smooth rough edges, lower second param to reduce the smoothing edges 
        gray_blurred = cv2.blur(gray, (4, 4))

        # Apply Hough transform on the blurred image.
        detected_circles = cv2.HoughCircles(gray_blurred,
                                            cv2.HOUGH_GRADIENT, 1, 20, param1=50,
                                            param2=30, minRadius=10, maxRadius=40)

        # Cycle through all detected circle and draw the circle
        if detected_circles is not None:

            detected_circles = np.uint16(np.around(detected_circles))
            chosen = None

            # the chosen circle to be registered and drawed is the one closest in distance according
            # to the previous state, hence reducing the generation of random circle
            for circle in detected_circles[0, :]:
                a, b, r = circle[0], circle[1], circle[2]
                if chosen is None: chosen = circle
                if prevCircle is not None:
                    if dist(chosen[0],chosen[1],prevCircle[0],prevCircle[1]) <= dist(circle[0],circle[1],prevCircle[0],prevCircle[1]):
                        chosen = circle
            
            # draw the circle
            cv2.circle(roi, (chosen[0], chosen[1]), chosen[2], (0, 255, 0), 2)
            
            prevCircle = chosen
            return True
        else:
            return False
```

initiation for the dist function and circle from previous state at the start of the program,
```python
# initiation for check circle function
dist = lambda x1,y1,x2,y2: (x1-x2)**2+(y1-y2)**2
prevCircle = None
```

#### Direction checker

The follow me feature of MSD required the MSD to be able to follow a certain human based from the reading of the camera. To do that we need to know when an object going to the left, right, front, back, or idle

The direction checker divided into two. The first one is depth direction and the other is horizontal direction. The depth direction measure the difference of boundary box's area current and previous state. If the current state area is smaller than the previous state, the object is moving away from the MSD (front). If the current state area is bigger than the previous state, the object is moving towards the MSD (back). If the difference is within the tolerable threshold, it means the object is idle.

The horizontal direction check . We define an object is heading to the left or right if the object is on the left side or right side of the video input. The goal is to keep the object in the middle of the video input within the tolerable threshold. This means that the object is aligned with MSD's vision.

Direction checker,
```python
prevX = None
prevArea = None

# check_depth_direction: used to tell front/back direction of an object
def check_depth_direction(prevarea, curarea):
    if abs(curarea-prevarea) <= 300:
        return "idle"
    elif curarea-prevarea < -300:
        return "front"
    else:
        return "back"

# check_horizontal_direction: used to tell direction of an object
def check_horizontal_direction(curx, width):
    center = width/2
    if abs(curx-center) <= 15:
        return "idle"
    elif curx-center < -15:
        return "left"
    else:
        return "right"
```

#### Applying the filter

To apply the filter we need to edit the original YOLO program, specifically in the [Post Processing The Output](#post-processing-the-output) section. To apply human filter we need to change,
```python
        if confidence > 0.2:
            # Object detected
            center_x = int(detection[0] * width)
            center_y = int(detection[1] * height)
            w = int(detection[3] * width)
            h = int(detection[3] * height)
```
into,
```python
            if confidence > 0.2 and class_id == 0:
                # Object detected
                center_x = int(detection[0] * width)
                center_y = int(detection[1] * height)
                w = int(detection[3] * width)
                h = int(detection[3] * height)
```

Then to apply circle filter or color filter we need to change,
```python
            boxes.append([x, y, w, h])
            confidences.append(float(confidence))
            class_ids.append(class_id)
```
into,
```python
            # check_circle or check_color
            if check_circle(roi):
                boxes.append([x, y, w, h])
                confidences.append(float(confidence))
                class_ids.append(class_id)
```

To add direction checker in the YOLO program we need to calculate the area of every boundary box and using the coordinate given by each boundary box, we can feed it to the `check_horizontal_direction()` and `check_depth_direction()`,
```python
        if confidence > 0.2 and class_id == 0:
            # Object detected
            center_x = int(detection[0] * width)
            center_y = int(detection[1] * height)
            w = int(detection[3] * width)
            h = int(detection[3] * height)
            area = w * h

            # Rectangle coordinates 
            # (1.8 correcting factor to the width and height) lower to get smaller boundary box
            x = int(center_x - w / 1.8)
            y = int(center_y - h / 1.8)

            # initiating previous state area
            if prevArea == None:
                prevArea = area

            # initiating previous state center x coordinate
            if prevX == None:
                prevX = center_x
            
            direction = check_horizontal_direction(center_x, width)

            # direction = check_horizontal_direction_fixed(prevX, center_x)

            depth = check_depth_direction(prevArea, area)
```

The final post processing section should look like this,
```python
# Showing informations on the screen
class_ids = []
confidences = []
boxes = []
directions = []
depths = []

# cycle through each object outs
for out in outs:
    for detection in out:
        scores = detection[5:]

        # the predicted class of an object is the class with highest score
        class_id = np.argmax(scores)
        confidence = scores[class_id]

        # gives a valid pass for an object with 20% or more confidence score
        # raise 0.2 for more stringent detection, lower for the other way around
        # class_id = 0 for human detection
        if confidence > 0.2 and class_id == 0:
            # Object detected
            center_x = int(detection[0] * width)
            center_y = int(detection[1] * height)
            w = int(detection[3] * width)
            h = int(detection[3] * height)
            area = w * h

            # Rectangle coordinates 
            # (1.8 correcting factor to the width and height) lower to get smaller boundary box
            x = int(center_x - w / 1.8)
            y = int(center_y - h / 1.8)

            # initiating previous state area
            if prevArea == None:
                prevArea = area

            # initiating previous state center x coordinate
            if prevX == None:
                prevX = center_x
            
            direction = check_horizontal_direction(center_x, width)

            # direction = check_horizontal_direction_fixed(prevX, center_x)

            depth = check_depth_direction(prevArea, area)

            # cropped frame using the coordinates of the rectangle surrounding an object
            roi = frame[y:y+h, x:x+w]

            # feed the cropped image (roi) to check_circle / check_colour to make sure
            # only register the rectangle that passed the check_circle or check_colour
            if check_circle(roi):
                depths.append(depth)
                directions.append(direction)
                boxes.append([x, y, w, h])
                confidences.append(float(confidence))
                class_ids.append(class_id)

# removes redundant overlapping bounding boxes
indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.4, 0.3)
```

### HSV Tuner

To use the color filter mentioned in the previos section, we need to adjust the range of color that we consider valid. Doing that means we need to adjust the lower and upper boundary of the range and show the thresholded image so the user can further adjust the range.

The general structure of the program are, 

Capture the video stream from default or supplied capturing device,
```python
cap = cv.VideoCapture(args.camera)
```

Create a window to display the default frame and the threshold frame,
```python
cv.namedWindow(window_capture_name)
cv.namedWindow(window_detection_name)
```

Create the trackbars to set the range of HSV values,
```python
cv.createTrackbar(low_H_name, window_detection_name , low_H, max_value_H, on_low_H_thresh_trackbar)
cv.createTrackbar(high_H_name, window_detection_name , high_H, max_value_H, on_high_H_thresh_trackbar)
cv.createTrackbar(low_S_name, window_detection_name , low_S, max_value, on_low_S_thresh_trackbar)
cv.createTrackbar(high_S_name, window_detection_name , high_S, max_value, on_high_S_thresh_trackbar)
cv.createTrackbar(low_V_name, window_detection_name , low_V, max_value, on_low_V_thresh_trackbar)
cv.createTrackbar(high_V_name, window_detection_name , high_V, max_value, on_high_V_thresh_trackbar)
```

Until the user want the program to exit update the threshold frame,
```python
ret, frame = cap.read()
if frame is None:
    break
frame_HSV = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
frame_threshold = cv.inRange(frame_HSV, (low_H, low_S, low_V), (high_H, high_S, high_V))
```

Show the image,
```python
cv.imshow(window_capture_name, frame)
cv.imshow(window_detection_name, frame_threshold)
```

For trackbar which controls the lower range of Hue, Saturation, and Value. Say for example hue value,
```python
def on_low_H_thresh_trackbar(val):
    global low_H
    global high_H
    low_H = val
    low_H = min(high_H-1, low_H)
    cv.setTrackbarPos(low_H_name, window_detection_name, low_H)
```

For trackbar which controls the upper range of Hue, Saturation, and Value. Say for example hue value,
```python
def on_high_H_thresh_trackbar(val):
    global low_H
    global high_H
    high_H = val
    high_H = max(high_H, low_H+1)
    cv.setTrackbarPos(high_H_name, window_detection_name, high_H)
```

### Using YOLO as ROS node

To change the original program into ROS node we just need to initiate the node itself and add the publisher to a certain topic.

initiate the ROS node in the [preparation](#loading-the-model-classes-and-other-preparation) section,
```python
rospy.init_node("detector")
```

We also need to initiate the publisher in the [preparation](#loading-the-model-classes-and-other-preparation) section,
```python
# Publisher & Subscriber Definition
pub = rospy.Publisher("/detector/direction", Int8, queue_size = 1)
# sub = rospy.Subscriber("/ball_on_robot_6", Int8, br6Callback)
```

Then we convert our original main loop into ROS process,
```python
r = rospy.Rate(1000) # 1000 Hz
while not rospy.is_shutdown():
    _, frame = cap.read()
    frame_id += 1

    height, width, channels = frame.shape
    print(height, width)
```

What we actually publish is the direction data in the form of number that represent a direction. 0 as idle, 1 as left, 2 as right. We publish the data in the [Post Processing](#post-processing-the-output) section,
```python
for out in outs:
    for detection in out:
        scores = detection[5:]

        # the predicted class of an object is the class with highest score
        class_id = np.argmax(scores)
        confidence = scores[class_id]

        # gives a valid pass for an object with 20% or more confidence score
        # raise 0.2 for more stringent detection, lower for the other way around
        # class_id = 0 for human detection
        if confidence > 0.2 and class_id == 0:
            # Object detected
            center_x = int(detection[0] * width)
            center_y = int(detection[1] * height)
            w = int(detection[3] * width)
            h = int(detection[3] * height)
            area = w * h

            # Rectangle coordinates 
            # (1.8 correcting factor to the width and height) lower to get smaller boundary box
            x = int(center_x - w / 1.8)
            y = int(center_y - h / 1.8)

            # initiating previous state area
            if prevArea == None:
                prevArea = area

            # initiating previous state center x coordinate
            if prevX == None:
                prevX = center_x
            
            # direction of the detected object
            direction, direction_data = check_horizontal_direction(center_x, width)
            pub.publish(direction_data) # publish the direction data
```