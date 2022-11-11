import cv2
import numpy as np
import time

dist = lambda x1,y1,x2,y2: (x1-x2)**2+(y1-y2)**2
prevCircle = None

def check_circle(roi):
    global prevCircle
    if 0 in np.shape(roi):
        return False
    else:
        print(np.shape(roi))
        
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

        # Blur using 3 * 3 kernel.
        gray_blurred = cv2.blur(gray, (4, 4))

        # Apply Hough transform on the blurred image.
        detected_circles = cv2.HoughCircles(gray_blurred,
                                            cv2.HOUGH_GRADIENT, 1, 20, param1=50,
                                            param2=30, minRadius=1, maxRadius=40)

        if detected_circles is not None:

            detected_circles = np.uint16(np.around(detected_circles))
            chosen = None

            for circle in detected_circles[0, :]:
                a, b, r = circle[0], circle[1], circle[2]
                if chosen is None: chosen = circle
                if prevCircle is not None:
                    if dist(chosen[0],chosen[1],prevCircle[0],prevCircle[1]) <= dist(circle[0],circle[1],prevCircle[0],prevCircle[1]):
                        chosen = circle
            
            cv2.circle(roi, (chosen[0], chosen[1]), chosen[2], (0, 255, 0), 2)
            
            prevCircle = chosen
            return True
        else:
            return False

def get_centered_contours(mask):
  # find contours
    cntrs = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cntrs = cntrs[0] if len(cntrs) == 2 else cntrs[1]
    sorted_contours = sorted(cntrs, key=cv2.contourArea, reverse=True)
    filterd_contours = []
    if sorted_contours != []:
        for k in range(len(sorted_contours)):
            if cv2.contourArea(sorted_contours[k]) < 1000.0:
                filterd_contours = sorted_contours[0:k]
                return filterd_contours
    return filterd_contours


def check_red_colour(roi):
    if np.shape(roi)[1] != 0:
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        # define range of blue color in HSV
        lower_red = np.array([0, 50, 50])
        upper_red = np.array([10, 255, 255])
        # Threshold the HSV image to get only blue colors
        mask = cv2.inRange(hsv, lower_red, upper_red)
        cnts = get_centered_contours(mask)
        if cnts != []:
            return True
        else:
            return False

# Load Yolo
net = cv2.dnn.readNet("weights/yolov3-tiny.weights", "cfg/yolov3-tiny.cfg")
classes = []

# Load coco
with open("lib/coco.names", "r") as f:
    classes = [line.strip() for line in f.readlines()]
layer_names = net.getLayerNames()
output_layers = [layer_names[i - 1] for i in net.getUnconnectedOutLayers()]
colors = np.random.uniform(0, 255, size=(len(classes), 3))

# Loading video
cap = cv2.VideoCapture("vidoes/kid_street_biking.mp4")

font = cv2.FONT_HERSHEY_PLAIN
starting_time = time.time()
frame_id = 0
while True:
    _, frame = cap.read()
    frame_id += 1

    height, width, channels = frame.shape

    # Detecting objects
    blob = cv2.dnn.blobFromImage(frame, 0.00392, (416, 416), (0, 0, 0), True, crop=False)

    net.setInput(blob)
    outs = net.forward(output_layers)

    # Showing informations on the screen
    class_ids = []
    confidences = []
    boxes = []
    for out in outs:
        for detection in out:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]
            if confidence > 0.2:
                # Object detected
                center_x = int(detection[0] * width)
                center_y = int(detection[1] * height)
                w = int(detection[3] * width)
                h = int(detection[3] * height)

                # Rectangle coordinates
                x = int(center_x - w / 1.8)
                y = int(center_y - h / 1.8)

                roi = frame[y:y+h, x:x+w]
                if check_circle(roi):
                    boxes.append([x, y, w, h])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)

    indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.4, 0.3)

    for i in range(len(boxes)):
        if i in indexes:
            x, y, w, h = boxes[i]
            label = str(classes[class_ids[i]])
            confidence = confidences[i]
            color = colors[class_ids[i]]
            cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
            cv2.putText(frame, label + " " + str(round(confidence, 2)), (x, y + 30), font, 2, color, 2)



    elapsed_time = time.time() - starting_time
    fps = frame_id / elapsed_time
    cv2.putText(frame, "FPS: " + str(round(fps, 2)), (10, 50), font, 2, (0, 0, 0), 3)
    cv2.imshow("YOLO Realtime Woman Detection", frame)
    key = cv2.waitKey(1)
    if key == 27:
        break

cap.release()
cv2.destroyAllWindows()