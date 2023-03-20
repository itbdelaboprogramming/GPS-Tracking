import cv2
import numpy as np
import time

# Load Yolo using cv2 Deep Neural Network module
net = cv2.dnn.readNet("weights/yolov3-tiny.weights", "cfg/yolov3-tiny-test.cfg")
classes = []

# Load coco using cv2 Deep Neural Network module
with open("lib/coco_test.names", "r") as f:
    classes = [line.strip() for line in f.readlines()]
layer_names = net.getLayerNames()
output_layers = [layer_names[i - 1] for i in net.getUnconnectedOutLayers()]

# generate random color for each detected object type 3*(0-255 range of color)
colors = np.random.uniform(0, 255, size=(len(classes), 3))

# Loading video input
cap = cv2.VideoCapture(0)

font = cv2.FONT_HERSHEY_PLAIN
starting_time = time.time()
frame_id = 0

# main loop
while True:
    _, frame = cap.read()
    frame_id += 1

    height, width, channels = frame.shape

    # Detecting objects
    # blobFromImage(image[, scalefactor[, size[, mean[, swapRB[, crop[, ddepth]]]]]]) 
    # create a 4D blob from a frame (scalefactor 1/255 scaling image pixels to 0-1, size provided by YOLO)
    blob = cv2.dnn.blobFromImage(
        frame, 1/18, (416, 416), (0, 0, 0), True, crop=False)

    net.setInput(blob)

    # outs: nested list containing centre coordinates, width and height, confidence and scores 
    outs = net.forward(output_layers)

    # Showing informations on the screen
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

                # Registering the bounding boxes
                boxes.append([x, y, w, h])
                confidences.append(float(confidence))
                class_ids.append(class_id)

    # removes redundant overlapping bounding boxes
    indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.4, 0.3)

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

    # frame calculation
    elapsed_time = time.time() - starting_time
    fps = frame_id / elapsed_time
    cv2.putText(frame, "FPS: " + str(round(fps, 2)),
                (10, 50), font, 2, (0, 0, 0), 3)

    # showed the window
    cv2.imshow("test", frame)
    key = cv2.waitKey(1)
    if key == 27:
        break

cap.release()
cv2.destroyAllWindows()