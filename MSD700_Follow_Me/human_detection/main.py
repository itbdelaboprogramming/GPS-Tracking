import os
import argparse
import cv2
import numpy as np
from config.definition import ROOT_DIR

dnn_model = os.path.join(ROOT_DIR, 'weights/yolov3-tiny.weights')
#print(dnn_model)

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

weight_path = os.path.join(ROOT_DIR, 'weights/yolov3-tiny.weights')
cfg_path = os.path.join(ROOT_DIR, 'cfg/yolov3-tiny.cfg')
print("Loading model from ", weight_path, "and", cfg_path)
net = cv2.dnn.readNet(weight_path, cfg_path)
confidence_threshold = 0.2
nms_threshold = 0.5

#load video from camera
device_id = args.camera
cap = cv2.VideoCapture(device_id)
#tick_frequency = cv2.getTickFrequency()
start_time = cv2.getTickCount()
frame_count = 0
fps = 0

#blob parameters
blob_scalefactor = 1/255
blob_size = (320, 320)
blob_scalar = (0, 0, 0)
blob_swapRB = True
blob_crop = False

while True:
    ret, frame = cap.read()

    height, width, channels = frame.shape

    blob = cv2.dnn.blobFromImage(frame, blob_scalefactor, blob_size, blob_scalar, blob_swapRB, blob_crop)
    net.setInput(blob)

    output = net.forward()

    boxes = []
    confidences = []
    class_ids = []

    for detection in output:
        scores = detection[5:]
        class_ids = np.argmax(scores)
        confidence = scores[class_ids]

        if class_ids == 0 and confidence > confidence_threshold:
            box = detection[0:4] * np.array([width, height, width, height])
            x_center, y_center, w, h = box.astype('int')
            x_min, y_min = int(x_center - (w/2)), int(y_center - (h/2))

            boxes.append([x_min, y_min, int(w), int(h)])
            confidences.append(float(confidence))
            print("get")
    
    indices = cv2.dnn.NMSBoxes(boxes, confidences, confidence_threshold, nms_threshold)

    for i in indices:
        i = i[0]

        x, y, w, h = boxes[i]
        confidence = confidences[i]

        cv2.rectangle(frame, (x, y), (x+2, y+h), (0,255,0), 2)
        label = f"Person: {confidence}"
        cv2.putText(frame, label, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
    
    #FPS Calc
    frame_count+=1
    current_time = cv2.getTickCount()
    elapsed_time = (current_time - start_time) /  cv2.getTickFrequency()
    if elapsed_time > 1.0:
        fps = round(frame_count / elapsed_time, 2)
        #print(fps)
        start_time = current_time
        frame_count = 0
    
    cv2.putText(frame, "FPS: " + str(fps), (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)

    cv2.imshow("Video Stream", frame)
    key = cv2.waitKey(1)
    if key == 27:
        print("Stream ended, key", key, "is pressed")
        break

cap.release()
cv2.destroyAllWindows()