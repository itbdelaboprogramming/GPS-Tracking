# https://pysource.com/instance-segmentation-mask-rcnn-with-python-and-opencv
import cv2
import time
import numpy as np
import pyrealsense2 as rs



# Loading Mask RCNN
#net = cv2.dnn.readNetFromTensorflow("dnn/frozen_inference_graph_coco.pb",
#                                            "dnn/mask_rcnn_inception_v2_coco_2018_01_28.pbtxt")
net = cv2.dnn.readNet("weights/yolov3-tiny.weights", "cfg/yolov3-tiny.cfg")
net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)

# Generate random colors
np.random.seed(2)
colors = np.random.randint(0, 255, (90, 3))

# Conf threshold
detection_threshold = 0.7
mask_threshold = 0.3

classes = []
with open("lib/coco.names", "r") as file_object:
        for class_name in file_object.readlines():
                class_name = class_name.strip()
                classes.append(class_name)

obj_boxes = []
obj_classes = []
obj_centers = []
obj_contours = []

        # Distances
distances = []


def detect_objects_mask(bgr_frame):
        blob = cv2.dnn.blobFromImage(bgr_frame, swapRB=True)
        net.setInput(blob)

        boxes, masks = net.forward(["detection_out_final", "detection_masks"])

        # Detect objects
        frame_height, frame_width, _ = bgr_frame.shape
        detection_count = boxes.shape[2]

        # Object Boxes
        obj_boxes = []
        obj_classes = []
        obj_centers = []
        obj_contours = []

        for i in range(detection_count):
            box = boxes[0, 0, i]
            class_id = box[1]
            score = box[2]
            color = colors[int(class_id)]
            if score < detection_threshold:
                continue

            if class_id == 0: # To filter what object you want to detect
                # Get box Coordinates
                x = int(box[3] * frame_width)
                y = int(box[4] * frame_height)
                x2 = int(box[5] * frame_width)
                y2 = int(box[6] * frame_height)
                obj_boxes.append([x, y, x2, y2])

                cx = (x + x2) // 2
                cy = (y + y2) // 2
                obj_centers.append((cx, cy))

                # append class
                obj_classes.append(class_id)

                # Contours
                # Get mask coordinates
                # Get the mask
                mask = masks[i, int(class_id)]
                roi_height, roi_width = y2 - y, x2 - x
                mask = cv2.resize(mask, (roi_width, roi_height))
                _, mask = cv2.threshold(mask, mask_threshold, 255, cv2.THRESH_BINARY)
                contours, _ = cv2.findContours(np.array(mask, np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                obj_contours.append(contours)

        return obj_boxes, obj_classes, obj_contours, obj_centers

def draw_object_mask(bgr_frame):
        # loop through the detection
        for box, class_id, contours in zip(obj_boxes, obj_classes, obj_contours):
            #if class_id == 0: # To filter what object you want to show
                x, y, x2, y2 = box
                roi = bgr_frame[y: y2, x: x2]
                roi_height, roi_width, _ = roi.shape
                color = colors[int(class_id)]

                roi_copy = np.zeros_like(roi)

                for cnt in contours:
                    # cv2.f(roi, [cnt], (int(color[0]), int(color[1]), int(color[2])))
                    cv2.drawContours(roi, [cnt], - 1, (int(color[0]), int(color[1]), int(color[2])), 3)
                    cv2.fillPoly(roi_copy, [cnt], (int(color[0]), int(color[1]), int(color[2])))
                    roi = cv2.addWeighted(roi, 1, roi_copy, 0.5, 0.0)
                    bgr_frame[y: y2, x: x2] = roi
        return bgr_frame

def draw_object_info(bgr_frame, depth_frame):
        # loop through the detection
        for box, class_id, obj_center in zip(obj_boxes, obj_classes, obj_centers):
            #if class_id == 0: # To filter what object you want to show
                x, y, x2, y2 = box

                color = colors[int(class_id)]
                color = (int(color[0]), int(color[1]), int(color[2]))

                cx, cy = obj_center

                depth_mm = depth_frame[cy, cx]

                cv2.line(bgr_frame, (cx, y), (cx, y2), color, 1)
                cv2.line(bgr_frame, (x, cy), (x2, cy), color, 1)

                class_name = classes[int(class_id)]
                cv2.rectangle(bgr_frame, (x, y), (x + 250, y + 70), color, -1)
                cv2.putText(bgr_frame, class_name.capitalize(), (x + 5, y + 25), 0, 0.8, (255, 255, 255), 2)
                cv2.putText(bgr_frame, "{} cm".format(depth_mm / 10), (x + 5, y + 60), 0, 1.0, (255, 255, 255), 2)
                cv2.rectangle(bgr_frame, (x, y), (x2, y2), color, 1)

        return bgr_frame


# Configure depth and color streams
print("Loading Intel Realsense Camera")
pipeline = rs.pipeline()

config = rs.config()
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)

# Start streaming
pipeline.start(config)
align_to = rs.stream.color
align = rs.align(align_to)


def get_frame_stream():
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        # frame_number = frames.get_frame_number()
        # frame_timestampt = frames.get_timestamp()
        # print(f'frame number is {frame_number}')
        # print(f'frame time is {frame_timestampt}')
        
        if not depth_frame or not color_frame:
            # If there is no frame, probably camera not connected, return False
            print("Error, impossible to get the frame, make sure that the Intel Realsense camera is correctly connected")
            return False, None, None
        
        # Apply filter to fill the Holes in the depth image
        spatial = rs.spatial_filter()
        spatial.set_option(rs.option.holes_fill, 3)
        filtered_depth = spatial.process(depth_frame)

        hole_filling = rs.hole_filling_filter()
        filled_depth = hole_filling.process(filtered_depth)

        
        # Create colormap to show the depth of the Objects
        colorizer = rs.colorizer()
        depth_colormap = np.asanyarray(colorizer.colorize(filled_depth).get_data())

        
        # Convert images to numpy arrays
        # distance = depth_frame.get_distance(int(50),int(50))
        # print("distance", distance)
        depth_image = np.asanyarray(filled_depth.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # cv2.imshow("Colormap", depth_colormap)
        # cv2.imshow("depth img", depth_image)

        return True, color_image, depth_image
    
def release():
        pipeline.stop()
        #print(depth_image)
        
        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        #depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.10), 2)

        # Stack both images horizontally
        
        #images = np.hstack((color_image, depth_colormap))

# Load Realsense camera

font = cv2.FONT_HERSHEY_PLAIN
starting_time = time.time()
frame_id = 1

while True:
    # Get frame in real time from Realsense camera
    ret, bgr_frame, depth_frame = get_frame_stream()
    # frame_id += 1

    # Get object mask
    boxes, classes, contours, centers = detect_objects_mask(bgr_frame)

    # Draw object mask
    bgr_frame = draw_object_mask(bgr_frame)

    # Show depth info of the objects
    draw_object_info(bgr_frame, depth_frame)

    current_time = time.time()
    elapsed_time = current_time - starting_time
    fps = frame_id / elapsed_time
    print(fps)
    cv2.putText(bgr_frame, "FPS: " + str(round(fps, 2)),
                (10, 50), font, 2, (0, 0, 0), 3)

    
    cv2.imshow("depth frame", depth_frame)
    cv2.imshow("Bgr frame", bgr_frame)

    starting_time = current_time

    key = cv2.waitKey(1)
    if key == 27:
        print(f"Key {key} is pressed. Proceed to exit")
        break

release()
cv2.destroyAllWindows()
