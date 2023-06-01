'''
import cv2
import numpy as np

def detect_color(image, lower_color, upper_color):
    # Convert the image to the HSV color space
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Create a mask based on the specified color range
    mask = cv2.inRange(hsv, lower_color, upper_color)

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Initialize an empty list to store bounding boxes
    bounding_boxes = []
    bounding_box = image

    # Loop over the contours and create bounding boxes
    for contour in contours:
        # Compute the bounding box for the contour
        x, y, w, h = cv2.boundingRect(contour)

        # Create a bounding rectangle
        bounding_box = cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # Add the bounding box coordinates to the list
        bounding_boxes.append((x, y, x + w, y + h))

    # Return the image with bounding boxes and the list of bounding boxes
    return bounding_box, bounding_boxes

# Load the image
image = cv2.imread("D:\\Nakayama Iron Works\\GPS-Tracking\\MSD700_Follow_Me\\img\\safety_vest_4.png")

# Define the lower and upper color range (in HSV)
lower_color = np.array([0, 140, 185])
upper_color = np.array([30, 255, 255])

# Detect the color and draw bounding rectangles
output_image, bounding_boxes = detect_color(image, lower_color, upper_color)

# Display the image with bounding rectangles
cv2.imshow("Color Detection", output_image)
cv2.waitKey(0)
cv2.destroyAllWindows()
'''

''' 
import cv2
import numpy as np

def detect_color(image, lower_color, upper_color, confidence_threshold=0.5, nms_threshold=0.3):
    # Convert the image to the HSV color space
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Create a mask based on the specified color range
    mask = cv2.inRange(hsv, lower_color, upper_color)

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Initialize lists to store bounding boxes and confidences
    bounding_boxes = []
    confidences = []

    # Loop over the contours and create bounding boxes
    for contour in contours:
        # Compute the bounding box for the contour
        x, y, w, h = cv2.boundingRect(contour)

        # Add the bounding box coordinates and confidence (area) to the lists
        bounding_boxes.append([x, y, x + w, y + h])
        confidences.append(cv2.contourArea(contour))

    # Apply non-maximum suppression to eliminate overlapping bounding boxes
    indices = cv2.dnn.NMSBoxes(bounding_boxes, confidences, confidence_threshold, nms_threshold)

    # Initialize an empty list for the filtered bounding boxes
    filtered_boxes = []

    # Check if there are any indices left after NMS
    if len(indices) > 0:
        # Loop over the indices and add the filtered bounding boxes to the list
        for i in indices.flatten():
            filtered_boxes.append(bounding_boxes[i])

    # Draw the filtered bounding boxes on the image
    for box in filtered_boxes:
        x, y, x2, y2 = box
        cv2.rectangle(image, (x, y), (x2, y2), (0, 255, 0), 2)

    # Return the image with bounding boxes and the list of filtered bounding boxes
    return image, filtered_boxes

# Load the image
image3 = cv2.imread("D:\\Nakayama Iron Works\\GPS-Tracking\\MSD700_Follow_Me\\img\\safety_vest_3.png")
image5 = cv2.imread("D:\\Nakayama Iron Works\\GPS-Tracking\\MSD700_Follow_Me\\img\\safety_vest_5.png")
image6 = cv2.imread("D:\\Nakayama Iron Works\\GPS-Tracking\\MSD700_Follow_Me\\img\\safety_vest_6.png")

# Define the lower and upper color range (in HSV)
lower_color = np.array([0, 140, 185])
upper_color = np.array([30, 255, 255])

# Set the confidence threshold and NMS threshold
confidence_threshold = 0.1
nms_threshold = 0.1

# Detect the color, apply NMS, and draw bounding boxes
output_image3, bounding_boxes3 = detect_color(image3, lower_color, upper_color, confidence_threshold, nms_threshold)
output_image5, bounding_boxes5 = detect_color(image5, lower_color, upper_color, confidence_threshold, nms_threshold)
output_image6, bounding_boxes6 = detect_color(image6, lower_color, upper_color, confidence_threshold, nms_threshold)

# Display the image with bounding boxes
cv2.imshow("Color Detection3", output_image3)
cv2.imshow("Color Detection5", output_image5)
cv2.imshow("Color Detection6", output_image6)
cv2.waitKey(0)
cv2.destroyAllWindows()
'''

import cv2
import numpy as np

def detect_color(image, lower_color, upper_color, confidence_threshold=0.5, nms_threshold=0.3):
    # Convert the image to the HSV color space
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Create a mask based on the specified color range
    mask = cv2.inRange(hsv, lower_color, upper_color)

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Initialize lists to store bounding boxes and confidences
    bounding_boxes = []
    confidences = []

    # Loop over the contours and create bounding boxes
    for contour in contours:
        # Compute the bounding box for the contour
        x, y, w, h = cv2.boundingRect(contour)

        # Add the bounding box coordinates and confidence (area) to the lists
        bounding_boxes.append([x, y, x + w, y + h])
        confidences.append(cv2.contourArea(contour))

    # Apply non-maximum suppression to eliminate overlapping bounding boxes
    indices = cv2.dnn.NMSBoxes(bounding_boxes, confidences, confidence_threshold, nms_threshold)

    # Initialize an empty list for the filtered bounding boxes
    filtered_boxes = []

    # Check if there are any indices left after NMS
    if len(indices) > 0:
        # Loop over the indices and add the filtered bounding boxes to the list
        for i in indices.flatten():
            filtered_boxes.append(bounding_boxes[i])

    # Draw the filtered bounding boxes on the image
    for box in filtered_boxes:
        x, y, x2, y2 = box
        cv2.rectangle(image, (x, y), (x2, y2), (0, 255, 0), 2)

    # Return the image with bounding boxes and the list of filtered bounding boxes
    return image, filtered_boxes

# Open a video capture object
video_capture = cv2.VideoCapture("C:\\Users\\luthf\\Videos\\Captures\\safety_vest_video.mp4")  # Replace "video.mp4" with your video file

# Define the lower and upper color range (in HSV)
lower_color = np.array([0, 140, 185])
upper_color = np.array([30, 255, 255])

# Set the confidence threshold and NMS threshold
confidence_threshold = 0.5
nms_threshold = 0.1

# Process each frame in the video
while True:
    # Read a frame from the video
    ret, frame = video_capture.read()

    # Check if the frame was successfully read
    if not ret:
        break

    # Detect the color, apply NMS, and draw bounding boxes
    output_frame, bounding_boxes = detect_color(frame, lower_color, upper_color, confidence_threshold, nms_threshold)

    # Display the frame with bounding boxes
    cv2.imshow("Color Detection", output_frame)

    # Check for the 'q' key to quit the program
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video capture object and close any open windows
video_capture.release()
cv2.destroyAllWindows()
