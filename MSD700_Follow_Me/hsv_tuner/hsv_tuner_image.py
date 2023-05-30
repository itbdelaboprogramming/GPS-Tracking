import cv2
import numpy as np

# Define initial lower and upper color values
lower_color = np.array([0, 140, 185])
upper_color = np.array([30, 255, 255])

'''
Note :
For green vest the range are
H : 25-140      30-60           27-50
S : 6-200       100-230         80-255
V : 80-250      125-250         125-235

For orange vest the range are
H : 0-30        5-20            9-12        -> 0-30
S : 165-230     140-235         173-255     -> 140-255
V : 200-250     185-255         242-255     -> 185-255
'''

# Create a callback function for the trackbar
def trackbar_callback(value):
    pass

# Create a window to display the camera frame
cv2.namedWindow("Camera Frame")

# Create trackbars for adjusting the color range
cv2.createTrackbar("Hue Min", "Camera Frame", lower_color[0], 179, trackbar_callback)
cv2.createTrackbar("Hue Max", "Camera Frame", upper_color[0], 179, trackbar_callback)
cv2.createTrackbar("Sat Min", "Camera Frame", lower_color[1], 255, trackbar_callback)
cv2.createTrackbar("Sat Max", "Camera Frame", upper_color[1], 255, trackbar_callback)
cv2.createTrackbar("Val Min", "Camera Frame", lower_color[2], 255, trackbar_callback)
cv2.createTrackbar("Val Max", "Camera Frame", upper_color[2], 255, trackbar_callback)

# Create a VideoCapture object to read from the camera (0 indicates the default camera)
cap = cv2.VideoCapture(0)

while True:
    # Read a frame from the camera
    #ret, frame = cap.read()
    frame = cv2.imread("D:\\Nakayama Iron Works\\GPS-Tracking\\MSD700_Follow_Me\\img\\safety_vest_5.png")

    # Convert the frame to HSV color space
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Get the current trackbar positions
    lower_color[0] = cv2.getTrackbarPos("Hue Min", "Camera Frame")
    upper_color[0] = cv2.getTrackbarPos("Hue Max", "Camera Frame")
    lower_color[1] = cv2.getTrackbarPos("Sat Min", "Camera Frame")
    upper_color[1] = cv2.getTrackbarPos("Sat Max", "Camera Frame")
    lower_color[2] = cv2.getTrackbarPos("Val Min", "Camera Frame")
    upper_color[2] = cv2.getTrackbarPos("Val Max", "Camera Frame")

    # Threshold the frame to obtain only the desired color range
    mask = cv2.inRange(hsv_frame, lower_color, upper_color)

    # Apply morphological operations to remove noise
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # Bitwise-AND the mask with the original frame
    filtered_frame = cv2.bitwise_and(frame, frame, mask=mask)

    # Display the camera frame and filtered frame
    cv2.imshow("Camera Frame", np.hstack((frame, filtered_frame)))

    # Exit loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the VideoCapture object and close any open windows
cap.release()
cv2.destroyAllWindows()
