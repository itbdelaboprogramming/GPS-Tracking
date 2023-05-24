import cv2
import numpy as np

# Define the lower and upper bounds of the color range in HSV color space
lower_color = np.array([100, 25, 200])
upper_color = np.array([180, 100, 255])

# Create a VideoCapture object to read from the camera (0 indicates the default camera)
cap = cv2.VideoCapture(0)

# Callback function for trackbar changes
def on_trackbar(val):
    pass

# Create a window for the trackbars
cv2.namedWindow("Trackbars")
cv2.createTrackbar("Rho", "Trackbars", 1, 10, on_trackbar)
cv2.createTrackbar("Theta", "Trackbars", 1, 180, on_trackbar)
cv2.createTrackbar("Threshold", "Trackbars", 50, 200, on_trackbar)
cv2.createTrackbar("MinLineLength", "Trackbars", 50, 200, on_trackbar)
cv2.createTrackbar("MaxLineGap", "Trackbars", 10, 50, on_trackbar)

while True:
    # Read a frame from the camera
    ret, frame = cap.read()

    # Convert the frame to HSV color space
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Threshold the frame to obtain only the desired color
    mask = cv2.inRange(hsv_frame, lower_color, upper_color)

    # Apply morphological operations to remove noise
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # Find contours of the objects in the masked image
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    ''' 
    # Iterate over the contours and filter out small ones
    min_contour_area = 100  # Minimum contour area to consider
    for contour in contours:
        if cv2.contourArea(contour) > min_contour_area:
            # Get the bounding box coordinates of the contour
            x, y, w, h = cv2.boundingRect(contour)

            # Draw the bounding box on the frame
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
    '''

    # Check if any contours are present
    if len(contours) > 0:
        # Find the contour with the largest area
        largest_contour = max(contours, key=cv2.contourArea)

        # Get the bounding box coordinates of the largest contour
        x, y, w, h = cv2.boundingRect(largest_contour)

        # Draw the bounding box on the frame
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # Extract the region of interest within the bounding box
        roi = mask[y:y + h, x:x + w]

        # Get the current trackbar values
        rho = cv2.getTrackbarPos("Rho", "Trackbars")
        theta = cv2.getTrackbarPos("Theta", "Trackbars")
        threshold = cv2.getTrackbarPos("Threshold", "Trackbars")
        min_line_length = cv2.getTrackbarPos("MinLineLength", "Trackbars")
        max_line_gap = cv2.getTrackbarPos("MaxLineGap", "Trackbars")

        # Detect lines within the region of interest using Hough Line Transform
        lines = cv2.HoughLinesP(roi, rho=rho, theta=np.pi / theta, threshold=threshold,
                                minLineLength=min_line_length, maxLineGap=max_line_gap)

        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                line_start = (x1 + x, y1 + y)
                line_end = (x2 + x, y2 + y)

                # Draw the detected line on the frame
                cv2.line(frame, line_start, line_end, (0, 0, 255), 2)

        ''' '''

    # Display the resulting frame
    cv2.imshow("Detected Objects", frame)

    # Exit loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the VideoCapture object and close any open windows
cap.release()
cv2.destroyAllWindows()
