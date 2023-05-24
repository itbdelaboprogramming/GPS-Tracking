import cv2
import numpy as np

# Define the lower and upper boundaries of the color you want to detect (in HSV color space)
lower_color = np.array([150, 25, 200])  # Replace with your desired lower color range
upper_color = np.array([170, 100, 255])  # Replace with your desired upper color range

# Open the video capture
cap = cv2.VideoCapture(0)

while True:
    # Read a frame from the video capture
    ret, frame = cap.read()

    # Convert the frame to the HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Create a mask based on the color range
    mask = cv2.inRange(hsv, lower_color, upper_color)

    # Find contours in the masked image
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Find the largest contour
    largest_contour = max(contours, key=cv2.contourArea) if contours else None

    if largest_contour is not None:
        # Get the bounding rectangle of the largest contour
        x, y, w, h = cv2.boundingRect(largest_contour)

        # Draw the contour of the largest contour
        cv2.drawContours(frame, [largest_contour], 0, (0, 255, 0), 2)

        # Draw the bounding rectangle on the frame
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # Draw a line around the edge of the bounding rectangle
        cv2.line(frame, (x, y), (x + w, y), (255, 0, 0), 2)  # Top line
        cv2.line(frame, (x, y), (x, y + h), (255, 0, 0), 2)  # Left line
        cv2.line(frame, (x + w, y), (x + w, y + h), (255, 0, 0), 2)  # Right line
        cv2.line(frame, (x, y + h), (x + w, y + h), (255, 0, 0), 2)  # Bottom line

    # Display the resulting frame
    cv2.imshow("Color Detection", frame)

    # Exit loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the VideoCapture object and close any open windows
cap.release()
cv2.destroyAllWindows()
