import cv2

# Function to create a tracker based on the provided tracker name
def create_tracker():
    tracker_name = cv2.selectTracker("Select Tracker Type (KCF, CSRT, MOSSE): ")
    
    if tracker_name == 'KCF':
        return cv2.TrackerKCF_create()
    elif tracker_name == 'CSRT':
        return cv2.TrackerCSRT_create()
    elif tracker_name == 'MOSSE':
        return cv2.TrackerMOSSE_create()
    else:
        raise ValueError("Invalid tracker name")

# Open the video capture
cap = cv2.VideoCapture(0)

# Read the first frame from the video capture
ret, frame = cap.read()

# Select the object to track
bbox = cv2.selectROI("Object Tracker", frame, fromCenter=False, showCrosshair=True)

# Create the object tracker
tracker = create_tracker()
tracker.init(frame, bbox)

while True:
    # Read the frame from the video capture
    ret, frame = cap.read()

    # Update the tracker and get the updated bounding box
    success, bbox = tracker.update(frame)

    # Convert the bounding box coordinates to integers
    bbox = tuple(map(int, bbox))

    # Draw the tracked bounding box on the frame
    if success:
        x, y, w, h = bbox
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

    # Display the frame
    cv2.imshow("Object Tracker", frame)

    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video capture and close the window
cap.release()
cv2.destroyAllWindows()
