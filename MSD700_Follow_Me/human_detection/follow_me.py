from device_camera import *
from darknet_yolo import *

net = DarknetDNN()
camera = DeviceCamera()

while True:
    #Get frame from camera
    frame = camera.get_frame()

    #Detect human from the frame
    net.detect_object(frame)
    
    #Draw bounding box of the human detected
    net.draw_object(frame)

    #Draw grid
    camera.create_grid()

    #Display the image
    camera.show()

    #exit condition
    key = cv2.waitKey(1)
    if key == 27:
        print(f"Key {key} is pressed.")
        break

camera.release()