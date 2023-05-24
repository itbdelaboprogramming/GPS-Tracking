from device_camera import *
from darknet_yolo import *
#from realsense_camera import *
import argparse

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

net = DarknetDNN()
camera = DeviceCamera(device_id=args.camera)
#realsense = RealsenseCamera()

while True:
    #Get frame from camera
    frame = camera.get_frame()
    #ret, bgr_frame, depth_frame = realsense.get_frame_stream()

    #Detect human from the frame
    net.detect_object(frame)
    #net.detect_object_distance(bgr_frame, depth_frame)
    
    #Draw bounding box of the human detected
    net.draw_object(frame)
    #net.draw_object_with_distance(bgr_frame)

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