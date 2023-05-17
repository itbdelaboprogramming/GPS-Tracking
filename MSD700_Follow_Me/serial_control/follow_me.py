from device_camera import *
from darknet_yolo import *
import argparse
import serial
import time

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

#Serial Object
port = '/dev/ttyUSB0'
ser = serial.Serial(port, 9600, timeout=1)
ser.reset_input_buffer()
timestamp = time.time()
frequency = 10 # in Hz

while True:
    #Get frame from camera
    frame = camera.get_frame()

    #Detect human from the frame
    net.detect_object(frame)
    
    #Draw bounding box of the human detected
    net.draw_object(frame)

    if time.time() - timestamp >= 1/frequency:
        direct = net.get_command()

        if direct == 'Right':
            command = '1'
        elif direct == 'Left':
            command = '2'
        elif direct == 'Center':
            command = '3'
        else:
            command = '0'
        
        ser.write(command.encode("utf-8"))
        timestamp = time.time()

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