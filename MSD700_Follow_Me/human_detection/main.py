import os
import argparse
import cv2
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

if args.image:
    image_path = input('Enter the path to the image: ')
    print(image_path)

weight_path = os.path.join(ROOT_DIR, 'weights/yolov3-tiny.weights')
cfg_path = os.path.join(ROOT_DIR, 'cfg/yolov3-tiny.cfg')
print("Loading model from ", weight_path, "and", cfg_path)
net = cv2.dnn.readNet(weight_path, cfg_path)
confidance_threshold = 0.2
nms_threshold = 0.5

