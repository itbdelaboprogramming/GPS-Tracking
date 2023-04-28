import os
import argparse
from config.definition import ROOT_DIR

dnn_model = os.path.join(ROOT_DIR, 'weights/yolov3-tiny.weights')
#print(dnn_model)

parser = argparse.ArgumentParser(
    description= 'This program will detect human',
    epilog= 'Hope this works'
)

#parser.add_argument("mode", choices=['i', 'v'] , help="Input mode for human detection, i for image and v for video")
#parser.add_argument("source", help="De")
parser.add_argument('-i', '--image', action='store_true')
parser.add_argument('source', default='0')
args = parser.parse_args()
print(args)
print(args.image)
print(args.source)