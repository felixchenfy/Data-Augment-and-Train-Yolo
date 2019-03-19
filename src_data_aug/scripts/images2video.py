

import numpy as np
import cv2
import sys, os, time
import numpy as np
import simplejson
import sys, os
import csv
CURR_PATH = os.path.dirname(os.path.abspath(__file__))+"/"
sys.path.append(CURR_PATH+"../")
from mylib.funcs import get_filenames
CURR_PATH = os.path.dirname(os.path.abspath(__file__))+"/"

if 1:
    image_folder = CURR_PATH + '../../data/b_room_1/'
    video_name = CURR_PATH + '../background.avi'
    fnames = get_filenames(image_folder, start_with='image')
    N = len(fnames)
    image_start = 1
    image_end = N
    framerate = 10
    FASTER_RATE = 1

# Read image and save to video'
fourcc = cv2.VideoWriter_fourcc(*'XVID')
cnt = 0
for i in range(image_start, image_end+1):
    cnt += 1
    fname = "image{:05d}.png".format(i)
    frame = cv2.imread(image_folder + fname)
    if cnt==1:
        width = frame.shape[1]
        height = frame.shape[0]
        video = cv2.VideoWriter(video_name, fourcc, framerate, (width,height))
    print("Processing the {}/{}th image: {}".format(cnt, image_end - image_start + 1, fname))
    if i%FASTER_RATE ==0:
        video.write(frame)

cv2.destroyAllWindows()
video.release()