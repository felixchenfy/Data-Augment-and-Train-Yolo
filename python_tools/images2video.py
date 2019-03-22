
import numpy as np
import cv2
import sys, os, time
import numpy as np
import simplejson
import csv
from os import listdir
from os.path import isfile, join
CURR_PATH = os.path.dirname(os.path.abspath(__file__))+"/"

int2str = lambda num, blank: ("{:0"+str(blank)+"d}").format(num)

def get_filenames(path, sort = True, start_with=None):
    onlyfiles = [f for f in listdir(path) if isfile(join(path, f))]

    if start_with is not None:
        onlyfiles = [f for f in onlyfiles if f.startswith((start_with,))]
    if sort:
        onlyfiles.sort()
    return onlyfiles

# ---------------------------------------------------------------------------

if 1:
    image_folder = CURR_PATH + '../results/video_meter/'
    video_name = CURR_PATH + '../results/video_meter.avi'
    fnames = get_filenames(image_folder, start_with=None)
    N = len(fnames)
    image_start = 1
    image_end = N
    framerate = 30
    FASTER_RATE = 1
    get_ith_image_name = lambda ith: "{:06d}.jpg".format(ith)

# Read image and save to video'
fourcc = cv2.VideoWriter_fourcc(*'XVID')
cnt = 0
for i in range(image_start, image_end+1):
    cnt += 1
    fname = get_ith_image_name(i)
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
