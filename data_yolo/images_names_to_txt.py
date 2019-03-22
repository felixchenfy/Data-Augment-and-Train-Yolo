
import numpy as np 
import cv2
import os, sys
import math
from os import listdir
from os.path import isfile, join
CURR_PATH = os.path.dirname(os.path.abspath(__file__))+"/"

# Settings
fname = ['video_bottle','video_meter'][1] 
folder =  CURR_PATH + "/" + fname + '/images/'
output_filename = fname +".txt"

def get_filenames(path, sort = True, start_with=None):
    onlyfiles = [f for f in listdir(path) if isfile(join(path, f))]

    if start_with is not None:
        onlyfiles = [f for f in onlyfiles if f.startswith((start_with,))]
    if sort:
        onlyfiles.sort()
    return onlyfiles

fnames = get_filenames(folder)
fnames = [folder + f for f in fnames]

with open(output_filename, 'w') as f:
    for fname in fnames:
        f.write(fname+"\n")
