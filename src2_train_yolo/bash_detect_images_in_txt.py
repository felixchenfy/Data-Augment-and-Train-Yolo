import os, sys
import cv2
import time
CURR_PATH = os.path.dirname(os.path.abspath(__file__))+"/"

# WARNING!!!! Please run this from the root folder: 
# $ python src2_train_yolo/bash_detect_images_in_txt.py

# Settings 
base_command = './darknet/darknet detector test src2_train_yolo/my-train.data src2_train_yolo/yolov3-my-test.cfg model/v1-6000.weights -dont_show'

object_name = 'video_meter'
input_txt_path = CURR_PATH + "../data_yolo/"+object_name+".txt"
output_images_folder = CURR_PATH + "../results/" + object_name + "/"

# Loop and detect
d = 0
cnt_line = 0

with open((input_txt_path ),'r') as f:
    for line in f:
        cnt_line += 1

        image = line.strip()
        print("Processing:", image)

        os.system(base_command + ' ' + image)

        img = cv2.imread(CURR_PATH + "../predictions.jpg")
        cv2.imwrite(output_images_folder + "{:06d}.jpg".format(cnt_line), img)

        cv2.imshow("Detection result" , img)
        q = cv2.waitKey(10)
        if q!=-1 and q=='q': # Why can't it stop ?!
            break