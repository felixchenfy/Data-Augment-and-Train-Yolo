import os, sys
import glob
os.system
import PIL
import PIL.Image as Image
CURR_PATH = os.path.dirname(os.path.abspath(__file__))+"/"

# WARNING!!!! Please run this from the root folder: 
# $ python src2_train_yolo/detect_images_in_txt.py

# Settings 
base_command = './darknet/darknet detector test src2_train_yolo/my-train.data src2_train_yolo/yolov3-my-test.cfg model/v1-6000.weights -dont_show'

object_name = 'video_meter'
input_txt_path = CURR_PATH + "../data_yolo/"+object_name+".txt"
output_images_folder = CURR_PATH + "../results/" + object_name + "/"

# Loop and detect
d = 0
cnt_line = 0

with open((input_txt_path ),'r') as fobj:
    for line in fobj:
        cnt_line += 1

        image = line.strip()
        print(image)

        commands = [base_command, image]
        os.system(' '.join(commands))

        predicted_image = Image.open( CURR_PATH + "../predictions.jpg")

        output = output_images_folder + "{:06d}.jpg".format(cnt_line)
        predicted_image.save(output)

        print(output)