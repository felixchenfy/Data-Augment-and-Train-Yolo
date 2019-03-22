

# Install

cd into the root of my project, then:  
> $ git clone https://github.com/AlexeyAB/darknet
> $ cd darknet
> $ make && cd ..

Example of usage:
> $ cd darkent && ./darknet detector test ./cfg/coco.data ./cfg/yolov3.cfg ./yolov3.weights data/dog.jpg

Then copy some necessary files to data folder of this project, which is needed later:
> $ cp -r darknet/data/labels data

Following operations are all done in my project's root folder.

# Collect training data

Generate 1095 images with objects

Read 170 negtive images from b_room_2/
Read 188 negtive images from b_room_3/
Read 258 negtive images from b_voc_2009/
Total negative = 616

Write 1711 images to /home/qiancheng/DISK/feiyu/auto_collect/data_yolo/images/

# Commands for Darknet

Train:
./darknet/darknet detector train src2_train_yolo/my-train.data src2_train_yolo/yolov3-my-train.cfg model/darknet53.conv.74 | tee backup/mylog.txt

Validate:
./darknet/darknet detector map src2_train_yolo/my-train.data src2_train_yolo/yolov3-my-train.cfg backup/yolov3-my-train_6000.weights

Test on single image:
./darknet/darknet detector test src2_train_yolo/my-train.data src2_train_yolo/yolov3-my-test.cfg model/v1-6000.weights data_yolo/example1.png -thresh 0.25
./darknet/darknet detector test src2_train_yolo/my-train.data src2_train_yolo/yolov3-my-test.cfg model/v1-6000.weights data_yolo/example2.png -thresh 0.25

Test on video (Bad):   
./darknet/darknet detector demo src2_train_yolo/my-train.data src2_train_yolo/yolov3-my-test.cfg model/v1-6000.weights data_yolo/test_bottle.avi -out_filename video_result.avi  
Bad result: (1) Low framerate. (2) The output avi cannot be opened.  

Test on batch of images (Bad):
./darknet/darknet detector test src2_train_yolo/my-train.data src2_train_yolo/yolov3-my-test.cfg model/v1-6000.weights -dont_show < data_yolo/video_bottle.txt > result.txt  
Bad result: (1) The bounding box information is not saved to txt, but only the detection result. (2) If not using -dont_show, I'll have to press keys one by one, which is pretty slow.  

Test on batch of images (Error. The repo is from https://github.com/vincentgong7/VG_AlexeyAB_darknet):  
./darknet/darknet detector batch src2_train_yolo/my-train.data src2_train_yolo/yolov3-my-test.cfg model/v1-6000.weights batch data_yolo/video_bottle/images/ tmp/ > tmp/results.txt  


# Other notes

## Basic usage of yolo


### Test on image
./darknet detector test cfg/coco.data cfg/yolov3.cfg yolov3.weights data/dog.jpg -thresh 0.25

./darknet detect cfg/yolov3.cfg yolov3.weights

### Test on webcam

$ ./darknet detector demo cfg/coco.data cfg/yolov3.cfg yolov3.weights
$ ./darknet detector demo cfg/coco.data cfg/yolov3-tiny.cfg yolov3-tiny.weights

check gpu usage: nvidia-smi

If memoery error: CUDA Error: out of memory
Then: 
$ subl cfg/yolov3.cfg
change from 'batch=64,subdivisions=16' to 'batch=32, subdivisions=2'


# Tutorials
https://github.com/AlexeyAB/darknet  
https://github.com/pascal1129/yolo_person_detect  
https://blog.csdn.net/lilai619/article/details/79695109  
https://blog.csdn.net/gzj2013/article/details/82285511  
https://zhuanlan.zhihu.com/p/35490655  
