(----  
DONT USE THIS 

My model's testing result using this keras-yolo3 is different from using Darkenet. The result is really bad here.
---)

# Install

cd into the root of my project, then:  
> $ git clone https://github.com/qqwweee/keras-yolo3
> $ cd keras-yolo3

Following operations are all done in the folder of keras-yolo3.

# Change bugs

1.In "yolo.py":

After this line "return_value, frame = vid.read()", add "if not return_value: break". The reason is, if it fails to read image from video, it needs to break the while loop.

2. In "yolo_video.py"

Change name of the input arguments

 parser.add_argument(
     '--model_path', type=str,
     help='path to model weight file, default ' + YOLO.get_defaults("model_path")
 )

 parser.add_argument(
     '--anchors_path', type=str,
     help='path to anchor definitions, default ' + YOLO.get_defaults("anchors_path")
 )

 parser.add_argument(
     '--classes_path', type=str,
     help='path to class definitions, default ' + YOLO.get_defaults("classes_path")
 )



# How to use 

## Its example usage

Test image
$ python yolo_video.py
$ /home/qiancheng/DISK/feiyu/TrainYolo/data_yolo/example1.png

Test video
$ python yolo_video.py --input /home/qiancheng/DISK/feiyu/TrainYolo/data_yolo/video_bottle.avi --output res_video.avi

Test wabcam
$ python yolo_video.py --input 0 --output ./

## Test my own

Change weights format:
$ python convert.py -w ../src2_train_yolo/yolov3-my-train.cfg ../model/v1-6000.weights model_data/v1-6000.h5


Test image
$ python yolo_video.py --model_path model_data/v1-6000.h5 --anchors_path model_data/yolo_anchors.txt --classes_path ../data_yolo/classes.txt --image
$ /home/qiancheng/DISK/feiyu/TrainYolo/data_yolo/example1.png

Test video
$ python yolo_video.py --model_path model_data/v1-6000.h5 --anchors_path model_data/yolo_anchors.txt --classes_path ../data_yolo/classes.txt --input ../data_yolo/video_bottle.avi --output ../results/video_bottle_res.avi


# Other notes

## -h for help

-h, --help         show this help message and exit
  --model MODEL      path to model weight file, default model_data/yolo.h5
  --anchors ANCHORS  path to anchor definitions, default
                     model_data/yolo_anchors.txt
  --classes CLASSES  path to class definitions, default
                     model_data/coco_classes.txt
  --gpu_num GPU_NUM  Number of GPU to use, default 1
  --image            Image detection mode, will ignore all positional
                     arguments
  --input [INPUT]    Video input path
  --output [OUTPUT]  [Optional] Video output path
