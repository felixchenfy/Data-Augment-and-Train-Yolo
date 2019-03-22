


# Basic usage of yolo


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
