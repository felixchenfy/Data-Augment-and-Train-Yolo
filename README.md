Use depth camera to detect objects on table.
Then, I'll put depth_cam onto Baxter to automatically collec training data for Yolo.

Data Augmentation and Train Yolo



# Train YOlo
Data format:

### 1. A .txt file containing all image paths:
```
/home/qiancheng/DISK/feiyu/darknet/train_yolo/data_voc/VOCdevkit/VOC2007/JPEGImages/000001.jpg
/home/qiancheng/DISK/feiyu/darknet/train_yolo/data_voc/VOCdevkit/VOC2007/JPEGImages/000002.jpg
...
```

### 2. Images and labels (6 digits)
Image:  
    <path>/JPEGImages/000001.jpg  

At the same level, put a folder named "labels":    
    <path>/labels//000001.txt  
It's content is:  
    <object-class> <x> <y> <width> <height>
```
11 0.34419263456090654 0.611 0.4164305949008499 0.262  
14 0.509915014164306 0.51 0.9745042492917847 0.972  
```
