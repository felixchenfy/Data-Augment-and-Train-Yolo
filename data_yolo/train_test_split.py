
import numpy as np 
from pathlib2 import Path

folder = '/home/qiancheng/DISK/feiyu/auto_collect/data_yolo/'
fin = 'images_info.txt'
fout_train = 'train.txt'
fout_test = 'test.txt'

# Read in image paths
contents = Path(fin).read_text().split('\n')
contents = list(filter(None, contents)) 
print(contents, type(contents))


# Train/Test split
N = len(contents)
num_train = 0.8

idx_all = range(N)
idx_train = np.random.permutation(N)[0:int(N*num_train)]
idx_test = list(set(idx_all).difference(set(idx_train)))
idx_train.sort()
idx_test.sort()
print("\n\nidx_train:\n", idx_train, "number = {}".format(len(idx_train)))
print("\n\nidx_test:\n", idx_test, "number = {}".format(len(idx_test)))

# Write to file
with open(folder+fout_train, 'w') as f:
    for i in idx_train:
        f.write(contents[i]+"\n")
with open(folder+fout_test, 'w') as f:
    for i in idx_test:
        f.write(contents[i]+"\n")