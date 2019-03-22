

import cv2
import numpy as np
import os, sys
import matplotlib.pyplot as plt 

def cc(I):
    # print(type(I),I.shape)
    if len(I.shape)==3 and I.shape[2]==3 and (type(I[0][0][0])!=np.float64):
        I = cv2.cvtColor(I, cv2.COLOR_BGR2RGB)
    return I

def cv2_imshow(img):
    cv2.imshow("image", image)
    cv2.waitKey()
    cv2.destroyAllWindows()

def show(img, size = (6, 10)):
    img = cc(img)
    plt.figure(figsize=size)
    plt.imshow(img)
    plt.show()
    
def show2(i1, i2, size=(6,12)):
    i1,i2 = cc(i1), cc(i2)
    plt.figure(figsize=size)
    plt.subplot(121)
    plt.imshow(i1)
    plt.subplot(122)
    plt.imshow(i2)

def show3(i1, i2, i3, size=(6,12)):
    i1,i2,i3=cc(i1),cc(i2),cc(i3)
    plt.figure(figsize=size)
    plt.subplot(131)
    plt.imshow(i1)
    plt.subplot(132)
    plt.imshow(i2) 
    plt.subplot(133)
    plt.imshow(i3)
    
def show22(i1, i2, i3, i4):
    i1,i2,i3,i4=cc(i1),cc(i2),cc(i3),cc(i4)
    # plt.figure(figsize=(10,14))
    plt.subplot(221)
    plt.imshow(i1)
    plt.subplot(222)
    plt.imshow(i2)
    plt.subplot(223)
    plt.imshow(i3)
    plt.subplot(224)
    plt.imshow(i4)