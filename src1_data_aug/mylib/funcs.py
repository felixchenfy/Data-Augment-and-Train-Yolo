
# Basics =============================================
import numpy as np
import cv2
import math
from os import listdir
from os.path import isfile, join

int2str = lambda num, blank: ("{:0"+str(blank)+"d}").format(num)

def get_filenames(path, sort = True, start_with=None):
    onlyfiles = [f for f in listdir(path) if isfile(join(path, f))]

    if start_with is not None:
        onlyfiles = [f for f in onlyfiles if f.startswith((start_with,))]
    if sort:
        onlyfiles.sort()
    return onlyfiles
    

# Image processing and basic operations =============================================
def add_color(img, mask, channel, mask_value = 1):
    img_disp = img.copy()
    i_sub = img_disp[..., channel]
    i_sub[np.where(mask==mask_value)] = 255
    return img_disp

def getBbox(mask):
    rows = np.any(mask, axis=1)
    cols = np.any(mask, axis=0)
    try:
        rmin, rmax = np.where(rows)[0][[0, -1]]
        cmin, cmax = np.where(cols)[0][[0, -1]]
        return rmin, rmax, cmin, cmax
    except:
        return None, None, None, None
        

def cropImg(img, rmin, rmax, cmin, cmax):
    if len(img.shape)==2:
        return img[rmin:rmax, cmin:cmax]
    else:
        return img[rmin:rmax, cmin:cmax, :]

# Simple maths =============================================
def randint(num):
    return int(np.random.random()*num)
def randval(num):
    return np.random.random()*num
def randval_mean0(num): 
    # return np.random.random()*num*2 - num
    return np.random.uniform(-num, num)

# Transform masked object onto another image =============================================
from skimage.transform import warp
from skimage.transform import AffineTransform

# translation
def createNewImgMask(img, mask, dst,
        obj_relative_size, # size relative to the image
    ):
    objcol = img.shape[1]
    objrow = img.shape[0]
    imgcol = dst.shape[1]
    imgrow = dst.shape[0]


    # Set scalling
    if 0:
        scale_x = 1.0 + randval_mean0(0.3)
        scale_y = 1.0 + randval_mean0(0.3)

    else:
        scale = 1.0 * ((imgcol+imgrow)*obj_relative_size) / (objcol+objrow)
        scale_x = scale * (1 + randval_mean0(0.2))
        scale_y = scale * (1 + randval_mean0(0.2))

    objcol_new = objcol * scale_x
    objrow_new = objrow * scale_y

    # Set transformation
    rotation = randval_mean0(1.0)
    shear = randval_mean0(0.03)
    tx = (imgcol )/2.0 + randval_mean0((imgcol - objcol_new)/2.0)/1.2
    ty = (imgrow )/2.0 + randval_mean0((imgrow - objrow_new)/2.0)/1.2

    # -- Start transform
    t1 = AffineTransform(translation=(-objcol/2, -objrow/2))
    t2 = AffineTransform(scale=(scale_x, scale_y), rotation=rotation, shear=shear)
    t3 = AffineTransform(translation=(tx, ty))
    
    def get(t):
        # print(t.params)
        return t.params
    t = get(t3).dot(get(t2)).dot(get(t1))
    tform = AffineTransform(t)
    
    if 0: # Print transform
        print("transf:scale_x={},scale_y={},rot={},shear={},tx={},ty={}".format(scale_x,scale_y,rotation,shear,tx,ty))
        print(tform.params)

    img_new = warp(img, tform.inverse, output_shape=(imgrow, imgcol))
    mask_new = warp(mask, tform.inverse, output_shape=(imgrow, imgcol))
    
    return img_new, mask_new

def createNewImage(src_img, src_mask, dst, obj_relative_size = 0.2):
    src_img_new, src_mask_new = createNewImgMask(src_img, src_mask, dst, obj_relative_size)
    dst_new = dst.copy()
    idx = np.where(src_mask_new==1)
    dst_new[idx] = (src_img_new[idx]*255).astype(np.uint8)
    return dst_new, src_img_new, src_mask_new