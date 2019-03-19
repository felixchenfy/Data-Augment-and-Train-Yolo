
import cv2
import numpy as np
import os, sys
CURR_PATH = os.path.dirname(os.path.abspath(__file__))+"/"

folder = CURR_PATH + "../data/03-06/"
mask = cv2.imread(folder + "mask/00001_mask.png", cv2.IMREAD_UNCHANGED)
image =  cv2.imread(folder + "image/00001_image.png")

blue =image[..., 0]
blue[np.where(mask==1)] = 255

cv2.imshow("image", image)
cv2.waitKey()
cv2.destroyAllWindows()
