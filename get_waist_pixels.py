import cv2
import numpy as np
import os
from functions import *




image_saved_path = '/streamlit_images'
if os.path.isdir(os.getcwd() + image_saved_path) == False:
	os.mkdir(os.getcwd() + image_saved_path)


process_image_path = '/streamlit_process_images'
if os.path.isdir(os.getcwd() + process_image_path) == False:
	os.mkdir(os.getcwd() + process_image_path)


i1_path = os.getcwd() + image_saved_path + '/front.png'
i1 = cv2.imread(i1_path)
m1_path = os.getcwd() + process_image_path + '/front.png'
m1 = cv2.imread(m1_path)
i2_path = os.getcwd() + image_saved_path + '/side.png'
i2 = cv2.imread(i2_path)
m2_path = os.getcwd() + process_image_path + '/side.png'
m2 = cv2.imread(m2_path)

actual_height = (12*5 + 5) * 2.54 # in cm
get_measuremnets(i1, m1, i2, m2, actual_height, True)

cv2.waitKey(0)