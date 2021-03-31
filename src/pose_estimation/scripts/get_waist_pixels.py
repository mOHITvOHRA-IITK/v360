import cv2
import numpy as np
from functions import *




front_image_real_path = '/home/mohit/v360_ws/src/pose_estimation/scripts/images/front4.png'
side_image_real_path = '/home/mohit/v360_ws/src/pose_estimation/scripts/images/side4.png'

front_image_human_part_seg_path = '/home/mohit/v360_ws/src/pose_estimation/scripts/output/front4.png'
side_image_human_part_seg_path = '/home/mohit/v360_ws/src/pose_estimation/scripts/output/side4.png'

# front_image_real_path = '/home/mohit/Self-Correction-Human-Parsing-master/input/1.png'
# side_image_real_path = '/home/mohit/Self-Correction-Human-Parsing-master/input/2.png'

# front_image_human_part_seg_path = '/home/mohit/Self-Correction-Human-Parsing-master/output/1.png'
# side_image_human_part_seg_path = '/home/mohit/Self-Correction-Human-Parsing-master/output/2.png'


i1 = cv2.imread(front_image_real_path)
i2 = cv2.imread(side_image_real_path)

m1 = cv2.imread(front_image_human_part_seg_path)
m2 = cv2.imread(side_image_human_part_seg_path)

actual_height = (12*5 + 5) * 2.54 # in cm
get_measuremnets(i1, m1, i2, m2, actual_height, True)

cv2.waitKey(0)