import cv2
import numpy as np
from functions import *

upper_legs_part = [128,0,128]
torse_part = [0,128,0]


front_image_real_path = '/home/mohit/v360_ws/src/pose_estimation/scripts/images/front.png'
side_image_real_path = '/home/mohit/v360_ws/src/pose_estimation/scripts/images/side.png'

front_image_human_part_seg_path = '/home/mohit/v360_ws/src/pose_estimation/scripts/output/front.png'
side_image_human_part_seg_path = '/home/mohit/v360_ws/src/pose_estimation/scripts/output/side.png'

# front_image_real_path = '/home/mohit/Self-Correction-Human-Parsing-master/input/1.png'
# side_image_real_path = '/home/mohit/Self-Correction-Human-Parsing-master/input/2.png'

# front_image_human_part_seg_path = '/home/mohit/Self-Correction-Human-Parsing-master/output/1.png'
# side_image_human_part_seg_path = '/home/mohit/Self-Correction-Human-Parsing-master/output/2.png'


i1 = cv2.imread(front_image_real_path)
i2 = cv2.imread(side_image_real_path)

m1 = cv2.imread(front_image_human_part_seg_path)
m2 = cv2.imread(side_image_human_part_seg_path)




front_waist_width, front_chest_width, front_person_height = get_front_chest_and_waist(i1, m1, True)
side_waist_width, side_chest_width, side_person_height = get_side_chest_and_waist(i2, m2, True)

actual_height = 65*2.54 # in cm, (1 inch = 2.54 cm)
print ('front', front_waist_width, front_chest_width, front_person_height)
print ('side', side_waist_width, side_chest_width, side_person_height)


cm_per_pixel = actual_height / front_person_height
front_waist_in_cm = front_waist_width*cm_per_pixel
front_chest_in_cm = front_chest_width*cm_per_pixel



cm_per_pixel = actual_height / side_person_height
side_waist_in_cm = side_waist_width*cm_per_pixel
side_chest_in_cm = side_chest_width*cm_per_pixel


print ('front_waist_in_cm', front_waist_in_cm, ' and in inches ', front_waist_in_cm/2.54)
print ('side_waist_in_cm', side_waist_in_cm, ' and in inches ', side_waist_in_cm/2.54)
r1 = front_waist_in_cm/2
r2 = side_waist_in_cm/2
waist = 2*(22/7)*np.sqrt(0.5*r1*r1 + 0.5*r2*r2)
print ('waist_in_cm', waist, ' and in inches ', waist/2.54)



print ('front_chest_in_cm', front_chest_in_cm, ' and in inches ', front_chest_in_cm/2.54)
print ('side_chest_in_cm', side_chest_in_cm, ' and in inches ', side_chest_in_cm/2.54)
r1 = front_chest_in_cm/2
r2 = side_chest_in_cm/2
chest = 2*(22/7)*np.sqrt(0.5*r1*r1 + 0.5*r2*r2)
print ('chest_in_cm', chest, ' and in inches ', chest/2.54)


cv2.waitKey(0)