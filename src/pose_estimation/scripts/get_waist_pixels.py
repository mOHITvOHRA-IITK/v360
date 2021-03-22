import cv2
import numpy as np
from functions import *

upper_legs_part = [128,0,128]
torse_part = [0,128,0]


front_image_real_path = '/home/mohit/Self-Correction-Human-Parsing-master/input/21.png'
side_image_real_path = '/home/mohit/Self-Correction-Human-Parsing-master/input/22.png'

front_image_human_part_seg_path = '/home/mohit/Self-Correction-Human-Parsing-master/output/21.png'
side_image_human_part_seg_path = '/home/mohit/Self-Correction-Human-Parsing-master/output/22.png'


i1 = cv2.imread(front_image_real_path)
i2 = cv2.imread(side_image_real_path)

m1 = cv2.imread(front_image_human_part_seg_path)
m2 = cv2.imread(side_image_human_part_seg_path)



actual_height = 66*2.54 # in cm, (1 inch = 2.54 cm)

front_waist_width, front_person_height = get_person_front_measurements(i1, m1)
side_waist_width, side_person_height = get_person_side_measurements(i2, m2)

print ('front', front_waist_width, front_person_height)
print ('side', side_waist_width, side_person_height)


cm_per_pixel = actual_height / front_person_height
front_waist_in_cm = cm_per_pixel*front_waist_width
print ('front_waist_in_cm', front_waist_in_cm, ' and in inches ', front_waist_in_cm/2.54)

cm_per_pixel = actual_height / side_person_height
side_waist_in_cm = cm_per_pixel*side_waist_width
print ('side_waist_in_cm', side_waist_in_cm, ' and in inches ', side_waist_in_cm/2.54)

r1 = front_waist_in_cm/2
r2 = side_waist_in_cm/2

waist = 2*(22/7)*np.sqrt(0.5*r1*r1 + 0.5*r2*r2)
print ('waist (cm)', waist)
waist = waist/2.54
print ('waist (inches)', waist)

front_waist_width, front_person_height = get_person_front_measurements(i1, m1, visualize=True)
side_waist_width, side_person_height = get_person_side_measurements(i2, m2, visualize=True)


