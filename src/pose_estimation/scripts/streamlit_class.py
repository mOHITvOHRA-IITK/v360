import cv2
import time
import numpy as np

global camera_object_start, camera_object
camera_object_start = False


global height_feet, height_inch
height_feet = 0
height_inch = 0

global start_count, timer_start_flag, start_time
start_count = False
timer_start_flag = False
start_time = 0


global front_image, front_image_flag, side_image, side_image_flag
front_image_flag = False
side_image_flag = False


countdown = 5

def store_height_feet(height_feet_var):
	global height_feet
	height_feet = height_feet_var
	



def store_height_inch(height_inch_var):
	global height_inch
	height_inch = height_inch_var
	



def get_height_feet_inch():
	global height_feet, height_inch
	return height_feet, height_inch


def stop_count_down():
	global start_count, timer_start_flag
	start_count = False
	timer_start_flag = False


def start_count_down():
	global start_count, timer_start_flag
	start_count = True
	timer_start_flag = False



def get_countdown():
	global timer_start_flag, start_time, start_count
	if start_count == True and timer_start_flag == False:
		timer_start_flag = True
		start_time = time.time()

	count_down = countdown - np.int( time.time() - start_time)

	if count_down <0:
		timer_start_flag = False
		start_count = False

	if count_down == 0 and timer_start_flag:
		start_count = False
		timer_start_flag = False
		return 0

	if start_count == True:
		return count_down
	else:
		return -1




def initialize_webcam():
	global camera_object_start, camera_object

	if camera_object_start == False:
		camera_object_start = True
		camera_object = cv2.VideoCapture(0)



def get_current_feed():
	global camera_object_start, camera_object

	if camera_object_start == True:
		_, frame = camera_object.read()
		return frame
	else:
		return None



def stop_webcam():
	global camera_object_start, camera_object

	initialize_webcam()

	camera_object_start = False
	camera_object.release()




def save_front_side_images(reset, frame):
	global front_image, front_image_flag, side_image, side_image_flag

	if reset:
		front_image_flag = False
		side_image_flag = False

	else:

		if front_image_flag == False:
			front_image_flag = True
			front_image = frame

		elif side_image_flag == False:
			side_image_flag = True
			side_image = frame



def visualize_images():
	global front_image, front_image_flag, side_image, side_image_flag

	if front_image_flag:
		cv2.imshow('front_image', front_image)
		cv2.waitKey(1)
	
		
	if side_image_flag:
		cv2.imshow('side_image', side_image)
		cv2.waitKey(1)
	
		


def check_both_images_saved():
	global front_image_flag, side_image_flag
	if front_image_flag and side_image_flag:
		return True
	else:
		return False




