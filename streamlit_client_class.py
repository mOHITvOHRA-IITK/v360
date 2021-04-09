import cv2
import time
import numpy as np
import os
import imagezmq
import socket

image_saved_path = '/streamlit_client_images'
if os.path.isdir(os.getcwd() + image_saved_path) == False:
	os.mkdir(os.getcwd() + image_saved_path)



global sender_obj, sende_obj_flag, sender_ip_address, front_image_flag, side_image_flag
sende_obj_flag = False
front_image_flag = False
side_image_flag = False


def create_sender_obj():
	global sender_obj, sende_obj_flag, sender_ip_address
	if sende_obj_flag == False:
		sender_obj = imagezmq.ImageSender(connect_to="tcp://172.26.174.143:5555")
		sende_obj_flag = True
		hostname = socket.gethostname()
		sender_ip_address = socket.gethostbyname(hostname)


def send_front_image(frame):
	print ('sending front image')
	global sender_obj, sender_ip_address, front_image_flag
	sender_obj.send_image(sender_ip_address, frame)
	front_image_flag = True
	cv2.imwrite(os.getcwd() + image_saved_path + '/front.png', frame)


def send_side_image(frame):
	print ('sending side image')
	global sender_obj, sender_ip_address, side_image_flag
	sender_obj.send_image(sender_ip_address, frame)
	side_image_flag = True
	cv2.imwrite(os.getcwd() + image_saved_path + '/side.png', frame)


def send_images(frame):
	global front_image_flag, side_image_flag
	if front_image_flag == False:
		send_front_image(frame)
	else:
		send_side_image(frame)


def check_image_status():
	global front_image_flag, side_image_flag
	if front_image_flag == True and side_image_flag == True:
		return True
	else:
		return False


def reset_image_front_status():
	global front_image_flag, side_image_flag
	front_image_flag = False
	if os.path.isfile(os.getcwd() + image_saved_path + '/front.png'):
		os.remove(os.getcwd() + image_saved_path + '/front.png')

	side_image_flag = False
	if os.path.isfile(os.getcwd() + image_saved_path + '/side.png'):
		os.remove(os.getcwd() + image_saved_path + '/side.png')


def reset_image_side_status():
	global side_image_flag
	side_image_flag = False
	if os.path.isfile(os.getcwd() + image_saved_path + '/side.png'):
		os.remove(os.getcwd() + image_saved_path + '/side.png')



global current_time, current_time_flag, max_count
current_time_flag = False
max_count = 5


def start_count_down():
	global current_time, current_time_flag
	if current_time_flag == False:
		current_time_flag = True
		current_time = time.time()


def get_countdown():
	global current_time, current_time_flag

	if current_time_flag == False:
		return -1
	else:
		count_down = max_count - np.int( time.time() - current_time)

		if count_down <= 0:
			current_time_flag = False

		return count_down


def stop_count_down():
	global current_time_flag
	current_time_flag = False




global camera_object_start, camera_object
camera_object_start = False


def initialize_webcam():
	global camera_object_start, camera_object
	if camera_object_start == False:
		camera_object_start = True
		camera_object = cv2.VideoCapture(0)


def stop_webcam():
	global camera_object_start, camera_object
	initialize_webcam()
	camera_object_start = False
	camera_object.release()


def get_current_feed():
	global camera_object_start, camera_object

	if camera_object_start == True:
		_, frame = camera_object.read()
		return frame
	else:
		return np.array([])



global height_inch, height_feet
height_inch = 0
height_feet = 0


def store_height_feet(feet):
	global height_feet
	height_feet = feet


def store_height_inch(inch):
	global height_inch
	height_inch = inch


def get_height_feet_inch():
	global height_inch, height_feet
	return height_inch, height_feet


# global imageHub2
# imageHub2 = imagezmq.ImageHub()

# def save_process_image():
# 	while True:
# 		(server_info, frame) = imageHub2.recv_image()
# 		imageHub2.send_reply(b'OK')
# 		cv2.imshow('frame', frame)
# 		cv2.waitKey(0)

