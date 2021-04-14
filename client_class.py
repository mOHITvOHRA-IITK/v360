import cv2
import time
import numpy as np
import os
import socket
import sys
import pickle
import struct
import zlib



image_saved_path = '/streamlit_client_images'
if os.path.isdir(os.getcwd() + image_saved_path) == False:
	os.mkdir(os.getcwd() + image_saved_path)


process_image_path = '/streamlit_server_process_images'
if os.path.isdir(os.getcwd() + process_image_path) == False:
	os.mkdir(os.getcwd() + process_image_path)


# My private IP
SERVER_IP = '172.26.174.143'

# Visual360 server IP
# SERVER_IP = '74.82.31.134'

PORT = 5000
BUFFER_SIZE = 4096





global global_s, front_image, side_image, socket_created_flag, front_image_flag, side_image_flag, info_updated
socket_created_flag = False
front_image_flag = False
side_image_flag = False
info_updated = False


def create_sender_obj():
	global global_s, socket_created_flag
	if socket_created_flag == False:
		global_s = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
		global_s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
		socket_created_flag = True
		

def save_images(frame):
	global front_image, side_image, front_image_flag, side_image_flag
	if front_image_flag == False:
		front_image_flag = True
		front_image = frame.copy()
		# cv2.imwrite(os.getcwd() + image_saved_path + '/front.png', frame)

	elif side_image_flag == False:
		side_image_flag = True
		side_image = frame.copy()
		# cv2.imwrite(os.getcwd() + image_saved_path + '/side.png', frame)


def check_image_status():
	global front_image_flag, side_image_flag
	if front_image_flag == True and side_image_flag == True:
		return True
	else:
		return False


def reset_image_front_status():
	global front_image_flag, info_updated
	info_updated = False
	front_image_flag = False
	if os.path.isfile(os.getcwd() + image_saved_path + '/front.png'):
		os.remove(os.getcwd() + image_saved_path + '/front.png')


def reset_image_side_status():
	global side_image_flag, info_updated
	info_updated = False
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



def transfer_data_to_server():
	global front_image, side_image, height_inch, height_feet, socket_created_flag
	global global_s

	while 1:
		try:
	
			global_s.connect((SERVER_IP, PORT))


			## Create a dictionary of input data and send to the server
			imageDict = {'front_image': front_image, 'side_image': side_image, 'feet': height_feet, 'inch': height_inch}
			pickleData = pickle.dumps(imageDict)
			global_s.sendall(struct.pack('>I', len(pickleData)))
			global_s.sendall(pickleData)
			

			## Receive the processed data from server
			data = b""
			data_size = struct.unpack('>I', global_s.recv(4))[0]
			while len(data) < data_size:
				data += global_s.recv(BUFFER_SIZE)
		
			imageDict = pickle.loads(data, fix_imports=True, encoding="bytes")

			global_s.close()
			socket_created_flag = False
			return imageDict


		except Exception as e:
			print ('In function "transfer_images_to_server", Exception')
			print (e)
	



global imageDict
imageDict = []

def store_imageDict(local_imageDict):
	global imageDict, info_updated
	imageDict = local_imageDict
	info_updated = True


def get_imageDict():
	global imageDict, info_updated
	return imageDict, info_updated

