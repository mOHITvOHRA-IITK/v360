import cv2
import time
import numpy as np
import os
import socket
import sys
import pickle
import struct
import zlib
from functions import get_measuremnets


image_saved_path = '/streamlit_client_images'
if os.path.isdir(os.getcwd() + image_saved_path) == False:
	os.mkdir(os.getcwd() + image_saved_path)


process_image_path = '/streamlit_server_process_images'
if os.path.isdir(os.getcwd() + process_image_path) == False:
	os.mkdir(os.getcwd() + process_image_path)


# My private IP
# SERVER_IP = '172.26.174.143'

# Visual360 server IP
SERVER_IP = '74.82.31.134'

PORT = 5000
BUFFER_SIZE = 4096

global global_s, socket_created_flag, front_image_flag, side_image_flag, calculate_human_dim
socket_created_flag = False
front_image_flag = False
side_image_flag = False
calculate_human_dim = False



def create_sender_obj():
	global global_s, socket_created_flag
	if socket_created_flag == False:
		global_s = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
		global_s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
		socket_created_flag = True
		

def save_images(frame):
	global front_image_flag, side_image_flag
	if front_image_flag == False:
		front_image_flag = True
		cv2.imwrite(os.getcwd() + image_saved_path + '/front.png', frame)
	elif side_image_flag == False:
		side_image_flag = True
		cv2.imwrite(os.getcwd() + image_saved_path + '/side.png', frame)


def check_image_status():
	global front_image_flag, side_image_flag
	if front_image_flag == True and side_image_flag == True:
		return True
	else:
		return False


def reset_image_front_status():
	global front_image_flag
	front_image_flag = False
	if os.path.isfile(os.getcwd() + image_saved_path + '/front.png'):
		os.remove(os.getcwd() + image_saved_path + '/front.png')


def reset_image_side_status():
	global side_image_flag
	side_image_flag = False
	if os.path.isfile(os.getcwd() + image_saved_path + '/side.png'):
		os.remove(os.getcwd() + image_saved_path + '/side.png')


def load_images_on_server():

	global global_s

	while 1:
		try:
			if os.path.isfile(os.getcwd() + image_saved_path + '/front.png') and os.path.isfile(os.getcwd() + image_saved_path + '/side.png'):
				global_s.connect((SERVER_IP, PORT))

				file_name = '/front.png'
				frame = cv2.imread(os.getcwd() + image_saved_path + file_name)
				result, frame = cv2.imencode('.png', frame)
				data = pickle.dumps(frame, 0)
				size = len(data)
				global_s.sendall(struct.pack(">L", size) + data)


				file_name = '/side.png'
				frame = cv2.imread(os.getcwd() + image_saved_path + file_name)
				result, frame = cv2.imencode('.png', frame)
				data = pickle.dumps(frame, 0)
				size = len(data)
				global_s.sendall(struct.pack(">L", size) + data)


				from_server = global_s.recv(4096)
				print (from_server)
				break


		except Exception as e:
			print ('In exception of function ---> load_images_on_server')
			print (e)





def receive_processed_images_from_server():
	global global_s, socket_created_flag, calculate_human_dim


	while 1:
		try:
			data = b""
			payload_size = struct.calcsize(">L")

			while len(data) < payload_size:
				data += global_s.recv(BUFFER_SIZE)
				
			packed_msg_size = data[:payload_size]
			data = data[payload_size:]
			msg_size = struct.unpack(">L", packed_msg_size)[0]

			while len(data) < msg_size:
				data += global_s.recv(4096)

			frame_data = data[:msg_size]
			data = data[msg_size:]
			frame=pickle.loads(frame_data, fix_imports=True, encoding="bytes")
			frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)
			# cv2.imshow('front', frame)
			cv2.imwrite(os.getcwd() + process_image_path + '/front.png', frame)


			while len(data) < payload_size:
				data += global_s.recv(BUFFER_SIZE)
				
			packed_msg_size = data[:payload_size]
			data = data[payload_size:]
			msg_size = struct.unpack(">L", packed_msg_size)[0]

			while len(data) < msg_size:
				data += global_s.recv(4096)

			frame_data = data[:msg_size]
			data = data[msg_size:]
			frame=pickle.loads(frame_data, fix_imports=True, encoding="bytes")
			frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)
			# cv2.imshow('side', frame)
			cv2.imwrite(os.getcwd() + process_image_path + '/side.png', frame)

			print ('Both processed frames are received in client\n')

			calculate_human_dim = True
			break


		except Exception as e:
			print ('In exception of function ---> receive_processed_images_from_server')
			print (e)
				

	global_s.close()
	socket_created_flag = False
	




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



global dim_status, human_waist, human_chest, human_thigh, human_front_sleeve_in_cm, human_dis_in_cm, human_image, human_side_image
human_waist = -1
human_chest = -1
human_thigh = -1
human_front_sleeve_in_cm = -1
human_dis_in_cm = -1
human_image = np.array([])
human_side_image = np.array([])



def get_human_info():
	global height_inch, height_feet, human_waist, human_chest, human_thigh, human_front_sleeve_in_cm, human_dis_in_cm, human_image, human_side_image, calculate_human_dim

	if calculate_human_dim:
		actual_height = (12*height_feet + height_inch) * 2.54 # in cm
		i1_path = os.getcwd() + image_saved_path + '/front.png'
		i1 = cv2.imread(i1_path)
		m1_path = os.getcwd() + process_image_path + '/front.png'
		m1 = cv2.imread(m1_path)
		i2_path = os.getcwd() + image_saved_path + '/side.png'
		i2 = cv2.imread(i2_path)
		m2_path = os.getcwd() + process_image_path + '/side.png'
		m2 = cv2.imread(m2_path)
		_, human_waist, human_chest, human_thigh, human_front_sleeve_in_cm, human_dis_in_cm, human_image, human_side_image = get_measuremnets(i1, m1, i2, m2, actual_height, True)
		calculate_human_dim = False


	return human_waist, human_chest, human_thigh, human_front_sleeve_in_cm, human_dis_in_cm, human_image, human_side_image
