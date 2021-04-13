import socket
import sys
import cv2
import pickle
import numpy as np
import struct
import zlib
import os
import time



image_saved_path = '/streamlit_client_images'
if os.path.isdir(os.getcwd() + image_saved_path) == False:
	os.mkdir(os.getcwd() + image_saved_path)

# My private IP
# SERVER_IP = '172.26.174.143'

# Visual360 server IP
SERVER_IP = '74.82.31.134'

PORT = 5000
BUFFER_SIZE = 4096

global global_s
global_s = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
global_s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)



def receive_images_from_system():
	print ('In Rceiving Function')
	global global_s

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

			print ('First Processed Frame Received')
			frame_data = data[:msg_size]
			data = data[msg_size:]
			frame=pickle.loads(frame_data, fix_imports=True, encoding="bytes")
			frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)
			# cv2.imshow('front', frame)


			while len(data) < payload_size:
				data += global_s.recv(BUFFER_SIZE)
				
			packed_msg_size = data[:payload_size]
			data = data[payload_size:]
			msg_size = struct.unpack(">L", packed_msg_size)[0]

			while len(data) < msg_size:
				data += global_s.recv(4096)

			print ('Second Processed Frame Received')
			frame_data = data[:msg_size]
			data = data[msg_size:]
			frame=pickle.loads(frame_data, fix_imports=True, encoding="bytes")
			frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)
			# cv2.imshow('side', frame)

			print ('LEAVING function "receive_images_from_system"')
			break


		except Exception as e:
			print ('In function "receive_images_from_system", First Exception')
			print (e)
				




def transfer_images_to_server():
	print ('In Transfer Function')
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
				print ('LEAVING function "transfer_images_to_server"')
				break


		except Exception as e:
			print ('In function "transfer_images_to_server", First Exception')
			print (e)
			



if __name__ == "__main__":
	while 1:

		transfer_images_to_server()
		receive_images_from_system()
		global_s.close()
		time.sleep(5.0)
		global_s = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
		global_s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)


