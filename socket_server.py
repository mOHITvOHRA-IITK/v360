import socket
import sys
import cv2
import pickle
import numpy as np
import struct
import zlib
import os
from streamlit_server_class import *
import time


global client_IP
client_IP = '0.0.0.0'
PORT = 5000



global global_s
global_s = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
global_s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

global conn
conn = None



image_saved_path = '/streamlit_server_images'
if os.path.isdir(os.getcwd() + image_saved_path) == False:
	os.mkdir(os.getcwd() + image_saved_path)


image_process_path = '/streamlit_server_process_images'
if os.path.isdir(os.getcwd() + image_saved_path) == False:
	os.mkdir(os.getcwd() + image_saved_path)

BUFFER_SIZE = 4096



def receive_images_from_client():
	print ('\nIn Rceiving Function')
	global global_s, conn, client_IP
	

	while 1:
		move_forward = True
		try:
			global_s.bind(('', PORT))
			global_s.listen(10)
		except Exception as e:
			print ('In function "receive_images_from_client", First Exception')
			print (e)
			move_forward = False

		if move_forward:
			break


	while 1:
		try:
			conn, addr = global_s.accept()
			client_IP = addr[0]
			data = b""
			payload_size = struct.calcsize(">L")

			while len(data) < payload_size:
				data += conn.recv(BUFFER_SIZE)
				
			packed_msg_size = data[:payload_size]
			data = data[payload_size:]
			msg_size = struct.unpack(">L", packed_msg_size)[0]

			while len(data) < msg_size:
				data += conn.recv(4096)

			print ('First Frame Received in Server')
			frame_data = data[:msg_size]
			data = data[msg_size:]
			frame=pickle.loads(frame_data, fix_imports=True, encoding="bytes")
			frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)
			cv2.imwrite(os.getcwd() + image_saved_path + '/front.png', frame)


			while len(data) < payload_size:
				data += conn.recv(BUFFER_SIZE)
				
			packed_msg_size = data[:payload_size]
			data = data[payload_size:]
			msg_size = struct.unpack(">L", packed_msg_size)[0]

			while len(data) < msg_size:
				data += conn.recv(4096)

			print ('Second Frame Received in Server')
			frame_data = data[:msg_size]
			data = data[msg_size:]
			frame=pickle.loads(frame_data, fix_imports=True, encoding="bytes")
			frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)
			cv2.imwrite(os.getcwd() + image_saved_path + '/side.png', frame)

			process_image()

			if os.path.isfile(os.getcwd() + image_process_path + '/front.png') and os.path.isfile(os.getcwd() + image_process_path + '/side.png'):
				conn.send(b"Files received and processed in the Server. Sharing the processed image with Client")	
				print ('LEAVING function "receive_images_from_client"')
				break


		except Exception as e:
			print ('In function "receive_images_from_client", Second Exception')
			print (e)
	

def transfer_images_to_client():
	print ('In Transfer Function')
	global global_s, conn, client_IP

	while 1:
		try:
			if os.path.isfile(os.getcwd() + image_process_path + '/front.png') and os.path.isfile(os.getcwd() + image_process_path + '/side.png'):

				file_name = '/front.png'
				frame = cv2.imread(os.getcwd() + image_process_path + file_name)
				result, frame = cv2.imencode('.png', frame)
				data = pickle.dumps(frame, 0)
				size = len(data)
				conn.sendall(struct.pack(">L", size) + data)


				file_name = '/side.png'
				frame = cv2.imread(os.getcwd() + image_process_path + file_name)
				result, frame = cv2.imencode('.png', frame)
				data = pickle.dumps(frame, 0)
				size = len(data)
				conn.sendall(struct.pack(">L", size) + data)
				print ('LEAVING function "transfer_images_to_client"')
				break

		
		except Exception as e:
			print ('In function "transfer_images_to_client", First Exception')
			print (e)





if __name__ == "__main__": 


	while 1:
		receive_images_from_client()
		transfer_images_to_client()
		global_s.close()
		time.sleep(5.0)
		global_s = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
		global_s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)







