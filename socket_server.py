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
			print ('In bind loop')
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
			print ('In accept loop')
			conn, addr = global_s.accept()
			client_IP = addr[0]
			data = b""

			data_size = struct.unpack('>I', conn.recv(4))[0]

			while len(data) < data_size:
				data += conn.recv(BUFFER_SIZE)
			

			imageDict = pickle.loads(data, fix_imports=True, encoding="bytes")
			front = imageDict['front_image']
			cv2.imwrite(os.getcwd() + image_saved_path + '/front.png', front)

			side = imageDict['side_image']
			cv2.imwrite(os.getcwd() + image_saved_path + '/side.png', side)

			height_feet = imageDict['feet']
			height_inch = imageDict['inch']


			process_image()
			waist, chest, thigh, front_sleeve_in_cm, dis_in_cm, image, side_image = get_human_info(height_feet, height_inch)


			imageDict = {'front_image': image, 'side_image': side_image, 'waist': round(waist), 'chest': round(chest), 'thigh': round(thigh), 'front_sleeve_in_cm': round(front_sleeve_in_cm), 'dis_in_cm': round(dis_in_cm) }
			pickleData = pickle.dumps(imageDict)
			conn.sendall(struct.pack('>I', len(pickleData)))
			conn.sendall(pickleData)

			print ('LEAVING function "receive_images_from_client"')
			break


		except Exception as e:
			print ('In function "receive_images_from_client", Second Exception')
			print (e)
		
	



if __name__ == "__main__": 


	while 1:
		receive_images_from_client()
		global_s.close()
		time.sleep(5.0)
		global_s = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
		global_s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)







