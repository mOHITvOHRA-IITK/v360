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
# SERVER_IP = '74.82.31.134'

# My private IP
SERVER_IP = '172.24.218.179'


PORT = 5000
BUFFER_SIZE = 4096

global global_s
global_s = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
global_s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)




def transfer_images_to_server(front, side, height_feet, height_inch):
	print ('In Transfer Function')
	global global_s

	while 1:
		try:
	
			global_s.connect((SERVER_IP, PORT))


			## Create a dictionary of input data and send to the server
			imageDict = {'front_image': front, 'side_image': side, 'feet': height_feet, 'inch': height_inch}
			pickleData = pickle.dumps(imageDict)
			global_s.sendall(struct.pack('>I', len(pickleData)))
			global_s.sendall(pickleData)
			

			## Receive the processed data from server
			data = b""
			data_size = struct.unpack('>I', global_s.recv(4))[0]
			while len(data) < data_size:
				data += global_s.recv(BUFFER_SIZE)
		
			imageDict = pickle.loads(data, fix_imports=True, encoding="bytes")
			return imageDict


		except Exception as e:
			print ('In function "transfer_images_to_server", First Exception')
			print (e)
			



if __name__ == "__main__":
	while 1:
		file_name = '/front.png'
		front = cv2.imread(os.getcwd() + image_saved_path + file_name)
		file_name = '/side.png'
		side = cv2.imread(os.getcwd() + image_saved_path + file_name)
		height_feet = 5
		height_inch = 6

		imageDict = transfer_images_to_server(front, side, height_feet, height_inch)

		cv2.imshow('front', imageDict['front_image'])
		cv2.imshow('side', imageDict['side_image'])
		print ('waist', imageDict['waist'])
		print ('chest', imageDict['chest'])
		print ('thigh', imageDict['thigh'])
		print ('front_sleeve_in_cm', imageDict['front_sleeve_in_cm'])
		print ('dis_in_cm', imageDict['dis_in_cm'])
		cv2.waitKey(1)	


		global_s.close()
		time.sleep(5.0)
		global_s = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
		global_s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)


