import socket
import sys
import cv2
import pickle
import numpy as np
import struct
import zlib
import os
import _thread
import time



image_saved_path = '/streamlit_client_images'
if os.path.isdir(os.getcwd() + image_saved_path) == False:
	os.mkdir(os.getcwd() + image_saved_path)

SERVER_IP = '172.26.174.143'
HOST=''
PORT = 8081

BUFFER_SIZE = 4096

global global_s
global_s = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
global_s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

global receive_thread_task_complete, receive_thread_created, transfer_thread_task_complete, transfer_thread_created
receive_thread_task_complete = False
receive_thread_created = False
transfer_thread_task_complete = False
transfer_thread_created = False



def receive_images_from_other_system():
	print ('IN receiving Thread')
	global global_s, receive_thread_task_complete
	global_s.bind(('',PORT))
	global_s.listen(10)

	while 1:
		try:
			conn,addr = global_s.accept()
			data = b""
			payload_size = struct.calcsize(">L")

			while len(data) < payload_size:
				data += conn.recv(BUFFER_SIZE)
				
			packed_msg_size = data[:payload_size]
			data = data[payload_size:]
			msg_size = struct.unpack(">L", packed_msg_size)[0]

			while len(data) < msg_size:
				data += conn.recv(4096)

			print ('frame received')
			frame_data = data[:msg_size]
			data = data[msg_size:]
			frame=pickle.loads(frame_data, fix_imports=True, encoding="bytes")
			frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)
			# cv2.imshow('front', frame)


			while len(data) < payload_size:
				data += conn.recv(BUFFER_SIZE)
				
			packed_msg_size = data[:payload_size]
			data = data[payload_size:]
			msg_size = struct.unpack(">L", packed_msg_size)[0]

			while len(data) < msg_size:
				data += conn.recv(4096)

			print ('frame received')
			frame_data = data[:msg_size]
			data = data[msg_size:]
			frame=pickle.loads(frame_data, fix_imports=True, encoding="bytes")
			frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)
			# cv2.imshow('side', frame)

			conn.close()
			receive_thread_task_complete = True
			print ('LEAVING receiving Thread')
			break


		except Exception as e:
			print ('in client receive except')
			print (e)
				

	sys.exit()



def transfer_images_to_other_system():
	print ('In transfer Thread')
	global global_s, transfer_thread_task_complete

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
				transfer_thread_task_complete = True
				print ('LEAVING transfer Thread')
				break


		except Exception as e:
			print ('in client transfer except')
			print (e)
			
	sys.exit()



if __name__ == "__main__":
    while 1:

    	if transfer_thread_task_complete == False:
    		if transfer_thread_created == False:
    			transfer_thread_created = True
    			_thread.start_new_thread( transfer_images_to_other_system, () )
    	elif receive_thread_task_complete == False:
    		if receive_thread_created == False:

    			global_s.close()
    			time.sleep(5.0)
    			global_s = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    			global_s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    			receive_thread_created = True
    			_thread.start_new_thread( receive_images_from_other_system, () )



    	if receive_thread_task_complete and transfer_thread_task_complete:

    		global_s.close()
    		# time.sleep(10.0)
    		# global_s = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    		# global_s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    	# 	receive_thread_task_complete = False
    	# 	transfer_thread_task_complete = False
    	# 	receive_thread_created = False
    	# 	transfer_thread_created = False
