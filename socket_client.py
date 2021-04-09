import socket
import sys
import cv2
import pickle
import numpy as np
import struct
import zlib
import os
import _thread



image_saved_path = '/streamlit_client_images'
if os.path.isdir(os.getcwd() + image_saved_path) == False:
	os.mkdir(os.getcwd() + image_saved_path)


HOST=''
PORT=8081

global_s = socket.socket(socket.AF_INET,socket.SOCK_STREAM)

def receive_images_from_other_system(s):
	s.bind((HOST,PORT))
	s.listen(10)

	while 1:
		try:
			conn,addr=s.accept()
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
			cv2.imshow('front', frame)


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
			cv2.imshow('side', frame)
			cv2.waitKey(0)


		except Exception as e:
			print ('in except loop')
			print (e)
			print ('Terminate the Client')
			conn.send(b"Please disconnet with server")		
			# conn.close()





def transfer_images_to_other_system(s):
	
	if os.path.isfile(os.getcwd() + image_saved_path + '/front.png') and os.path.isfile(os.getcwd() + image_saved_path + '/side.png'):
		s.connect(('0.0.0.0', PORT))

		file_name = '/front.png'
		frame = cv2.imread(os.getcwd() + image_saved_path + file_name)
		result, frame = cv2.imencode('.png', frame)
		data = pickle.dumps(frame, 0)
		size = len(data)
		s.sendall(struct.pack(">L", size) + data)


		file_name = '/side.png'
		frame = cv2.imread(os.getcwd() + image_saved_path + file_name)
		result, frame = cv2.imencode('.png', frame)
		data = pickle.dumps(frame, 0)
		size = len(data)
		s.sendall(struct.pack(">L", size) + data)
		s.close()




if __name__ == "__main__":
    _thread.start_new_thread( receive_images_from_other_system, (global_s,) )
    _thread.start_new_thread( transfer_images_to_other_system, (global_s,) )

    while 1:
    	pass
