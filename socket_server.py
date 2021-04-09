# import socket ,errno          
# import cv2
# import numpy as np
# import sys
# import base64
# import numpy as np

# serv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# HOST = ''	# Symbolic name meaning all available interfaces
# PORT = 8080	# Arbitrary non-privileged port

# try:
# 	serv.bind((HOST, PORT))
# except socket.error as msg:
# 	print ('Bind failed.Please run this server after some time')
# 	sys.exit()


# serv.listen(10)

# SEPARATOR = "<SEPARATOR>"




# # while True:
# #     conn, addr = serv.accept()
# #     from_client = ''
# #     while True:
# #         data = conn.recv(4096)
# #         if not data: 
# #         	break

# #         from_client += str(data)
# #         print (from_client)
# #         conn.send(b"I am SERVER")
# #     conn.close()
# #     print ('client disconnected')


# while 1:
# 	try:
# 		BUFFER_SIZE = 4096
# 		conn, addr = serv.accept()
# 		received = conn.recv(BUFFER_SIZE).decode()
# 		filename, filesize = received.split(SEPARATOR)
# 		filesize = int(filesize)
# 		print (filename, filesize)

# 		jpg_as_text = ''

# 		b = int(filesize/1000)

# 		while True:
# 			bytes_read = conn.recv(BUFFER_SIZE).decode('utf-8')
# 			jpg_as_text += bytes_read


# 			filesize2 = str(sys.getsizeof(jpg_as_text))
# 			# print ('filesize2', filesize2)

# 			a = int(int(filesize2)/1000)
			


# 			if a == b:
# 				# b = 100
# 				# text_file = open("/home/mohit/v360/server_data.txt", "w")
# 				# n = text_file.write(jpg_as_text)
# 				# text_file.close()
# 				# print ('jpg_as_text\n', jpg_as_text)
# 				# print (type(jpg_as_text))
# 				im_bytes = base64.b64decode(jpg_as_text)
# 				im_arr = np.frombuffer(im_bytes, dtype=np.uint8)  # im_arr is one-dim Numpy array
# 				print (im_arr.shape)
# 				img = cv2.imdecode(im_arr, flags=cv2.IMREAD_COLOR)
# 				print (np.array(img).shape)
				

# 				jpg_as_text = bytes(jpg_as_text, 'utf-8')
# 				nparr = np.frombuffer(jpg_as_text, np.uint8)
# 				print (nparr)
# 				img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
# 				cv2.imshow('img', img)
# 				cv2.waitKey(100)



				



# 	except Exception as e:
# 		print ('in except loop')
# 		print (e)
# 		print ('Terminate the Client')
# 		conn.send(b"Please disconnet with server")		
# 		# conn.close()
#     












import socket
import sys
import cv2
import pickle
import numpy as np
import struct
import zlib
import os
import _thread
from streamlit_server_class import *

HOST=''
PORT=8081

global_s = socket.socket(socket.AF_INET,socket.SOCK_STREAM)



image_saved_path = '/streamlit_server_images'
if os.path.isdir(os.getcwd() + image_saved_path) == False:
	os.mkdir(os.getcwd() + image_saved_path)


image_process_path = '/streamlit_server_process_images'
if os.path.isdir(os.getcwd() + image_saved_path) == False:
	os.mkdir(os.getcwd() + image_saved_path)

BUFFER_SIZE = 4096


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
			cv2.imwrite(os.getcwd() + image_saved_path + '/front.png', frame)


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
			cv2.imwrite(os.getcwd() + image_saved_path + '/side.png', frame)

			process_image()


		except Exception as e:
			print ('in except loop')
			print (e)
			print ('Terminate the Client')
			conn.send(b"Please disconnet with server")		
			# conn.close()



def transfer_images_to_other_system(s):
	
	if os.path.isfile(os.getcwd() + image_process_path + '/front.png') and os.path.isfile(os.getcwd() + image_process_path + '/side.png'):
		s.connect(('0.0.0.0', PORT))

		file_name = '/front.png'
		frame = cv2.imread(os.getcwd() + image_process_path + file_name)
		result, frame = cv2.imencode('.png', frame)
		data = pickle.dumps(frame, 0)
		size = len(data)
		s.sendall(struct.pack(">L", size) + data)


		file_name = '/side.png'
		frame = cv2.imread(os.getcwd() + image_process_path + file_name)
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


