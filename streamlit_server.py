import imagezmq
from streamlit_server_class import *


imageHub = imagezmq.ImageHub()


while True:
	print ('server')

	(client_info, frame) = imageHub.recv_image()
	imageHub.send_reply(b'OK')

	# cv2.imshow('frame', frame)
	# cv2.waitKey(10)

	save_and_send_image(frame, client_info)