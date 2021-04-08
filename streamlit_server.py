import numpy as np
import imagezmq
import cv2


imageHub = imagezmq.ImageHub()

while True:
	(hostname, frame) = imageHub.recv_image()
	print ('receiving images from ' hostname )
	cv2.imshow('frame', frame)
	cv2.waitKey(1)