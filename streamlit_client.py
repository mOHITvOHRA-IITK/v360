import imagezmq
import argparse
import socket
import time
import cv2

ap = argparse.ArgumentParser()
ap.add_argument("-s", "--server_ip", required=True,
	help="ip address of the server to which the client will connect")
ap.add_argument("-p", "--server_port", required=True,
	help="port of the server to which the client will connect")
args = vars(ap.parse_args())



sender = imagezmq.ImageSender(connect_to="tcp://{}:{}".format(args["server_ip"], args["server_port"]))
client_host_name = socket.gethostname()


camera_object = cv2.VideoCapture(0)
time.sleep(2.0)

while (1):
	_, frame = camera_object.read()
	frame = cv2.flip(frame, 1)
	sender.send_image(client_host_name, frame)

