from class_definations import vision_demo_class
import argparse


parser = argparse.ArgumentParser()
parser.add_argument('-s', '--image_timer_val', help='Set the countdown for saving current image', type=int, default=5)
parser.add_argument('-w', '--web_service', help='SSet the application status touse it either on browser or not', type=bool, default=False)
args = parser.parse_args()

o = vision_demo_class(args.image_timer_val, args.web_service)

while True:
	# o.contactless_GUI()
	o.first_screen()
