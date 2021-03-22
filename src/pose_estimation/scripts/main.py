from class_definations import vision_demo_class
import argparse


parser = argparse.ArgumentParser()
parser.add_argument('-s', '--image_timer_val', help='Set the countdown for saving current image', type=int, default=5)
args = parser.parse_args()


o = vision_demo_class(args.image_timer_val)

while True:
	o.contactless_GUI()