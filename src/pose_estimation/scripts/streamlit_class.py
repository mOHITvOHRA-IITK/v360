import time
import numpy as np

global height_feet, height_inch
height_feet = 0
height_inch = 0

global start_count, timer_start_flag, start_time
start_count = False
timer_start_flag = False
start_time = 0

countdown = 5

def store_height_feet(height_feet_var):
	global height_feet
	height_feet = height_feet_var
	



def store_height_inch(height_inch_var):
	global height_inch
	height_inch = height_inch_var
	



def get_height_feet_inch():
	global height_feet, height_inch
	return height_feet, height_inch


def stop_count_down():
	global start_count, timer_start_flag
	start_count = False
	timer_start_flag = False


def start_count_down():
	global start_count, timer_start_flag
	start_count = True
	timer_start_flag = False



def get_countdown():
	global timer_start_flag, start_time, start_count
	if start_count == True and timer_start_flag == False:
		timer_start_flag = True
		start_time = time.time()

	count_down = countdown - np.int( time.time() - start_time)
	# print ('countdown', countdown)
	# print ('time.time()', time.time())
	# print ('start_time', start_time)
	# print ('count_down', count_down)
	# print ('start_count', start_count)
	# print ()

	if count_down <0:
		timer_start_flag = False
		start_count = False

	if start_count == True:
		return count_down
	else:
		return -1