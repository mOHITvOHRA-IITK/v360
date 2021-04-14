import socket
import time
import streamlit as st
import cv2
import os
from client_class import *





st.title("Visual Try On")

live_feed, _, options = st.beta_columns([9, 1, 9])
create_sender_obj()

with options:
	help_ = st.checkbox('Help')
	
	help_line1 = st.empty()
	help_line2 = st.empty()
	help_line3 = st.empty()
	help_line4 = st.empty()
	images_st = st.empty()
	help_line5 = st.empty()
	if help_:
		help_line1.text('1. Click on "Live Feed" to start the \ncamera.')
		help_line2.text('2. Place camera parallel to ground \nat the waist height.')
		help_line3.text('3. Feed your height.')
		help_line4.text('4. Store front and side human body \nimages as shown below.')

		image_example_path = '/help_images'

		if os.path.isfile(os.getcwd() + image_example_path + '/front.png') and os.path.isfile(os.getcwd() + image_example_path + '/side.png'):
			image = cv2.imread(os.getcwd() + image_example_path + '/front.png')
			image1 = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
		
			image = cv2.imread(os.getcwd() + image_example_path + '/side.png')
			image2 = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

			h,w,c = np.array(image).shape
			blank_image = 255*np.ones([int(h/3), w,c], np.uint8)
			h,w,c = np.array(blank_image).shape
			images_st.image([image1, blank_image, image2], caption=['Front image', '', 'Side image'], width=96)


		help_line5.text('5. Press "Process" button')
		




	height = st.checkbox('Height')

	feet_slider = st.empty()
	inch_slider = st.empty()

	if height:
		local_height_feet = feet_slider.slider('feets', min_value=0, max_value=7, value=None)
		local_height_inch = inch_slider.slider('inches', min_value=0, max_value=12, value=None)
		
		if local_height_feet > 0:
			store_height_feet(local_height_feet)
			store_height_inch(local_height_inch)

	else:
		feet_slider.empty()
		inch_slider.empty()


	save = st.checkbox('Save')

	save_front_button = st.empty()
	save_side_button = st.empty()

	if save:
		s_f_i = save_front_button.button('save_front_image')
		if s_f_i:
			reset_image_front_status()
			start_count_down()

		s_s_i = save_side_button.button('save_side_image')
		if s_s_i:
			reset_image_side_status()
			start_count_down()

	else:
		stop_count_down()
		save_front_button.empty()
		save_side_button.empty()





	process = st.button('Process')
	if process:
		imageDict = transfer_data_to_server()
		store_imageDict(imageDict)

		



with live_feed:

	feed = st.checkbox('Live Feed')

	if feed:
		initialize_webcam()
	else:
		stop_webcam()


	FRAME_WINDOW = st.image([])
	key_count = 0

	height_text = st.empty()
	image_save_status_text = st.empty()
	save_text = st.empty()

	waist_text = st.empty()
	chest_text = st.empty()
	thigh_text = st.empty()
	sleeve_text = st.empty()
	length_text = st.empty()
	image_widget = st.empty()
	side_image_widget = st.empty()

	while feed:

		frame = get_current_feed()
		if np.array(frame).size == 0:
			break

		frame = cv2.flip(frame, 1) 
		color_converted_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
		FRAME_WINDOW.image(color_converted_frame, caption='Live Feed')

		key_count += 1

		#### Visualizing the height
		height_inch, height_feet = get_height_feet_inch()
		string_data = str(height_feet) + ' feet, ' + str(height_inch) + ' inch'
		input = height_text.text_input('height', value=string_data, key=key_count)

		
		#### Visualizing the images status	
		if check_image_status():
			string_data = 'Images are loaded'
			input = image_save_status_text.text_input('image save status', value=string_data, key=key_count)
		else:
			string_data = 'Need To save images'
			input = image_save_status_text.text_input('image save status', value=string_data, key=key_count)

		count_down = get_countdown()
		empty_string_data = '***********'
		string_data = 'saving image in ' +  str(count_down) + ' seconds'
		if count_down == -1:
			input = save_text.text_input('image countdown', value=empty_string_data, key=key_count)
		else:
			input = save_text.text_input('image countdown', value=string_data, key=key_count)
			if count_down == 0:
				save_images(frame)
				stop_count_down()


		



		imageDict, info_updated = get_imageDict()
		if info_updated:
			waist = imageDict['waist']
			chest = imageDict['chest']
			thigh = imageDict['thigh']
			front_sleeve_in_cm = imageDict['front_sleeve_in_cm']
			dis_in_cm = imageDict['dis_in_cm']
			image = imageDict['front_image']
			side_image = imageDict['side_image']



			if waist > 0:
				string_data = 'waist ' + str(round(waist)) + '(cm), ' + str(round(waist/2.54)) + '(inches)'
				input = waist_text.text_input('waist', value=string_data, key=key_count)

			if chest > 0:
				string_data = 'chest ' + str(round(chest)) + '(cm), ' + str(round(chest/2.54)) + '(inches)'
				input = chest_text.text_input('chest', value=string_data, key=key_count)

			if thigh > 0:
				string_data = 'thigh ' + str(round(thigh)) + '(cm), ' + str(round(thigh/2.54)) + '(inches)'
				input = thigh_text.text_input('thigh', value=string_data, key=key_count)

			if front_sleeve_in_cm > 0:
				string_data = 'front_sleeve ' + str(round(front_sleeve_in_cm)) + '(cm), ' + str(round(front_sleeve_in_cm/2.54)) + '(inches)'
				input = sleeve_text.text_input('front_sleeve', value=string_data, key=key_count)

			if dis_in_cm > 0:
				string_data = 'length ' + str(round(dis_in_cm)) + '(cm), ' + str(round(dis_in_cm/2.54)) + '(inches)'
				input = length_text.text_input('length', value=string_data, key=key_count)


			if np.array(image).size != 0:
				image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
				image_widget.image(image, caption='Front image')


			if np.array(side_image).size != 0:
				side_image = cv2.cvtColor(side_image, cv2.COLOR_BGR2RGB)
				side_image_widget.image(side_image, caption='Side image')
		

