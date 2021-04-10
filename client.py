import socket
import time
import streamlit as st
import cv2
import os
from client_class import *





st.title("Visual Try On")

live_feed, _, options = st.beta_columns([9, 1, 9])


with options:
	help_ = st.checkbox('Help')
	
	help_text = st.empty()
	if help_:
		help_text.text('1. Place camera parallel to ground at sufficient height. \n2. Store front and side human body images. \n3. Store images when complete body lies at centre of frame. \n4. For front face, arms should be parallel to ground like T shape, and legs should be at 60 degrees seperated. \n5. For side pose, arm should be parallel to groung and both arms should be on your front side. \n6. Feed your height and process')
	else:
		help_text.empty()




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
		create_sender_obj()
		load_images_on_server()
		receive_processed_images_from_server()

		



with live_feed:

	feed = st.checkbox('Live Feed')

	if feed:
		initialize_webcam()
	else:
		stop_webcam()


	FRAME_WINDOW = st.image([])
	key_count = 0
	height_text = st.empty()
	save_text = st.empty()
	image_save_status_text = st.empty()

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


		



		waist, chest, thigh, front_sleeve_in_cm, dis_in_cm, image, side_image = get_human_info()

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
		

