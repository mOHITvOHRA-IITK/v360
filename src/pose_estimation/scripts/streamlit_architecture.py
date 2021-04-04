import os
import cv2
import streamlit as st
from streamlit_class import *


st.title("Architecture")

image_saved_path = './streamlit_images'

live_feed, _, options = st.beta_columns([4, 1, 1])

with options:
	height = st.checkbox('Height')
	save = st.checkbox('Save')
	upload = st.checkbox('Upload')

	feet_slider = st.empty()
	inch_slider = st.empty()

	if height:

		local_height_feet = feet_slider.slider('feets', min_value=0, max_value=7, value=None)
		local_height_inch = inch_slider.slider('inches', min_value=0, max_value=12, value=None)
		
		if local_height_feet > 0:
			store_height_feet(local_height_feet)
			store_height_inch(local_height_inch)

		height_feet, height_inch = get_height_feet_inch()
	else:
		feet_slider.empty()
		inch_slider.empty()


	save_front_button = st.empty()
	save_side_button = st.empty()

	if save:
		s_f_i = save_front_button.button('save_front_image')
		if s_f_i:
			start_count_down()
			save_front_side_images(True, None)

			if os.path.isfile(image_saved_path + '/front.png'):
				os.remove(image_saved_path + '/front.png')
			if os.path.isfile(image_saved_path + '/side.png'):
				os.remove(image_saved_path + '/side.png')
			
			

		s_s_i = save_side_button.button('save_side_image')
		if s_s_i:
			start_count_down()
	else:
		stop_count_down()
		save_front_button.empty()
		save_side_button.empty()




	file_uploader_button = st.empty()
	if upload:
		uploaded_files = file_uploader_button.file_uploader('upload saved images', accept_multiple_files=True)
		for uploaded_file in uploaded_files:
			st.image(uploaded_file, caption='Uploaded Image', use_column_width=True)


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

	while feed:
		# visualize_images()
		frame = get_current_feed()
		frame = cv2.flip(frame, 1) 
		frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
		FRAME_WINDOW.image(frame)

		key_count += 1

		if height:
			string_data = str(height_feet) + ' feet, ' + str(height_inch) + ' inch'
			input = height_text.text_input('height', value=string_data, key=key_count)

		if save:
			
			if check_both_images_saved():
				string_data = os.getcwd() + image_saved_path
				input = image_save_status_text.text_input('image save status', value=string_data, key=key_count)
			else:
				string_data = 'Need To save images'
				input = image_save_status_text.text_input('image save status', value=string_data, key=key_count)

				count_down = get_countdown()
				empty_string_data = '***********'
				string_data = 'saving image in ' +  str(count_down) + ' seconds'
				if count_down > -1:
					input = save_text.text_input('image countdown', value=string_data, key=key_count)
					if count_down == 0:
						frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
						if os.path.isfile(image_saved_path + '/front.png') == False:
							cv2.imwrite(image_saved_path + '/front.png', frame)

						elif os.path.isfile(image_saved_path + '/side.png') == False:
							cv2.imwrite(image_saved_path + '/side.png', frame)

						save_front_side_images(False, frame)
				else:
					input = save_text.text_input('image countdown', value=empty_string_data, key=key_count)


		
			
		

