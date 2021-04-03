import cv2
import streamlit as st
from streamlit_class import *
# from streamlit.ScriptRequestQueue import RerunData
# from streamlit.ScriptRunner import RerunException
# from streamlit.server.Server import Server
# import streamlit.ReportThread as ReportThread


# def rerun():
#     """Rerun a Streamlit app from the top!"""
#     widget_states = _get_widget_states()
#     raise RerunException(RerunData(widget_states))


height_feet = 0
height_inch = 0

count = 0

height_feet, height_inch = get_height_feet_inch()
store_height_feet(height_feet)
store_height_inch(height_inch)


st.title("Architecture")

height = st.checkbox('Height')

if height:
	height_feet, height_inch = get_height_feet_inch()
	h_f_i = st.button('height_feet_increase')
	if h_f_i:
	    height_feet += 1



	h_f_d = st.button('height_feet_decrease')
	if h_f_d:
	    height_feet -= 1
	    if height_feet < 0:
	    	height_feet = 0



	h_i_i = st.button('height_inch_increase')
	if h_i_i:
	    height_inch += 1
	    height_inch = height_inch % 12


	h_i_d = st.button('height_inch_decrease')
	if h_i_d:
	    height_inch -= 1
	    if height_inch < 0:
	    	height_inch = 0


	store_height_feet(height_feet)
	store_height_inch(height_inch)

placeholder2 = st.empty()
key_count = 0
string_data = str(height_feet) + ' feet, ' + str(height_inch) + ' inch'
input = placeholder2.text_input('height', value=string_data, key=key_count)


save = st.checkbox('Save')

if save:
	s_f_i = st.button('save_front_image')
	if s_f_i:
		start_count_down()


	s_s_i = st.button('save_side_image')
	if s_s_i:
		start_count_down()


count_down = get_countdown()
placeholder = st.empty()
key_count = 0
empty_string_data = '***********'
input = placeholder.text_input('image countdown', value=empty_string_data, key=key_count)
while(count_down > -1):

	if save == False:
		stop_count_down()
		break
	key_count += 1
	count_down = get_countdown()
	string_data = 'saving image in ' +  str(count_down) + ' seconds'
	input = placeholder.text_input('image countdown', value=string_data, key=key_count)
	print (string_data, end="\r")

	if count_down < 0:
		input = placeholder.text_input('image countdown', value=empty_string_data, key=key_count)





	