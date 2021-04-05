import os
import cv2
from io import BytesIO
from PIL import Image
import base64
import streamlit as st
from streamlit_class import *

def get_image_download_link(img):

	im_pil = Image.fromarray(img)
	buffered = BytesIO()
	im_pil.save(buffered, format="JPEG")
	img_str = base64.b64encode(buffered.getvalue()).decode()
	href = f'<a href="data:file/jpg;base64,{img_str}">Download result</a>'
	return href



st.title("Server")

image_saved_path = '/streamlit_saved_images'
if os.path.isdir(os.getcwd() + image_saved_path) == False:
	os.mkdir(os.getcwd() + image_saved_path)

image_placeholder = st.empty()
local_image, status = visualize_specif_image()

if status:
	st.image(local_image, caption='Uploaded Image', use_column_width=True)
	st.markdown(get_image_download_link(local_image), unsafe_allow_html=True)


download_button = st.empty()
download = download_button.button('download_image')
if download:
	print ('image will be downloaded')
	uploaded_image = save_uploaded_image(os.getcwd() + image_saved_path + '/1.png')





file_uploader_button = st.empty()	
uploaded_files = file_uploader_button.file_uploader('upload saved images', accept_multiple_files=True)
for uploaded_file in uploaded_files:
	st.image(uploaded_file, caption='Uploaded Image', use_column_width=True)
	file_details = {"FileName":uploaded_file.name,"FileType":uploaded_file.type,"FileSize":uploaded_file.size}
	st.write(file_details)

	file_bytes = np.asarray(bytearray(uploaded_file.read()), dtype=np.uint8)
	opencv_image = cv2.imdecode(file_bytes, 1)
	get_uploaded_image(opencv_image)

	



    
     





		
		

