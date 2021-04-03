import cv2
import streamlit as st

st.title("Webcam Live Feed")
feed = st.checkbox('Feed')
FRAME_WINDOW = st.image([])
camera = cv2.VideoCapture(0)

while feed:
    _, frame = camera.read()
    frame = cv2.flip(frame, 1) 
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    FRAME_WINDOW.image(frame)
else:
    st.write('Stopped')