from flask import Flask, flash, request, redirect, url_for, render_template, Response
from werkzeug.utils import secure_filename
from class_definations import vision_demo_class
import argparse
import os

parser = argparse.ArgumentParser()
parser.add_argument('-s', '--image_timer_val', help='Set the countdown for saving current image', type=int, default=5)
parser.add_argument('-w', '--web_service', help='SSet the application status touse it either on browser or not', type=bool, default=True)
args = parser.parse_args()

UPLOAD_FOLDER = '/home/mohit/v360_ws/src/pose_estimation/scripts/images_flask'
ALLOWED_EXTENSIONS = set(['txt', 'pdf', 'png', 'jpg', 'jpeg', 'gif'])


app = Flask(__name__)
app.config['UPLOAD_FOLDER'] = UPLOAD_FOLDER

@app.route('/', methods=['GET', 'POST'])
def index():
	# rendering webpage
	return render_template('index.html')



def gen(o):
    while True:

    	# ###for vision_demo_class
        o.first_screen()
        frame = o.get_frame()
        
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n')


@app.route('/video_feed')
def video_feed():
    return Response(gen(vision_demo_class(args.image_timer_val, args.web_service)),
                    mimetype='multipart/x-mixed-replace; boundary=frame')



if __name__ == '__main__':
    # defining server ip address and port
    app.run(debug=True)












    