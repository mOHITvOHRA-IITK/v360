import cv2
import time
import numpy as np
import os
import torch
from PIL import Image
from tqdm import tqdm
from torch.utils.data import DataLoader
import torchvision.transforms as transforms
import networks
from utils.transforms import transform_logits
from datasets.simple_extractor_dataset import SimpleFolderDataset
from functions import get_measuremnets



image_saved_path = '/streamlit_images'
if os.path.isdir(os.getcwd() + image_saved_path) == False:
	os.mkdir(os.getcwd() + image_saved_path)


process_image_path = '/streamlit_process_images'
if os.path.isdir(os.getcwd() + process_image_path) == False:
	os.mkdir(os.getcwd() + process_image_path)


num_classes = 7
model = networks.init_model('resnet101', num_classes=num_classes, pretrained=None)
state_dict = torch.load(os.getcwd() + '/weights/' + 'exp-schp-201908270938-pascal-person-part.pth')['state_dict']
from collections import OrderedDict
new_state_dict = OrderedDict()
for k, v in state_dict.items():
    name = k[7:]  # remove `module.`
    new_state_dict[name] = v
model.load_state_dict(new_state_dict)
model.cuda()
model.eval()




global camera_object_start, camera_object
camera_object_start = False


global height_feet, height_inch
height_feet = 0
height_inch = 0

global start_count, timer_start_flag, start_time
start_count = False
timer_start_flag = False
start_time = 0


global front_image, front_image_flag, side_image, side_image_flag
front_image_flag = False
side_image_flag = False


global human_waist, human_chest, human_thigh, human_front_sleeve_in_cm, human_dis_in_cm, human_image, human_side_image
human_waist = -1
human_chest = -1
human_thigh = -1
human_front_sleeve_in_cm = -1
human_dis_in_cm = -1
human_image = np.array([])
human_side_image = np.array([])


global uploaded_image, uploaded_image_flag
uploaded_image_flag = False

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

	if count_down <0:
		timer_start_flag = False
		start_count = False

	if count_down == 0 and timer_start_flag:
		start_count = False
		timer_start_flag = False
		return 0

	if start_count == True:
		return count_down
	else:
		return -1




def initialize_webcam():
	global camera_object_start, camera_object

	if camera_object_start == False:
		camera_object_start = True
		camera_object = cv2.VideoCapture(0)



def get_current_feed():
	global camera_object_start, camera_object

	if camera_object_start == True:
		_, frame = camera_object.read()
		return frame
	else:
		return None



def stop_webcam():
	global camera_object_start, camera_object

	initialize_webcam()

	camera_object_start = False
	camera_object.release()




def save_front_side_images(reset, frame):
	global front_image, front_image_flag, side_image, side_image_flag

	if reset:
		front_image_flag = False
		side_image_flag = False

	else:

		if front_image_flag == False:
			front_image_flag = True
			front_image = frame

		elif side_image_flag == False:
			side_image_flag = True
			side_image = frame



def visualize_images():
	global front_image, front_image_flag, side_image, side_image_flag

	if front_image_flag:
		cv2.imshow('front_image', front_image)
		cv2.waitKey(1)
	
		
	if side_image_flag:
		cv2.imshow('side_image', side_image)
		cv2.waitKey(1)
	
		


def check_both_images_saved():
	global front_image_flag, side_image_flag
	if front_image_flag and side_image_flag:
		return True
	else:
		return False




def get_palette(num_cls):
    """ Returns the color map for visualizing the segmentation mask.
    Args:
        num_cls: Number of classes
    Returns:
        The color map
    """
    n = num_cls
    palette = [0] * (n * 3)
    for j in range(0, n):
        lab = j
        palette[j * 3 + 0] = 0
        palette[j * 3 + 1] = 0
        palette[j * 3 + 2] = 0
        i = 0
        while lab:
            palette[j * 3 + 0] |= (((lab >> 0) & 1) << (7 - i))
            palette[j * 3 + 1] |= (((lab >> 1) & 1) << (7 - i))
            palette[j * 3 + 2] |= (((lab >> 2) & 1) << (7 - i))
            i += 1
            lab >>= 3
    return palette




def process_image(feet, inch):

	if (os.path.isfile(os.getcwd() + image_saved_path + '/front.png') and os.path.isfile(os.getcwd() + image_saved_path + '/side.png')) :
		input_size = [512, 512]
		transform = transforms.Compose([transforms.ToTensor(), transforms.Normalize(mean=[0.406, 0.456, 0.485], std=[0.225, 0.224, 0.229])])
		dataset = SimpleFolderDataset(root=os.getcwd() + image_saved_path, input_size=input_size, transform=transform)
		dataloader = DataLoader(dataset)

		palette = get_palette(num_classes)

		with torch.no_grad():
			for idx, batch in enumerate(tqdm(dataloader)):
				image, meta = batch
				img_name = meta['name'][0]
				c = meta['center'].numpy()[0]
				s = meta['scale'].numpy()[0]
				w = meta['width'].numpy()[0]
				h = meta['height'].numpy()[0]

				output = model(image.cuda())
				upsample = torch.nn.Upsample(size=input_size, mode='bilinear', align_corners=True)
				upsample_output = upsample(output[0][-1][0].unsqueeze(0))
				upsample_output = upsample_output.squeeze()
				upsample_output = upsample_output.permute(1, 2, 0)  # CHW -> HWC

				logits_result = transform_logits(upsample_output.data.cpu().numpy(), c, s, w, h, input_size=input_size)
				parsing_result = np.argmax(logits_result, axis=2)
				parsing_result_path = os.path.join(os.getcwd() + process_image_path, img_name[:-4] + '.png')
				output_img = Image.fromarray(np.asarray(parsing_result, dtype=np.uint8))
				output_img.putpalette(palette)
				output_img.save(parsing_result_path)

		actual_height = (12*feet + inch) * 2.54 # in cm
		i1_path = os.getcwd() + image_saved_path + '/front.png'
		i1 = cv2.imread(i1_path)
		m1_path = os.getcwd() + process_image_path + '/front.png'
		m1 = cv2.imread(m1_path)
		i2_path = os.getcwd() + image_saved_path + '/side.png'
		i2 = cv2.imread(i2_path)
		m2_path = os.getcwd() + process_image_path + '/side.png'
		m2 = cv2.imread(m2_path)
		status, waist, chest, thigh, front_sleeve_in_cm, dis_in_cm, image, side_image = get_measuremnets(i1, m1, i2, m2, actual_height, True)

		return status, waist, chest, thigh, front_sleeve_in_cm, dis_in_cm, image, side_image

	else:
		return False, -1, -1, -1, -1, -1, None, None




def store_human_info(waist, chest, thigh, front_sleeve_in_cm, dis_in_cm, image, side_image):
	global human_waist, human_chest, human_thigh, human_front_sleeve_in_cm, human_dis_in_cm, human_image, human_side_image

	human_waist = waist
	human_chest = chest
	human_thigh = thigh
	human_front_sleeve_in_cm = front_sleeve_in_cm
	human_dis_in_cm = dis_in_cm
	human_image = image
	human_side_image = side_image



def get_human_info():
	global human_waist, human_chest, human_thigh, human_front_sleeve_in_cm, human_dis_in_cm, human_image, human_side_image
	return human_waist, human_chest, human_thigh, human_front_sleeve_in_cm, human_dis_in_cm, human_image, human_side_image






def get_uploaded_image(image):
	global uploaded_image, uploaded_image_flag
	if uploaded_image_flag == False:
		uploaded_image = image
		uploaded_image_flag = True



def save_uploaded_image(path):
	global uploaded_image, uploaded_image_flag
	if uploaded_image_flag:
		uploaded_image_flag = False
		# cv2.imwrite(path, uploaded_image)
		return uploaded_image





def visualize_specif_image():
	global uploaded_image, uploaded_image_flag
	if uploaded_image_flag:
		frame = cv2.cvtColor(uploaded_image, cv2.COLOR_RGB2BGR)
		return frame, True
	else:
		return np.array([]), False

	
