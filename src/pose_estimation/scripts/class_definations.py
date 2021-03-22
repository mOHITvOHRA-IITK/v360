import cv2
import numpy as np
import time
import os
from termcolor import colored
import torch
from PIL import Image
from tqdm import tqdm
from torch.utils.data import DataLoader
import torchvision.transforms as transforms
import networks
from utils.transforms import transform_logits
from datasets.simple_extractor_dataset import SimpleFolderDataset








radius_fraction = 0.06
alpha = 0.4



images_folder_path = './images/'
output_path = './output/'


model = networks.init_model('resnet101', num_classes=7, pretrained=None)
state_dict = torch.load('/home/mohit/Self-Correction-Human-Parsing-master/exp-schp-201908270938-pascal-person-part.pth')['state_dict']
from collections import OrderedDict
new_state_dict = OrderedDict()
for k, v in state_dict.items():
    name = k[7:]  # remove `module.`
    new_state_dict[name] = v
model.load_state_dict(new_state_dict)
model.cuda()
model.eval()


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







def write_data(current_frame, text, rec_x, rec_y, rec_w, rec_h, text_offset_x, text_offset_y, text_format1, text_format2, text_color):

	image_height, image_width, _ = current_frame.shape

	overlay = current_frame.copy()
	cv2.rectangle(current_frame,( np.int(rec_x*image_width), np.int(rec_y*image_height) ), ( np.int((rec_x + rec_w)*image_width), np.int((rec_y + rec_h)*image_height) ), (0,0,0), -1)
	current_frame = cv2.addWeighted(overlay, alpha, current_frame, 1 - alpha, 0)
	cv2.putText(current_frame, str(text),  ( (np.int((rec_x + text_offset_x)*image_width)), (np.int((rec_y + text_offset_y)*image_height)) ) , cv2.FONT_HERSHEY_SIMPLEX, text_format1, text_color, text_format2, cv2.LINE_AA)

	return current_frame


global mouseX, mouseY
mouseX = -200
mouseY = -200




def get_mouse_click(event,x,y,flags,param):
    global mouseX, mouseY
    if event == cv2.EVENT_LBUTTONDOWN:
        mouseX,mouseY = x,y
        






class vision_demo_class:

	

	def __init__(self, image_timer_value):



		self.cap = cv2.VideoCapture(0)
		if not self.cap.isOpened():
		    raise IOError("Cannot open webcam")
		    exit()

		# self.cap.set(cv2.CAP_PROP_FPS, 30)
		# self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
		# self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

		_, frame = self.cap.read()
		self.image_height, self.image_width, layers = frame.shape

		self.exit_point = np.array([np.int(0.15*self.image_width), np.int(0.15*self.image_height)], dtype=int)
		self.save_point = np.array([np.int(0.38*self.image_width), np.int(0.15*self.image_height)], dtype=int)
		self.info_point = np.array([np.int(0.62*self.image_width), np.int(0.15*self.image_height)], dtype=int)
		self.process_point = np.array([np.int(0.85*self.image_width), np.int(0.15*self.image_height)], dtype=int)

		self.exit_button = False
		self.save_button = False
		self.info_button = False
		self.process_button = False
		
		self.current_img_timer = 0
		self.img_timer_flag = False
		self.image_timer_value = image_timer_value
		

		cv2.namedWindow('Human_measurement')
		cv2.setMouseCallback('Human_measurement', get_mouse_click)		

	
	

	def get_current_frame(self):

		_, frame = self.cap.read()
		frame = cv2.flip(frame, 1) 

		self.timer = cv2.getTickCount()     # start the timer for the calculation of fps in function view_frame

		self.current_frame = frame.copy()  
		self.current_frame2 = frame.copy()  



	def view_frame(self):
	
		self.fps = cv2.getTickFrequency() / (cv2.getTickCount() - self.timer)        # fps calculation with timer start in function get_current_frame.
		# self.current_frame = write_data(self.current_frame, 'fps:' + str(int(self.fps)), 0.05, 0.74, 0.17, 0.10, 0.01, 0.07, 1, 2, (255, 0, 255))
		cv2.imshow('Human_measurement', self.current_frame)
		cv2.waitKey(1)


	
	def buttons(self, array, background_color, txt, text_color):

		overlay = self.current_frame.copy()
		cv2.circle(self.current_frame, (array[0], array[1]), np.int(radius_fraction*self.image_width), background_color, -1)
		self.current_frame = cv2.addWeighted(overlay, alpha, self.current_frame, 1 - alpha, 0)
		cv2.putText(self.current_frame, txt,  ( (array[0]- np.int(2*radius_fraction*self.image_width/3)), (array[1] + np.int(radius_fraction*self.image_height/3)) ) , cv2.FONT_HERSHEY_SIMPLEX, 1, text_color, 3, cv2.LINE_AA)



	def button_selected(self, array):

		global mouseX, mouseY

		lw_pt_x = array[0] - mouseX
		lw_pt_y = array[1] - mouseY
		dis_lw_pt = np.sqrt(lw_pt_x*lw_pt_x + lw_pt_y*lw_pt_y)
		
	
		if (dis_lw_pt < np.int(radius_fraction*self.image_width)):
			mouseX = -200
			mouseY = -200
			return True
		

		return False



	def contactless_GUI(self):

		self.get_current_frame()


		if ( (self.exit_button == False and self.save_button == False) and (self.info_button == False and self.process_button == False) ):
			self.buttons(self.exit_point, (0, 0,255), 'Ext', (255, 255, 255))
			self.exit_button = self.button_selected(self.exit_point)

			self.buttons(self.save_point, (255, 0, 0), 'Sav', (255, 255, 255))
			self.save_button = self.button_selected(self.save_point)

			self.buttons(self.info_point, (255, 0, 0), 'Inf', (255, 255, 255))
			self.info_button = self.button_selected(self.info_point)

			self.buttons(self.process_point, (255, 0, 0), 'Prc', (255, 255, 255))
			self.process_button = self.button_selected(self.process_point)

		else:
			if self.exit_button:
				print()
				print (colored("********************************", 'cyan'))
				print (colored("You have touched the exit button", 'red'))
				print (colored("********************************", 'cyan'))
				self.cap.release()
				cv2.destroyAllWindows()
				exit()


			if self.save_button:
				self.buttons(self.save_point, (0, 255, 0), 'Sav', (255, 255, 255))
				self.save_img()


			if self.info_button:
				self.buttons(self.info_point, (0, 255, 0), ' <', (255, 255, 255))
				self.Print_info()


			if self.process_button:
				self.buttons(self.process_point, (0, 255, 0), 'Prc', (255, 255, 255))
				self.Process_image()




		self.view_frame()



	def save_img(self):


		if self.img_timer_flag == False:
			self.current_img_timer = time.time()
			self.img_timer_flag = True


		count_down = self.image_timer_value - np.int( time.time() - self.current_img_timer)
		
		
		if ( self.img_timer_flag and count_down <= self.image_timer_value ):
			self.current_frame = write_data(self.current_frame, 'Saving image in ' + str(int(count_down)) + ' secs ', 0.0, 0.4, 1.0, 0.15, 0.04, 0.12, 2, 2, (255,255,255))

			images_folder_list = os.listdir(images_folder_path)
			img_num = len(images_folder_list) % 2
			if count_down == 0:
				if img_num == 0:
					cv2.imwrite(images_folder_path + '/front.png', self.current_frame2)
					os.remove(images_folder_path + '/side.png')
				else:
					cv2.imwrite(images_folder_path + '/side.png', self.current_frame2)
				self.img_timer_flag = False
				self.save_button = False



	def Print_info(self):
		self.current_frame = write_data(self.current_frame, 'Store front and side face images', 0.0, 0.25, 1.0, 0.15, 0.04, 0.12, 1, 2, (255,255,255))
		self.current_frame = write_data(self.current_frame, 'For Front, stand in T pose', 0.0, 0.4, 1.0, 0.15, 0.04, 0.12, 1, 2, (255,255,255))
		self.current_frame = write_data(self.current_frame, 'For side, rotate about 90 in T pose', 0.0, 0.55, 1.0, 0.15, 0.04, 0.12, 1, 2, (255,255,255))
		self.current_frame = write_data(self.current_frame, 'Sav button saves single images', 0.0, 0.7, 1.0, 0.15, 0.04, 0.12, 1, 2, (255,255,255))
		self.current_frame = write_data(self.current_frame, 'Prc processes the saved images', 0.0, 0.85, 1.0, 0.15, 0.04, 0.12, 1, 2, (255,255,255))

		local_info_button = self.button_selected(self.info_point)
		if local_info_button == True:
			self.info_button = False



	def Process_image(self):

		input_size = [512, 512]
		transform = transforms.Compose([transforms.ToTensor(), transforms.Normalize(mean=[0.406, 0.456, 0.485], std=[0.225, 0.224, 0.229])])
		dataset = SimpleFolderDataset(root=images_folder_path, input_size=input_size, transform=transform)
		dataloader = DataLoader(dataset)

		palette = get_palette(7)

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
				parsing_result_path = os.path.join(output_path, img_name[:-4] + '.png')
				output_img = Image.fromarray(np.asarray(parsing_result, dtype=np.uint8))
				output_img.putpalette(palette)
				output_img.save(parsing_result_path)
			

		
		self.process_button = False