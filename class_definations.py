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

from functions import *






radius_fraction = 0.06
alpha = 0.4




images_folder_path = './images/'
output_path = './output/'


num_classes = 7
# num_classes = 20
# num_classes = 18
model = networks.init_model('resnet101', num_classes=num_classes, pretrained=None)
state_dict = torch.load('/home/mohit/Self-Correction-Human-Parsing-master/exp-schp-201908270938-pascal-person-part.pth')['state_dict']
# state_dict = torch.load('/home/mohit/Self-Correction-Human-Parsing-master/exp-schp-201908261155-lip.pth')['state_dict']
# state_dict = torch.load('/home/mohit/Self-Correction-Human-Parsing-master/exp-schp-201908301523-atr.pth')['state_dict']
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
        




previous_frame = None

class vision_demo_class:

	

	def __init__(self, image_timer_value, use_server_arg):


		global previous_frame
		self.use_server = use_server_arg
		self.cap = cv2.VideoCapture(0)
		# if not self.cap.isOpened():
		#     raise IOError("Cannot open webcam")


		# self.cap.set(cv2.CAP_PROP_FPS, 30)
		# self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
		# self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

		frame_received, frame = self.cap.read()
		if frame_received == False:
			frame = previous_frame
		else:
			previous_frame = frame
		

		self.image_height, self.image_width, layers = frame.shape

				
		self.current_img_timer = 0
		self.img_timer_flag = False
		self.image_timer_value = image_timer_value



		###########################################################
		################# Buttons for first screen ################
		###########################################################
		self.exit_point = np.array([np.int(0.15*self.image_width), np.int(0.15*self.image_height)], dtype=int)
		self.fit_point = np.array([np.int(0.5*self.image_width), np.int(0.15*self.image_height)], dtype=int)
		self.setting_point = np.array([np.int(0.85*self.image_width), np.int(0.15*self.image_height)], dtype=int)
		self.help_point = np.array([np.int(0.15*self.image_width), np.int(0.85*self.image_height)], dtype=int)

		self.exit_button = False
		self.fit_button = False
		self.setting_button = False
		self.help_button = False


		###########################################################
		####################### Return button #####################
		###########################################################
		self.return_point = np.array([np.int(0.85*self.image_width), np.int(0.85*self.image_height)], dtype=int)


		###########################################################
		################ Buttons for Setting screen ###############
		###########################################################
		self.save_point = np.array([np.int(0.15*self.image_width), np.int(0.15*self.image_height)], dtype=int)
		self.height_point = np.array([np.int(0.5*self.image_width), np.int(0.15*self.image_height)], dtype=int)
		self.process_point = np.array([np.int(0.85*self.image_width), np.int(0.15*self.image_height)], dtype=int)

		self.save_button = False
		self.height_button = False
		self.process_button = False


		###########################################################
		################# Buttons for Height screen ###############
		###########################################################
		self.feet_increment = np.array([np.int(0.43*self.image_width), np.int(0.25*self.image_height)], dtype=int)
		self.feet = np.array([np.int(0.43*self.image_width), np.int(0.45*self.image_height)], dtype=int)
		self.feet_decrement = np.array([np.int(0.43*self.image_width), np.int(0.65*self.image_height)], dtype=int)

		self.inch_increment = np.array([np.int(0.57*self.image_width), np.int(0.25*self.image_height)], dtype=int)
		self.inch = np.array([np.int(0.57*self.image_width), np.int(0.45*self.image_height)], dtype=int)
		self.inch_decrement = np.array([np.int(0.57*self.image_width), np.int(0.65*self.image_height)], dtype=int)


		self.height_feet = 0
		self.height_inch = 0

		
		if self.use_server == False:
			cv2.namedWindow('Visual_Try_ON')
			cv2.setMouseCallback('Visual_Try_ON', get_mouse_click)		

	
	
	def __del__(self):
		#releasing camera
		self.cap.release()

	
	def get_current_frame(self):

		global previous_frame

		frame_received, frame = self.cap.read()
		if frame_received == False:
			frame = previous_frame
		else:
			previous_frame = frame

		frame = cv2.flip(frame, 1) 

		self.timer = cv2.getTickCount()     # start the timer for the calculation of fps in function view_frame

		self.current_frame = frame.copy()  
		self.current_frame2 = frame.copy()  



	def view_frame(self):
	
		# self.fps = cv2.getTickFrequency() / (cv2.getTickCount() - self.timer)        # fps calculation with timer start in function get_current_frame.
		# self.current_frame = write_data(self.current_frame, 'fps:' + str(int(self.fps)), 0.05, 0.74, 0.17, 0.10, 0.01, 0.07, 1, 2, (255, 0, 255))
		cv2.imshow('Visual_Try_ON', self.current_frame)
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
					if os.path.isfile(images_folder_path + '/side.png'):
						os.remove(images_folder_path + '/side.png')
				else:
					cv2.imwrite(images_folder_path + '/side.png', self.current_frame2)
				self.img_timer_flag = False
				self.save_button = False



		local_save_button = self.button_selected(self.return_point)
		if local_save_button == True:
			self.save_button = False
			self.img_timer_flag = False



	def Process_image(self):

		if ( (os.path.isfile(images_folder_path + '/front.png')) & (os.path.isfile(images_folder_path + '/side.png')) ):

			input_size = [512, 512]
			transform = transforms.Compose([transforms.ToTensor(), transforms.Normalize(mean=[0.406, 0.456, 0.485], std=[0.225, 0.224, 0.229])])
			dataset = SimpleFolderDataset(root=images_folder_path, input_size=input_size, transform=transform)
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
					parsing_result_path = os.path.join(output_path, img_name[:-4] + '.png')
					output_img = Image.fromarray(np.asarray(parsing_result, dtype=np.uint8))
					output_img.putpalette(palette)
					output_img.save(parsing_result_path)
			
			self.process_button = False
			
			actual_height = (12*self.height_feet + self.height_inch)*2.54 # in cm, (1 inch = 2.54 cm)

			f_path = images_folder_path + '/front.png'
			human_part_seg_f_path = output_path + '/front.png'
			i1 = cv2.imread(f_path)
			m1 = cv2.imread(human_part_seg_f_path)

			s_path = images_folder_path + '/side.png'
			human_part_seg_s_path = output_path + '/side.png'
			i2 = cv2.imread(s_path)
			m2 = cv2.imread(human_part_seg_s_path)

			actual_height = (12*self.height_feet + self.height_inch) * 2.54 # conversion in cm
			get_measuremnets(i1, m1, i2, m2, actual_height, True)


		else:
			if self.img_timer_flag == False:
				self.current_img_timer = time.time()
				self.img_timer_flag = True

			count_down = self.image_timer_value - np.int( time.time() - self.current_img_timer)
		
			if ( self.img_timer_flag and count_down <= self.image_timer_value):
				self.current_frame = write_data(self.current_frame, 'Please save some images', 0.0, 0.4, 1.0, 0.15, 0.18, 0.10, 1, 2, (255,255,255))

			
			if count_down == 0 or self.button_selected(self.return_point):
				self.process_button = False
				self.img_timer_flag = False




	def first_screen(self):


		self.get_current_frame()


		if ( (self.exit_button == False) and (self.fit_button == False) and (self.setting_button == False) and (self.help_button == False) ):
			self.buttons(self.exit_point, (0, 0, 255), 'Ext', (255, 255, 255))
			self.exit_button = self.button_selected(self.exit_point)

			self.buttons(self.fit_point, (255, 0, 0), 'Fit', (255, 255, 255))
			self.fit_button = self.button_selected(self.fit_point)

			self.buttons(self.setting_point, (255, 0, 0), 'Set', (255, 255, 255))
			self.setting_button = self.button_selected(self.setting_point)

			self.buttons(self.help_point, (255, 0, 0), ' ?', (255, 255, 255))
			self.help_button = self.button_selected(self.help_point)


		else:
			if self.exit_button:
				print()
				print (colored("********************************", 'cyan'))
				print (colored("You have touched the exit button", 'red'))
				print (colored("********************************", 'cyan'))
				self.cap.release()
				cv2.destroyAllWindows()
				exit()


			if self.help_button:
				self.buttons(self.return_point, (255, 0, 0), '<', (255, 255, 255))
				self.help_screen()


			if self.fit_button:
				self.buttons(self.return_point, (255, 0, 0), '<', (255, 255, 255))
				self.fitting_screen()


			if self.setting_button:
				self.buttons(self.return_point, (255, 0, 0), '<', (255, 255, 255))
				self.setting_screen()


		if self.use_server == False:
			self.view_frame()	





	def fitting_screen(self):
		self.current_frame = write_data(self.current_frame, 'In development phase', 0.0, 0.4, 1.0, 0.15, 0.2, 0.1, 1, 2, (255,255,255))
		local_fit_button = self.button_selected(self.return_point)
		if local_fit_button == True:
			self.fit_button = False



	def help_screen(self):
		self.current_frame = write_data(self.current_frame, 'Store front and side face images', 0.0, 0.0, 1.0, 0.15, 0.04, 0.12, 1, 2, (255,255,255))
		self.current_frame = write_data(self.current_frame, 'For Front, stand in T pose', 0.0, 0.15, 1.0, 0.15, 0.04, 0.12, 1, 2, (255,255,255))
		self.current_frame = write_data(self.current_frame, 'For side, rotate about 90 in T pose', 0.0, 0.30, 1.0, 0.15, 0.04, 0.12, 1, 2, (255,255,255))
		self.current_frame = write_data(self.current_frame, 'Sav button saves single images', 0.0, 0.45, 1.0, 0.15, 0.04, 0.12, 1, 2, (255,255,255))
		self.current_frame = write_data(self.current_frame, 'Prc processes the saved images', 0.0, 0.6, 1.0, 0.15, 0.04, 0.12, 1, 2, (255,255,255))

		local_help_button = self.button_selected(self.return_point)
		if local_help_button == True:
			self.help_button = False



	def setting_screen(self):

		if ( (self.save_button == False) and (self.height_button == False) and (self.process_button == False) ):
			self.buttons(self.save_point, (255, 0, 0), 'Sav', (255, 255, 255))
			self.save_button = self.button_selected(self.save_point)

			self.buttons(self.height_point, (255, 0, 0), 'Hit', (255, 255, 255))
			self.height_button = self.button_selected(self.height_point)

			self.buttons(self.process_point, (255, 0, 0), 'Prc', (255, 255, 255))
			self.process_button = self.button_selected(self.process_point)


		else:
			
			if self.save_button:
				self.save_img()


			if self.height_button:
				self.buttons(self.return_point, (255, 0, 0), '<', (255, 255, 255))
				self.height_screen()


			if self.process_button:
				self.Process_image()



		local_setting_button = self.button_selected(self.return_point)
		if local_setting_button == True:
			self.setting_button = False



	def height_screen(self):
		# self.current_frame = write_data(self.current_frame, 'In development phase', 0.0, 0.4, 1.0, 0.15, 0.2, 0.1, 1, 2, (255,255,255))
		local_height_button = self.button_selected(self.return_point)
		if local_height_button == True:
			self.height_button = False



		self.buttons(self.feet_increment, (255, 0, 0), ' +', (255, 255, 255))
		self.buttons(self.feet, (255, 0, 0), ' ' + str(self.height_feet) + '\'', (255, 255, 255))		
		self.buttons(self.feet_decrement, (255, 0, 0), ' -', (255, 255, 255))

		self.buttons(self.inch_increment, (255, 0, 0), ' +', (255, 255, 255))
		self.buttons(self.inch, (255, 0, 0), str(self.height_inch).zfill(2) + '\"', (255, 255, 255))		
		self.buttons(self.inch_decrement, (255, 0, 0), ' -', (255, 255, 255))


		local_feet_increment = self.button_selected(self.feet_increment)
		if local_feet_increment:
			self.height_feet += 1


		local_feet_decrement = self.button_selected(self.feet_decrement)
		if local_feet_decrement:
			self.height_feet -= 1
			if self.height_feet < 0:
				self.height_feet = 0


		local_inch_increment = self.button_selected(self.inch_increment)
		if local_inch_increment:
			self.height_inch += 1
			self.height_inch %= 12


		local_inch_decrement = self.button_selected(self.inch_decrement)
		if local_inch_decrement:
			self.height_inch -= 1
			if self.height_inch < 0:
				self.height_inch = 0

		

	def get_frame(self):
		ret, jpeg = cv2.imencode('.jpg', self.current_frame)
		return jpeg.tobytes()