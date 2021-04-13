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

import cv2
import os
import socket




image_saved_path = '/streamlit_server_images'
if os.path.isdir(os.getcwd() + image_saved_path) == False:
	os.mkdir(os.getcwd() + image_saved_path)


process_image_path = '/streamlit_server_process_images'
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



def process_image():
	
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














