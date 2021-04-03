import cv2
import numpy as np

from skimage.morphology import skeletonize
from sklearn import linear_model



### BGR
head_part = [0,0,128]
torse_part = [0,128,0]
upper_arm_part = [0,128,128]
lower_arm_part = [128,0,0]
upper_legs_part = [128,0,128]
lower_legs_part = [128,128, 0]




def get_waist_pixels_mask(mask):
	mask_array = np.array(mask, np.uint8)

	torse_mask_array = 0*mask_array
	upper_legs_array = 0*mask_array
	waist_array = 0*mask_array

	x = (mask_array[:,:,0] == torse_part[0]) & (mask_array[:,:,1] == torse_part[1]) & (mask_array[:,:,2] == torse_part[2])
	torse_mask_array[x, :] = [255, 255, 255]

	kernel = np.ones((3,3),np.uint8)
	torse_mask_array_dilated = cv2.dilate(torse_mask_array,kernel, iterations = 1)

	x = (mask_array[:,:,0] == upper_legs_part[0]) & (mask_array[:,:,1] == upper_legs_part[1]) & (mask_array[:,:,2] == upper_legs_part[2])
	upper_legs_array[x, :] = [255, 255, 255]

	waist_pixels = (torse_mask_array_dilated[:,:,0] == 255) & (torse_mask_array_dilated[:,:,1] == 255) & (torse_mask_array_dilated[:,:,2] == 255) & (upper_legs_array[:,:,0] == 255) & (upper_legs_array[:,:,1] == 255) & (upper_legs_array[:,:,2] == 255)
	waist_array[waist_pixels, :] = [255, 255, 255]

	return waist_array



def get_person_bounding_box(mask, visualize_bbox=False):
	mask_array = np.array(mask, np.uint8)

	complete_body_array = 255*np.ones(mask_array.shape, np.uint8)
	x = (mask_array[:,:,0] == 0) & (mask_array[:,:,1] == 0) & (mask_array[:,:,2] == 0)
	complete_body_array[x, :] = [0,0,0]

	complete_body_array = cv2.cvtColor(complete_body_array,cv2.COLOR_RGB2GRAY)

	(major, minor, _) = cv2.__version__.split(".")
	if (np.int(major) >= 4):
		contours, _ = cv2.findContours(complete_body_array, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	else:
		_, contours, _ = cv2.findContours(complete_body_array, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

	num_pts = 0
	index = -1
	for j in range(len(contours)):
		pts = cv2.contourArea(contours[j])
		if pts > num_pts:
			index = j
			num_pts = pts

	if index >= 0:
		rec_x, rec_y, rec_w, rec_h = cv2.boundingRect(contours[index])
	else:
		rec_x, rec_y, rec_w, rec_h = [0,0,0,0]

	cv2.rectangle(complete_body_array, (rec_x, rec_y), (rec_x + rec_w, rec_y + rec_h), 255, 2)

	if visualize_bbox:
		cv2.imshow('complete_body_array', complete_body_array)
		cv2.waitKey(1)

	return rec_x, rec_y, rec_w, rec_h



def get_part_bounding_box(mask, visualize_bbox=False):
	mask = np.array(mask, np.uint8)
	mask = cv2.cvtColor(mask,cv2.COLOR_RGB2GRAY)

	(major, minor, _) = cv2.__version__.split(".")
	if (np.int(major) >= 4):
		contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	else:
		_, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

	rec_x, rec_y, rec_w, rec_h = [0,0,0,0]
	final_contour = []

	for j in range(len(contours)):
		for k in range(len(contours[j])):
			final_contour.append(contours[j][k])


	if len(contours) > 0:
		rec_x, rec_y, rec_w, rec_h = cv2.boundingRect(np.array(final_contour))
		


	cv2.rectangle(mask, (rec_x, rec_y), (rec_x + rec_w, rec_y + rec_h), 255, 2)

	if visualize_bbox:
		cv2.imshow('waist_array', waist_array)
		cv2.waitKey(1)

	return rec_x, rec_y, rec_w, rec_h



def get_person_front_measurements(image, mask, visualize=False):
	waist_mask = get_waist_pixels_mask(mask)
	waist_bbox = get_part_bounding_box(waist_mask)
	person_bbox = get_person_bounding_box(mask)

	if visualize:
		p1 = (waist_bbox[0], np.int(waist_bbox[1] + waist_bbox[3]/2))
		p2 = (waist_bbox[0] + waist_bbox[2], np.int(waist_bbox[1] + waist_bbox[3]/2))
		cv2.arrowedLine(image, p1, p2, (0,0,255), 2, tipLength = 0.1)
		cv2.arrowedLine(image, p2, p1, (0,0,255), 2, tipLength = 0.1)

		p1 = (np.int(person_bbox[0] + 1.1*person_bbox[2]), person_bbox[1])
		p2 = (np.int(person_bbox[0] + 1.1*person_bbox[2]), np.int(person_bbox[1] + person_bbox[3]))
		cv2.arrowedLine(image, p1, p2, (255,0,255), 2, tipLength = 0.03)
		cv2.arrowedLine(image, p2, p1, (255,0,255), 2, tipLength = 0.03)

		cv2.imshow('front_image', image)
		cv2.waitKey(1)



	return waist_bbox[2], person_bbox[3]



def get_person_side_measurements(image, mask, visualize=False):
	waist_mask = get_waist_pixels_mask(mask)
	waist_bbox = get_part_bounding_box(waist_mask)
	person_bbox = get_person_bounding_box(mask)

	if visualize:
		# p1 = (waist_bbox[0], waist_bbox[1])
		# p2 = (waist_bbox[0] + waist_bbox[2], waist_bbox[1] + waist_bbox[3])
		p1 = (waist_bbox[0], np.int(waist_bbox[1] + waist_bbox[3]/2))
		p2 = (waist_bbox[0] + waist_bbox[2], np.int(waist_bbox[1] + waist_bbox[3]/2))
		cv2.arrowedLine(image, p1, p2, (0,0,255), 2, tipLength = 0.1)
		cv2.arrowedLine(image, p2, p1, (0,0,255), 2, tipLength = 0.1)


		p1 = (np.int(person_bbox[0] + 1.1*person_bbox[2]), person_bbox[1])
		p2 = (np.int(person_bbox[0] + 1.1*person_bbox[2]), np.int(person_bbox[1] + person_bbox[3]))
		cv2.arrowedLine(image, p1, p2, (255,0,255), 2, tipLength = 0.03)
		cv2.arrowedLine(image, p2, p1, (255,0,255), 2, tipLength = 0.03)

		cv2.imshow('side_image', image)
		cv2.waitKey(1)



	# return np.int(np.sqrt(waist_bbox[2]*waist_bbox[2] + waist_bbox[3]*waist_bbox[3])), person_bbox[3]
	return waist_bbox[2], person_bbox[3]



def get_person_side_measurements_with_waitKey_0(image, mask, visualize=False):
	waist_mask = get_waist_pixels_mask(mask)
	waist_bbox = get_part_bounding_box(waist_mask)
	person_bbox = get_person_bounding_box(mask)

	if visualize:
		# p1 = (waist_bbox[0], waist_bbox[1])
		# p2 = (waist_bbox[0] + waist_bbox[2], waist_bbox[1] + waist_bbox[3])
		p1 = (waist_bbox[0], np.int(waist_bbox[1] + waist_bbox[3]/2))
		p2 = (waist_bbox[0] + waist_bbox[2], np.int(waist_bbox[1] + waist_bbox[3]/2))
		cv2.arrowedLine(image, p1, p2, (0,0,255), 2, tipLength = 0.1)
		cv2.arrowedLine(image, p2, p1, (0,0,255), 2, tipLength = 0.1)


		p1 = (np.int(person_bbox[0] + 1.1*person_bbox[2]), person_bbox[1])
		p2 = (np.int(person_bbox[0] + 1.1*person_bbox[2]), np.int(person_bbox[1] + person_bbox[3]))
		cv2.arrowedLine(image, p1, p2, (255,0,255), 2, tipLength = 0.03)
		cv2.arrowedLine(image, p2, p1, (255,0,255), 2, tipLength = 0.03)

		cv2.imshow('side_image', image)
		cv2.waitKey(0)



	# return np.int(np.sqrt(waist_bbox[2]*waist_bbox[2] + waist_bbox[3]*waist_bbox[3])), person_bbox[3]
	return waist_bbox[2], person_bbox[3]



def get_chest_pixels_mask(mask):
	mask_array = np.array(mask, np.uint8)

	torse_mask_array = 0*mask_array
	upper_arm_part_array = 0*mask_array
	chest_array = 0*mask_array

	x = (mask_array[:,:,0] == torse_part[0]) & (mask_array[:,:,1] == torse_part[1]) & (mask_array[:,:,2] == torse_part[2])
	torse_mask_array[x, :] = [255, 255, 255]

	kernel = np.ones((3,3),np.uint8)
	torse_mask_array_dilated = cv2.dilate(torse_mask_array,kernel, iterations = 1)

	x = (mask_array[:,:,0] == upper_arm_part[0]) & (mask_array[:,:,1] == upper_arm_part[1]) & (mask_array[:,:,2] == upper_arm_part[2])
	upper_arm_part_array[x, :] = [255, 255, 255]

	chest_pixels = (torse_mask_array_dilated[:,:,0] == 255) & (torse_mask_array_dilated[:,:,1] == 255) & (torse_mask_array_dilated[:,:,2] == 255) & (upper_arm_part_array[:,:,0] == 255) & (upper_arm_part_array[:,:,1] == 255) & (upper_arm_part_array[:,:,2] == 255)
	chest_array[chest_pixels, :] = [255, 255, 255]

	return chest_array



def get_chest_pixels(mask):
	chest_mask = get_chest_pixels_mask(mask)
	rec_x, rec_y, rec_w, rec_h = get_part_bounding_box(chest_mask)

	chest_array = 0*chest_mask
	chest_array[rec_y+rec_h,:,:] = [255,255,255]

	mask_array = np.array(mask, np.uint8)
	torse_mask_array = 0*mask_array
	x = (mask_array[:,:,0] == torse_part[0]) & (mask_array[:,:,1] == torse_part[1]) & (mask_array[:,:,2] == torse_part[2])
	torse_mask_array[x, :] = [255, 255, 255]

	chest_pixels = (chest_array[:,:,0] == 255) & (chest_array[:,:,1] == 255) & (chest_array[:,:,2] == 255) & (torse_mask_array[:,:,0] == 255) & (torse_mask_array[:,:,1] == 255) & (torse_mask_array[:,:,2] == 255)
	chest_array = 0*chest_mask
	chest_array[chest_pixels,:] = [255,255,255]


	return chest_array



def get_front_chest_and_waist(image, mask, visualize=False):
	waist_mask = get_waist_pixels_mask(mask)
	waist_bbox = get_part_bounding_box(waist_mask)

	chest_mask = get_chest_pixels(mask)
	chest_bbox = get_part_bounding_box(chest_mask)

	person_bbox = get_person_bounding_box(mask)

	p1 = (waist_bbox[0], np.int(waist_bbox[1] + waist_bbox[3]/2))
	p2 = (waist_bbox[0] + waist_bbox[2], np.int(waist_bbox[1] + waist_bbox[3]/2))
	cv2.arrowedLine(image, p1, p2, (0,0,255), 2, tipLength = 0.1)
	cv2.arrowedLine(image, p2, p1, (0,0,255), 2, tipLength = 0.1)


	p1 = (chest_bbox[0], np.int(chest_bbox[1] + chest_bbox[3]/2))
	p2 = (chest_bbox[0] + chest_bbox[2], np.int(chest_bbox[1] + chest_bbox[3]/2))
	cv2.arrowedLine(image, p1, p2, (0,0,255), 2, tipLength = 0.1)
	cv2.arrowedLine(image, p2, p1, (0,0,255), 2, tipLength = 0.1)


	p1 = (np.int(person_bbox[0] + 1.1*person_bbox[2]), person_bbox[1])
	p2 = (np.int(person_bbox[0] + 1.1*person_bbox[2]), np.int(person_bbox[1] + person_bbox[3]))
	cv2.arrowedLine(image, p1, p2, (255,0,255), 2, tipLength = 0.03)
	cv2.arrowedLine(image, p2, p1, (255,0,255), 2, tipLength = 0.03)


	if visualize==True:
		cv2.imshow('front', image)
		cv2.waitKey(1)

	return image, waist_bbox[2], chest_bbox[2], person_bbox[3]



def get_side_chest_and_waist(image, mask, visualize=False):
	waist_mask = get_waist_pixels_mask(mask)
	waist_bbox = get_part_bounding_box(waist_mask)

	chest_mask = get_chest_pixels(mask)
	chest_bbox = get_part_bounding_box(chest_mask)

	person_bbox = get_person_bounding_box(mask)

	p1 = (waist_bbox[0], np.int(waist_bbox[1] + waist_bbox[3]/2))
	p2 = (waist_bbox[0] + waist_bbox[2], np.int(waist_bbox[1] + waist_bbox[3]/2))
	cv2.arrowedLine(image, p1, p2, (0,0,255), 2, tipLength = 0.1)
	cv2.arrowedLine(image, p2, p1, (0,0,255), 2, tipLength = 0.1)


	p1 = (chest_bbox[0], np.int(chest_bbox[1] + chest_bbox[3]/2))
	p2 = (chest_bbox[0] + chest_bbox[2], np.int(chest_bbox[1] + chest_bbox[3]/2))
	cv2.arrowedLine(image, p1, p2, (0,0,255), 2, tipLength = 0.1)
	cv2.arrowedLine(image, p2, p1, (0,0,255), 2, tipLength = 0.1)


	p1 = (np.int(person_bbox[0] + 1.1*person_bbox[2]), person_bbox[1])
	p2 = (np.int(person_bbox[0] + 1.1*person_bbox[2]), np.int(person_bbox[1] + person_bbox[3]))
	cv2.arrowedLine(image, p1, p2, (255,0,255), 2, tipLength = 0.03)
	cv2.arrowedLine(image, p2, p1, (255,0,255), 2, tipLength = 0.03)


	if visualize==True:
		
		cv2.imshow('side', image)
		cv2.waitKey(1)

	return image, waist_bbox[2], chest_bbox[2], person_bbox[3]



def get_human_head_torse_fraction_of_actual_height(image, mask):
	person_bbox = get_person_bounding_box(mask)


	mask_array = np.array(mask, np.uint8)
	head_torse_array = 0*mask_array


	x = (mask_array[:,:,0] == torse_part[0]) & (mask_array[:,:,1] == torse_part[1]) & (mask_array[:,:,2] == torse_part[2])
	head_torse_array[x, :] = [255, 255, 255]

	x = (mask_array[:,:,0] == head_part[0]) & (mask_array[:,:,1] == head_part[1]) & (mask_array[:,:,2] == head_part[2])
	head_torse_array[x, :] = [255, 255, 255]


	part_bbox = get_part_bounding_box(head_torse_array)


	# p1 = (np.int(part_bbox[0] + part_bbox[2]), part_bbox[1])
	# p2 = (np.int(part_bbox[0] + part_bbox[2]), np.int(part_bbox[1] + part_bbox[3]))
	# cv2.arrowedLine(image, p1, p2, (255,0,255), 2, tipLength = 0.03)
	# cv2.arrowedLine(image, p2, p1, (255,0,255), 2, tipLength = 0.03)
	# cv2.imshow('image', image)

	return person_bbox[3], part_bbox[3]

	

def fit_rotated_rectangle_on_thighs(image, mask, side_image, side_mask, visualize=False):

	mask_array = np.array(mask, np.uint8)
	upper_legs_array = 0*mask_array

	x = (mask_array[:,:,0] == upper_legs_part[0]) & (mask_array[:,:,1] == upper_legs_part[1]) & (mask_array[:,:,2] == upper_legs_part[2])
	y = (mask_array[:,:,0] == lower_legs_part[0]) & (mask_array[:,:,1] == lower_legs_part[1]) & (mask_array[:,:,2] == lower_legs_part[2])
	upper_legs_array[x, :] = [255, 255, 255]
	upper_legs_array[y, :] = [255, 255, 255]

	upper_legs_only_array = 0*mask_array
	upper_legs_only_array[x, :] = [255, 255, 255]

	part_bbox = get_person_bounding_box(upper_legs_array)

	new_part_bbox = list(part_bbox)
	centroid_point = []
	while(1):
		new_part_bbox[1] += 1
		new_part_bbox[3] -= 1

		new_mask = 0*mask
		cv2.rectangle(new_mask, (new_part_bbox[0], new_part_bbox[1]), (new_part_bbox[0] + new_part_bbox[2], new_part_bbox[1] + new_part_bbox[3]), (255,255,255), -1)
		x = (upper_legs_array[:,:,0] == 255) & (new_mask[:,:,0] == 255)
		new_mask = 0*mask
		new_mask[x, :] = [255, 255, 255]


		(major, minor, _) = cv2.__version__.split(".")
		if (np.int(major) >= 4):
			contours, _ = cv2.findContours(new_mask[:,:,0], cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		else:
			_, contours, _ = cv2.findContours(new_mask[:,:,0], cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

		if (len(contours) > 1):
			new_part_bbox[1] += 2
			break

		


	cropped_upper_legs_array = 0*mask_array
	cv2.rectangle(cropped_upper_legs_array, (new_part_bbox[0], new_part_bbox[1]), (new_part_bbox[0] + new_part_bbox[2], new_part_bbox[1] + np.int(1.0*new_part_bbox[3])), (255,255,255), -1)
	x = (upper_legs_array[:,:,0] == 255) & (cropped_upper_legs_array[:,:,0] == 255)
	cropped_upper_legs_array = 0*mask_array
	cropped_upper_legs_array[x, :] = [255, 255, 255]

	# ### To visualize the cropped legs
	# cv2.imshow('cropped_upper_legs_array', cropped_upper_legs_array)
	# cv2.waitKey(0)


	cropped = 0*mask_array
	cv2.rectangle(cropped, (part_bbox[0], part_bbox[1]), (part_bbox[0] + part_bbox[2], part_bbox[1] + np.int(1.0*part_bbox[3] - new_part_bbox[3])), (255,255,255), -1)
	x = (upper_legs_array[:,:,0] == 255) & (cropped[:,:,0] == 255)
	cropped = 0*mask_array
	cropped[x, :] = [255, 255, 255]
	pixel_loc = np.where(cropped[:,:,0] > 0)
	centroid_point = ( np.int(np.mean(pixel_loc[0])), np.int(np.mean(pixel_loc[1])) )


	(major, minor, _) = cv2.__version__.split(".")
	if (np.int(major) >= 4):
		contours, _ = cv2.findContours(cropped_upper_legs_array[:,:,0], cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	else:
		_, contours, _ = cv2.findContours(cropped_upper_legs_array[:,:,0], cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

	num_pts = 0
	index = -1
	for j in range(len(contours)):
		pts = cv2.contourArea(contours[j])
		if pts > num_pts:
			index = j
			num_pts = pts

	final_pixel_loc = [] 
	start_pixel_loc = []
	mid_pixel_loc = []
	forth_pixel_loc = []

	if index > -1:
		# minAreaRect = cv2.minAreaRect(contours[index])
		# box = cv2.boxPoints(minAreaRect)
		# box = np.int0(box)
	
		# x_diff = box[2][1] - box[3][1]
		# y_diff = box[2][0] - box[3][0]

		# dis = np.sqrt(x_diff*x_diff + y_diff*y_diff)
		# x_diff /= dis
		# y_diff /= dis

		# new_pixel_x = box[3][1] 
		# new_pixel_y = box[3][0] 
		# while (1):
		# 	new_pixel_x += x_diff
		# 	new_pixel_y += y_diff

		# 	if ((upper_legs_array[np.int(new_pixel_x), np.int(new_pixel_y),0] > 0) or (upper_legs_array[np.int(new_pixel_x), np.int(new_pixel_y),1] > 0) or (upper_legs_array[np.int(new_pixel_x), np.int(new_pixel_y),2] > 0)):
		# 		continue
		# 	else:
		# 		break

		# final_pixel_loc = (np.int(new_pixel_x), np.int(new_pixel_y))

		# new_pixel_x = box[3][1] 
		# new_pixel_y = box[3][0] 
		# while (1):
		# 	new_pixel_x -= x_diff
		# 	new_pixel_y -= y_diff

		# 	if ((upper_legs_array[np.int(new_pixel_x), np.int(new_pixel_y),0] > 0) or (upper_legs_array[np.int(new_pixel_x), np.int(new_pixel_y),1] > 0) or (upper_legs_array[np.int(new_pixel_x), np.int(new_pixel_y),2] > 0)):
		# 		continue
		# 	else:
		# 		break

		# start_pixel_loc = (np.int(new_pixel_x), np.int(new_pixel_y))


		


		part_bbox2 = cv2.boundingRect(contours[index])
		new_part_bbox = list(part_bbox2)
		pixel_loc = np.where(cropped_upper_legs_array > 0)
		
		first_point = (new_part_bbox[0], new_part_bbox[1])
		second_point = (new_part_bbox[0] + new_part_bbox[2], new_part_bbox[1])

		f_dis_min = 10000
		s_dis_min = 10000
		f_dis_index = -1
		s_dis_index = -1

		for i in range(pixel_loc[0].shape[0]):
			x = pixel_loc[0][i]
			y = pixel_loc[1][i]
			y_ = new_part_bbox[1]

			f_dis = np.sqrt((x - first_point[1])*(x - first_point[1]) + (y - first_point[0])*(y - first_point[0]))
			s_dis = np.sqrt((x - second_point[1])*(x - second_point[1]) + (y - second_point[0])*(y - second_point[0]))

			f_y_dis = np.sqrt((y_ - x)*(y_ - x))

			if ((f_dis < f_dis_min) and (f_y_dis < 2)):
			# if (f_dis < f_dis_min):
				f_dis_min = f_dis
				f_dis_index = i

			if ((s_dis < s_dis_min) and (f_y_dis < 2)):
			# if s_dis < s_dis_min:
				s_dis_min = s_dis
				s_dis_index = i


		final_pixel_loc = (pixel_loc[0][f_dis_index], pixel_loc[1][f_dis_index])
		start_pixel_loc = (pixel_loc[0][s_dis_index], pixel_loc[1][s_dis_index])


			

		new_part_bbox = list(part_bbox)
		new_part_bbox[3] = 1
		while(1):

			temp_array = 0*mask_array
			cv2.rectangle(temp_array, (new_part_bbox[0], new_part_bbox[1]), (new_part_bbox[0] + new_part_bbox[2], new_part_bbox[1] + np.int(1.0*new_part_bbox[3])), (255,255,255), -1)
			x = (upper_legs_array[:,:,0] == 255) & (temp_array[:,:,0] == 255)
			temp_array = 0*mask_array
			temp_array[x, :] = [255, 255, 255]
			pixel_loc = np.where(temp_array > 0)
			if pixel_loc[1].shape[0] == 0:
				continue
			
			min_x = np.min(pixel_loc[1])
			max_x = np.max(pixel_loc[1])

			
			start_point_x_max_diff = start_pixel_loc[1] - max_x
			if start_point_x_max_diff < 0:
				start_point_x_max_diff = -start_point_x_max_diff

			start_point_x_min_diff = start_pixel_loc[1] - min_x
			if start_point_x_min_diff < 0:
				start_point_x_min_diff = -start_point_x_min_diff				
			
			final_point_x_max_diff = final_pixel_loc[1] - max_x
			if final_point_x_max_diff < 0:
				final_point_x_max_diff = -final_point_x_max_diff
			
			final_point_x_min_diff = final_pixel_loc[1] - min_x
			if final_point_x_min_diff < 0:
				final_point_x_min_diff = -final_point_x_min_diff


			if ( (final_point_x_min_diff < start_point_x_min_diff) or (final_point_x_max_diff < start_point_x_max_diff) ):
				if (final_point_x_min_diff < final_point_x_max_diff):
					mid_pixel_loc = (new_part_bbox[1], min_x)
				else:
					mid_pixel_loc = (new_part_bbox[1], max_x)

				break

			new_part_bbox[1] += 1


		

		part_bbox = get_person_bounding_box(upper_legs_only_array)
		new_part_bbox = list(part_bbox)
		pixel_loc = np.where(upper_legs_only_array > 0)
		
		first_point = (new_part_bbox[0], new_part_bbox[1] + new_part_bbox[3])
		second_point = (new_part_bbox[0] + new_part_bbox[2], new_part_bbox[1] + new_part_bbox[3])

		f_dis_min = 10000
		s_dis_min = 10000
		f_dis_index = -1
		s_dis_index = -1

		for i in range(pixel_loc[0].shape[0]):
			x = pixel_loc[0][i]
			y = pixel_loc[1][i]

			f_dis = np.sqrt((x - first_point[1])*(x - first_point[1]) + (y - first_point[0])*(y - first_point[0]))
			s_dis = np.sqrt((x - second_point[1])*(x - second_point[1]) + (y - second_point[0])*(y - second_point[0]))

			if f_dis < f_dis_min:
				f_dis_min = f_dis
				f_dis_index = i

			if s_dis < s_dis_min:
				s_dis_min = s_dis
				s_dis_index = i


		forth_pixel_loc1 = (pixel_loc[0][f_dis_index], pixel_loc[1][f_dis_index])
		forth_pixel_loc2 = (pixel_loc[0][s_dis_index], pixel_loc[1][s_dis_index])


		dis1 = np.sqrt((forth_pixel_loc1[0] - final_pixel_loc[0])*(forth_pixel_loc1[0] - final_pixel_loc[0]) + (forth_pixel_loc1[1] - final_pixel_loc[1])*(forth_pixel_loc1[1] - final_pixel_loc[1]))
		dis2 = np.sqrt((forth_pixel_loc2[0] - final_pixel_loc[0])*(forth_pixel_loc2[0] - final_pixel_loc[0]) + (forth_pixel_loc2[1] - final_pixel_loc[1])*(forth_pixel_loc2[1] - final_pixel_loc[1]))

		if dis1 < dis2:
			forth_pixel_loc = forth_pixel_loc1
		else:
			forth_pixel_loc = forth_pixel_loc2



		centroid_start_dis = np.sqrt( (centroid_point[0] - start_pixel_loc[0])*(centroid_point[0] - start_pixel_loc[0]) + (centroid_point[1] - start_pixel_loc[1])*(centroid_point[1] - start_pixel_loc[1]) )
		centroid_final_dis = np.sqrt( (centroid_point[0] - final_pixel_loc[0])*(centroid_point[0] - final_pixel_loc[0]) + (centroid_point[1] - final_pixel_loc[1])*(centroid_point[1] - final_pixel_loc[1]) )

		inner_thigh_point = start_pixel_loc
		outer_thigh_point = final_pixel_loc

		if centroid_final_dis < centroid_start_dis:
			inner_thigh_point = final_pixel_loc
			outer_thigh_point = start_pixel_loc


		
		# ### To visualize centroid point
		# p1 = (centroid_point[1], centroid_point[0])
		# cv2.circle(image, p1, 10, (255,0,0), -1)

		# ### To visualize forth_pixel_loc
		# p1 = (forth_pixel_loc[1], forth_pixel_loc[0])
		# cv2.circle(image, p1, 10, (255,0,0), -1)

		p1 = (inner_thigh_point[1], inner_thigh_point[0])
		p2 = (outer_thigh_point[1], outer_thigh_point[0])

		# ### To visualize inner_thigh_point
		# cv2.circle(image, p1, 10, (0,255,0), -1)

		# ### To visualize outer_thigh_point
		# cv2.circle(image, p2, 10, (255,0,0), -1)

		cv2.arrowedLine(image, p1, p2, (0,0,255), 2, tipLength = 0.1)
		cv2.arrowedLine(image, p2, p1, (0,0,255), 2, tipLength = 0.1)


		if visualize:
			cv2.imshow('image', image)



		total_dis = np.sqrt((mid_pixel_loc[0] - forth_pixel_loc[0])*(mid_pixel_loc[0] - forth_pixel_loc[0]) + (mid_pixel_loc[1] - forth_pixel_loc[1])*(mid_pixel_loc[1] - forth_pixel_loc[1]))
		mid_dis = np.sqrt((mid_pixel_loc[0] - final_pixel_loc[0])*(mid_pixel_loc[0] - final_pixel_loc[0]) + (mid_pixel_loc[1] - final_pixel_loc[1])*(mid_pixel_loc[1] - final_pixel_loc[1]))
		fraction = mid_dis / total_dis

		mask_array = np.array(side_mask, np.uint8)
		upper_legs_array = 0*mask_array
		x = (mask_array[:,:,0] == upper_legs_part[0]) & (mask_array[:,:,1] == upper_legs_part[1]) & (mask_array[:,:,2] == upper_legs_part[2])
		upper_legs_array[x, :] = [255, 255, 255]

		part_bbox = get_person_bounding_box(upper_legs_array)
		y_cordinate = part_bbox[1] + np.int(part_bbox[3]*fraction)
		new_part_bbox = list(part_bbox)
		new_part_bbox[1] = y_cordinate
		new_part_bbox[3] = 1

		rec_array = 0*mask_array
		cv2.rectangle(rec_array, (new_part_bbox[0], new_part_bbox[1]), (new_part_bbox[0] + new_part_bbox[2], new_part_bbox[1] + new_part_bbox[3]), (255,255,255), -1)
		x = (rec_array[:,:,0] == 255) & (upper_legs_array[:,:,0] == 255)
		rec_array = 0*mask_array
		rec_array[x, :] = [255, 255, 255]
		pixel_loc = np.where(rec_array > 0)

		min_x = np.min(pixel_loc[1])
		max_x = np.max(pixel_loc[1])

		
		side_f_point = (y_cordinate, min_x)
		side_s_point = (y_cordinate, max_x)

		p1 = (side_f_point[1], side_f_point[0])
		p2 = (side_s_point[1], side_s_point[0])
		
		cv2.arrowedLine(side_image, p1, p2, (0,0,255), 2, tipLength = 0.1)
		cv2.arrowedLine(side_image, p2, p1, (0,0,255), 2, tipLength = 0.1)

		if visualize:
			cv2.imshow('side_image', side_image)


		return inner_thigh_point, outer_thigh_point, mid_pixel_loc, forth_pixel_loc, side_f_point, side_s_point, image, side_image, True


	return 0,0,0,0,0,0,False



def get_measuremnets(i1, m1, i2, m2, actual_height, visualize=False):
	m1 = remove_unwanted_person_seg(m1)
	m2 = remove_unwanted_person_seg(m2)

	p1, p2, p3, p4 = get_ankle_pixels(i1, m1)

	
	# get side view full height and torse+head ratio
	side_full_height_pixels, side_head_torse_height_pixels = get_human_head_torse_fraction_of_actual_height(i2, m2)

	# calculate front view expected full heigt pixels using only head+torse pixels
	real_front_height_pixels, front_head_torse_height_pixels = get_human_head_torse_fraction_of_actual_height(i1, m1)
	front_full_height_pixels = front_head_torse_height_pixels *(side_full_height_pixels / side_head_torse_height_pixels)

	# print ('real_front_height_pixels ', real_front_height_pixels)
	# print ('calculated_front_height_pixels ', front_full_height_pixels)

	i1, front_waist_dim, front_chest_dim, _ = get_front_chest_and_waist(i1, m1)
	i2, side_waist_dim, side_chest_dim, _ = get_side_chest_and_waist(i2, m2)
	i1, sleeve_start_pixel_loc, sleeve_final_pixel_loc = get_sleeve_points(i1, m1)
	start_pixel_loc, final_pixel_loc, mid_pixel_loc, forth_pixel_loc, side_f_point, side_s_point, image, side_image, status = fit_rotated_rectangle_on_thighs(i1, m1, i2, m2)

	if status == True:
		side_cm_per_pixel = actual_height/side_full_height_pixels
		front_cm_per_pixel = actual_height/front_full_height_pixels

		side_waist_in_cm = side_waist_dim*side_cm_per_pixel
		front_waist_in_cm = front_waist_dim*front_cm_per_pixel
		r1 = front_waist_in_cm/2
		r2 = side_waist_in_cm/2
		waist = 2*(22/7)*np.sqrt(0.5*r1*r1 + 0.5*r2*r2)
		print ('waist_in_cm', waist, ' and in inches ', waist/2.54)

		side_chest_in_cm = side_chest_dim*side_cm_per_pixel
		front_chest_in_cm = front_chest_dim*front_cm_per_pixel
		r1 = front_chest_in_cm/2
		r2 = side_chest_in_cm/2
		chest = 2*(22/7)*np.sqrt(0.5*r1*r1 + 0.5*r2*r2)
		print ('chest_in_cm', chest, ' and in inches ', chest/2.54)


		side_thigh_dim = np.sqrt( (side_f_point[0] - side_s_point[0])*(side_f_point[0] - side_s_point[0]) + (side_f_point[1] - side_s_point[1])*(side_f_point[1] - side_s_point[1]))
		front_thigh_dim = np.sqrt( (final_pixel_loc[0] - start_pixel_loc[0])*(final_pixel_loc[0] - start_pixel_loc[0]) + (final_pixel_loc[1] - start_pixel_loc[1])*(final_pixel_loc[1] - start_pixel_loc[1]))
		side_thigh_in_cm = side_thigh_dim*side_cm_per_pixel
		front_thigh_in_cm = front_thigh_dim*front_cm_per_pixel
		r1 = front_thigh_in_cm/2
		r2 = side_thigh_in_cm/2
		thigh = 2*(22/7)*np.sqrt(0.5*r1*r1 + 0.5*r2*r2)
		print ('thigh_in_cm', thigh, ' and in inches ', thigh/2.54)

		
		front_sleeve_dim = np.sqrt( (sleeve_final_pixel_loc[0] - sleeve_start_pixel_loc[0])*(sleeve_final_pixel_loc[0] - sleeve_start_pixel_loc[0]) + (sleeve_final_pixel_loc[1] - sleeve_start_pixel_loc[1])*(sleeve_final_pixel_loc[1] - sleeve_start_pixel_loc[1]))
		front_sleeve_in_cm = front_sleeve_dim*front_cm_per_pixel
		print ('sleeve_in_cm', front_sleeve_in_cm, ' and in inches ', front_sleeve_in_cm/2.54)


		dis_array2 = []
		p1_thigh_dis = np.sqrt( (final_pixel_loc[0] - p1[1])*(final_pixel_loc[0] - p1[1]) + (final_pixel_loc[1] - p1[0])*(final_pixel_loc[1] - p1[0]))
		p2_thigh_dis = np.sqrt( (final_pixel_loc[0] - p2[1])*(final_pixel_loc[0] - p2[1]) + (final_pixel_loc[1] - p2[0])*(final_pixel_loc[1] - p2[0]))
		p3_thigh_dis = np.sqrt( (final_pixel_loc[0] - p3[1])*(final_pixel_loc[0] - p3[1]) + (final_pixel_loc[1] - p3[0])*(final_pixel_loc[1] - p3[0]))
		p4_thigh_dis = np.sqrt( (final_pixel_loc[0] - p4[1])*(final_pixel_loc[0] - p4[1]) + (final_pixel_loc[1] - p4[0])*(final_pixel_loc[1] - p4[0]))
		dis_array2.append(p1_thigh_dis)
		dis_array2.append(p2_thigh_dis)
		dis_array2.append(p3_thigh_dis)
		dis_array2.append(p4_thigh_dis)
		index2 = np.argmin(dis_array2)

		dis_array = []

		if index2 < 2:
			p1_thigh_dis = np.sqrt( (start_pixel_loc[0] - p1[1])*(start_pixel_loc[0] - p1[1]) + (start_pixel_loc[1] - p1[0])*(start_pixel_loc[1] - p1[0]))
			p2_thigh_dis = np.sqrt( (start_pixel_loc[0] - p2[1])*(start_pixel_loc[0] - p2[1]) + (start_pixel_loc[1] - p2[0])*(start_pixel_loc[1] - p2[0]))
			dis_array.append(p1_thigh_dis)
			dis_array.append(p2_thigh_dis)
		else:
			p3_thigh_dis = np.sqrt( (start_pixel_loc[0] - p3[1])*(start_pixel_loc[0] - p3[1]) + (start_pixel_loc[1] - p3[0])*(start_pixel_loc[1] - p3[0]))
			p4_thigh_dis = np.sqrt( (start_pixel_loc[0] - p4[1])*(start_pixel_loc[0] - p4[1]) + (start_pixel_loc[1] - p4[0])*(start_pixel_loc[1] - p4[0]))
			dis_array.append(p3_thigh_dis)
			dis_array.append(p4_thigh_dis)
		


		index = np.argmin(dis_array)
		dis = dis_array[index]
		dis_in_cm = dis*front_cm_per_pixel
		print ('length_in_cm', dis_in_cm, ' and in inches ', dis_in_cm/2.54)

		p = (start_pixel_loc[1], start_pixel_loc[0])
		p_ = (0, 0)

		if ((index == 0) & (index2 < 2)):
			p_ = (p1[0], p1[1])

		if ((index == 1) & (index2 < 2)):
			p_ = (p2[0], p2[1])

		if ((index == 0) & (index2 > 1)):
			p_ = (p3[0], p3[1])

		if ((index == 1) & (index2 > 1)):
			p_ = (p4[0], p4[1])

		

		cv2.arrowedLine(image, p_, p, (0,0,255), 2, tipLength = 0.03)
		cv2.arrowedLine(image, p, p_, (0,0,255), 2, tipLength = 0.03)

		cv2.imshow('image', image)
		cv2.imshow('side_image', side_image)
		



	else:
		print ("need to get clear images, Can not find the dimensions")



def remove_unwanted_person_seg(mask, Visualize=False):
	mask_array = np.array(mask, np.uint8)

	complete_body_array = 255*np.ones(mask_array.shape, np.uint8)
	x = (mask_array[:,:,0] == 0) & (mask_array[:,:,1] == 0) & (mask_array[:,:,2] == 0)
	complete_body_array[x, :] = [0,0,0]

	if Visualize:
		cv2.imshow('Raw mask', mask)

	complete_body_array = cv2.cvtColor(complete_body_array,cv2.COLOR_RGB2GRAY)

	(major, minor, _) = cv2.__version__.split(".")
	if (np.int(major) >= 4):
		contours, _ = cv2.findContours(complete_body_array, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	else:
		_, contours, _ = cv2.findContours(complete_body_array, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

	num_pts = 0
	index = -1
	for j in range(len(contours)):
		pts = cv2.contourArea(contours[j])
		if pts > num_pts:
			index = j
			num_pts = pts

	if index >= 0:
		complete_body_array = np.zeros(mask_array.shape, np.uint8)
		cv2.drawContours(complete_body_array, contours, index, (255,255,255), -1)
		x = (complete_body_array[:,:,0] == 0) & (complete_body_array[:,:,1] == 0) & (complete_body_array[:,:,2] == 0)
		mask[x, :] = [0,0,0]
	
	
	if Visualize:
		cv2.imshow('After processing', mask)
		cv2.waitKey(1)

	return mask



def get_sleeve_points(image, mask):
	mask_array = np.array(mask, np.uint8)
	arm_array = 0*mask_array

	x = (mask_array[:,:,0] == lower_arm_part[0]) & (mask_array[:,:,1] == lower_arm_part[1]) & (mask_array[:,:,2] == lower_arm_part[2])
	y = (mask_array[:,:,0] == upper_arm_part[0]) & (mask_array[:,:,1] == upper_arm_part[1]) & (mask_array[:,:,2] == upper_arm_part[2])
	arm_array[x, :] = [255, 255, 255]
	arm_array[y, :] = [255, 255, 255]

	part_bbox = get_person_bounding_box(arm_array)
	new_part_bbox = list(part_bbox)
	pixel_loc = np.where(arm_array > 0)

	chest_mask = get_chest_pixels_mask(mask)
	pixel_loc2 = np.where(chest_mask > 0)
	
	first_point = (new_part_bbox[0], new_part_bbox[1])
	second_point = (new_part_bbox[0] + new_part_bbox[2], new_part_bbox[1])

	# s1 = (first_point[0], first_point[1])
	# s2 = (second_point[0], second_point[1])
	# cv2.circle(image, s1, 10, (0,255,255), -1)
	# cv2.circle(image, s2, 10, (0,255,255), -1)

	# for i in range(pixel_loc2[0].shape[0]):
	# 	s1 = (pixel_loc2[1][i], pixel_loc2[0][i])
	# 	cv2.circle(image, s1, 1, (0,255,255), -1)


	### Calculating shoulder point
	f_dis_min = 10000
	f_dis_index = -1
	s_dis_min = 10000
	s_dis_index = -1

	for i in range(pixel_loc2[0].shape[0]):
		x = pixel_loc2[0][i]
		y = pixel_loc2[1][i]

		f_dis = np.sqrt((x - first_point[1])*(x - first_point[1]) + (y - first_point[0])*(y - first_point[0]))
		s_dis = np.sqrt((x - second_point[1])*(x - second_point[1]) + (y - second_point[0])*(y - second_point[0]))

		if (f_dis < f_dis_min):
			f_dis_min = f_dis
			f_dis_index = i

		if (s_dis < s_dis_min):
			s_dis_min = s_dis
			s_dis_index = i

	estimated_shoulder_loc1 = (pixel_loc2[0][f_dis_index], pixel_loc2[1][f_dis_index])
	estimated_shoulder_loc2 = (pixel_loc2[0][s_dis_index], pixel_loc2[1][s_dis_index])

	# s1 = (estimated_shoulder_loc1[1], estimated_shoulder_loc1[0])
	# s2 = (estimated_shoulder_loc2[1], estimated_shoulder_loc2[0])
	# cv2.circle(image, s1, 10, (0,255,0), -1)
	# cv2.circle(image, s2, 10, (0,255,0), -1)


	f1_dis = np.sqrt((estimated_shoulder_loc1[0] - first_point[1])*(estimated_shoulder_loc1[0] - first_point[1]) + (estimated_shoulder_loc1[1] - first_point[0])*(estimated_shoulder_loc1[1] - first_point[0]))
	s1_dis = np.sqrt((estimated_shoulder_loc1[0] - second_point[1])*(estimated_shoulder_loc1[0] - second_point[1]) + (estimated_shoulder_loc1[1] - second_point[0])*(estimated_shoulder_loc1[1] - second_point[0]))

	f2_dis = np.sqrt((estimated_shoulder_loc2[0] - first_point[1])*(estimated_shoulder_loc2[0] - first_point[1]) + (estimated_shoulder_loc2[1] - first_point[0])*(estimated_shoulder_loc2[1] - first_point[0]))
	s2_dis = np.sqrt((estimated_shoulder_loc2[0] - second_point[1])*(estimated_shoulder_loc2[0] - second_point[1]) + (estimated_shoulder_loc2[1] - second_point[0])*(estimated_shoulder_loc2[1] - second_point[0]))


	dis_array = np.array([f1_dis, s1_dis, f2_dis, s2_dis])
	index = np.argmin(dis_array)
	# print (dis_array)
	# print (index)
	final_pixel_loc = (-1, -1)

	if index == 0:
		final_pixel_loc = (pixel_loc2[0][f_dis_index], pixel_loc2[1][f_dis_index])

	if index == 1:
		final_pixel_loc = (pixel_loc2[0][f_dis_index], pixel_loc2[1][f_dis_index])
		second_point = (new_part_bbox[0], new_part_bbox[1])

	if index == 2:
		final_pixel_loc = (pixel_loc2[0][s_dis_index], pixel_loc2[1][s_dis_index])

	if index == 3:
		final_pixel_loc = (pixel_loc2[0][s_dis_index], pixel_loc2[1][s_dis_index])
		second_point = (new_part_bbox[0], new_part_bbox[1])


	### Calculating wrist point
	s_dis_min = 10000
	s_dis_index = -1

	for i in range(pixel_loc[0].shape[0]):
		x = pixel_loc[0][i]
		y = pixel_loc[1][i]

		s_dis = np.sqrt((x - second_point[1])*(x - second_point[1]) + (y - second_point[0])*(y - second_point[0]))
		if s_dis < s_dis_min:
			s_dis_min = s_dis
			s_dis_index = i

	start_pixel_loc = (pixel_loc[0][s_dis_index], pixel_loc[1][s_dis_index])


	p1 = (final_pixel_loc[1], final_pixel_loc[0])
	p2 = (start_pixel_loc[1], start_pixel_loc[0])
	
	##### To visualize wrist point
	# cv2.circle(image, p2, 10, (0,255,0), -1)


	##### To visualize shoulder point
	# cv2.circle(image, p1, 10, (0,255,0), -1)

	cv2.arrowedLine(image, p1, p2, (0,0,255), 2, tipLength = 0.05)
	cv2.arrowedLine(image, p2, p1, (0,0,255), 2, tipLength = 0.05)

	return image, start_pixel_loc, final_pixel_loc
	


def get_ankle_pixels(i1, m1):
	

	mask_array = 0*m1
	x =  (m1[:,:,0] == lower_legs_part[0]) & (m1[:,:,1] == lower_legs_part[1]) & (m1[:,:,2] == lower_legs_part[2])
	mask_array[x, :] = [255, 255, 255]

	skeleton = skeletonize(mask_array)
	x = (skeleton[:,:,0] > 0) | (skeleton[:,:,1] > 0) | (skeleton[:,:,2] > 0)
	skeleton[x] = [255, 255, 255]
	skeleton = np.array(skeleton, np.uint8)

	complete_body_array = cv2.cvtColor(mask_array,cv2.COLOR_RGB2GRAY)

	(major, minor, _) = cv2.__version__.split(".")
	if (np.int(major) >= 4):
		contours, _ = cv2.findContours(complete_body_array, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	else:
		_, contours, _ = cv2.findContours(complete_body_array, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

	points_x = []
	points_y = []

	for j in range(2):
		contour_mask = 0*skeleton
		cv2.drawContours(contour_mask, contours, j, (255,255,255), 1)
		rec_x, rec_y, rec_w, rec_h = cv2.boundingRect(contours[j])
		box_array = 0*m1
		cv2.rectangle(box_array, (rec_x, rec_y), (rec_x + rec_w, rec_y + rec_h), (255,255,255), -1)


		x = (skeleton[:,:,0] > 0) & (box_array[:,:,0] > 0) 
		h,w,c = skeleton.shape
		dst = np.zeros([h,w, c], np.uint8)
		dst[x] = [255, 255, 255]

		dst2 = np.zeros([h,w], np.uint8)
		dst2[x] = 255


		pixel_loc = np.where(dst > 0)
		kernel_size = 11
		score_array = []

		for j in range(np.array(pixel_loc).shape[1]):
			x = pixel_loc[0][j]
			y = pixel_loc[1][j]
			local_dst2 = np.zeros([kernel_size, kernel_size])
			local_dst2 = dst2[x-np.int(kernel_size/2):x+np.int(kernel_size/2)+1, y-np.int(kernel_size/2):y+np.int(kernel_size/2)+1]

			local_score = 0
			for d in range(kernel_size):
				if ( (local_dst2[d,d] > 0) or (local_dst2[d, kernel_size-d-1] > 0) ):
					local_score += 1

			score_array.append(local_score)

		
		index = np.argmax(score_array)
		x_min = pixel_loc[0][index]
		

		line_mask = 0*skeleton
		line_mask[x_min,:,:] = [255,255,255]

		x = (contour_mask[:,:,0] > 0) & (line_mask[:,:,0] > 0)
		line_mask = 0*skeleton
		line_mask[x,:] = [255,255,255]



		pixel_loc = np.where(line_mask > 0)

		min_index = np.argmin(pixel_loc[1])
		x_min = pixel_loc[0][min_index]
		y_min = pixel_loc[1][min_index]

		max_index = np.argmax(pixel_loc[1])
		x_max = pixel_loc[0][max_index]
		y_max = pixel_loc[1][max_index]


		points_x.append(x_min)
		points_y.append(y_min)

		points_x.append(x_max)
		points_y.append(y_max)

		# cv2.imshow('line_mask', line_mask)
		# cv2.imshow('contour_mask', contour_mask)
		# cv2.waitKey(0)

		


	
	p1 = (points_y[0], points_x[0])
	p2 = (points_y[1], points_x[1])
	p3 = (points_y[2], points_x[2])
	p4 = (points_y[3], points_x[3])

	# cv2.circle(i1, p1, 5, (255,0,0), -1)
	# cv2.circle(i1, p2, 5, (255,0,0), -1)
	# cv2.circle(i1, p3, 5, (0,255,0), -1)
	# cv2.circle(i1, p4, 5, (0,255,0), -1)
	# cv2.imshow('i1', i1)

	return p1, p2, p3, p4

