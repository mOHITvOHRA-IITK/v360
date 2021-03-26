import cv2
import numpy as np


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


	if visualize==True:
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

		cv2.imshow('front', image)
		cv2.waitKey(1)

	return waist_bbox[2], chest_bbox[2], person_bbox[3]



def get_side_chest_and_waist(image, mask, visualize=False):
	waist_mask = get_waist_pixels_mask(mask)
	waist_bbox = get_part_bounding_box(waist_mask)

	chest_mask = get_chest_pixels(mask)
	chest_bbox = get_part_bounding_box(chest_mask)

	person_bbox = get_person_bounding_box(mask)


	if visualize==True:
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

		cv2.imshow('side', image)
		cv2.waitKey(1)

	return waist_bbox[2], chest_bbox[2], person_bbox[3]



def get_human_head_torse_fraction_of_actual_height(image, mask):
	person_bbox = get_person_bounding_box(mask)


	mask_array = np.array(mask, np.uint8)
	head_torse_array = 0*mask_array


	x = (mask_array[:,:,0] == torse_part[0]) & (mask_array[:,:,1] == torse_part[1]) & (mask_array[:,:,2] == torse_part[2])
	head_torse_array[x, :] = [255, 255, 255]

	x = (mask_array[:,:,0] == head_part[0]) & (mask_array[:,:,1] == head_part[1]) & (mask_array[:,:,2] == head_part[2])
	head_torse_array[x, :] = [255, 255, 255]


	part_bbox = get_part_bounding_box(head_torse_array)

	p1 = (np.int(part_bbox[0] + 1.1*part_bbox[2]), part_bbox[1])
	p2 = (np.int(part_bbox[0] + 1.1*part_bbox[2]), np.int(part_bbox[1] + part_bbox[3]))
	cv2.arrowedLine(image, p1, p2, (255,0,255), 2, tipLength = 0.03)
	cv2.arrowedLine(image, p2, p1, (255,0,255), 2, tipLength = 0.03)

	cv2.imshow('image', image)

	return person_bbox[3], part_bbox[3]

	


def fit_rotated_rectangle_on_lower_leg(image, mask):

	mask_array = np.array(mask, np.uint8)
	lower_legs_array = 0*mask_array

	x = (mask_array[:,:,0] == lower_legs_part[0]) & (mask_array[:,:,1] == lower_legs_part[1]) & (mask_array[:,:,2] == lower_legs_part[2])
	lower_legs_array[x, :] = [255, 255, 255]


	part_bbox = get_person_bounding_box(lower_legs_array)
	part_bbox = [part_bbox[0], part_bbox[1], part_bbox[2], np.int(part_bbox[3]/2)]
	lower_legs_rec_array = 0*mask_array
	cv2.rectangle(lower_legs_rec_array, (part_bbox[0], part_bbox[1]), (part_bbox[0] + part_bbox[2], part_bbox[1] + part_bbox[3]), (255,255,255), -1)
	
	final_lower_leg_mask = np.zeros(mask.shape[0:2], np.uint8)
	x = (mask_array[:,:,0] == lower_legs_part[0]) & (mask_array[:,:,1] == lower_legs_part[1]) & (mask_array[:,:,2] == lower_legs_part[2]) & (lower_legs_rec_array[:,:,0] == 255) & (lower_legs_rec_array[:,:,1] == 255) & (lower_legs_rec_array[:,:,2] == 255)
	final_lower_leg_mask[x] = [255]
	

	(major, minor, _) = cv2.__version__.split(".")
	if (np.int(major) >= 4):
		contours, _ = cv2.findContours(final_lower_leg_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	else:
		_, contours, _ = cv2.findContours(final_lower_leg_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

	num_pts = 0
	index = -1
	for j in range(len(contours)):
		pts = cv2.contourArea(contours[j])
		if pts > num_pts:
			index = j
			num_pts = pts

	if index > -1:
		minAreaRect = cv2.minAreaRect(contours[index])

		bounding_box_centre = (minAreaRect[0][0], minAreaRect[0][1])
		bounding_box_dimensions = (minAreaRect[1][0], minAreaRect[1][1])
		# bounding_box_dimensions = (0, minAreaRect[1][1])
		bbox_angle = minAreaRect[2]
		# if bbox_angle < 0:
		# 	bbox_angle -=-90
		# else:
		# 	bbox_angle += 90

		minAreaRect = (bounding_box_centre, bounding_box_dimensions, bbox_angle)

		box = cv2.boxPoints(minAreaRect)
		box = np.int0(box)
		cv2.drawContours(image, [box], 0, (0,255,0), 2)

	cv2.imshow('final_lower_leg_mask', final_lower_leg_mask)

	p1 = (np.int(part_bbox[0] + 1.1*part_bbox[2]), part_bbox[1])
	p2 = (np.int(part_bbox[0] + 1.1*part_bbox[2]), np.int(part_bbox[1] + part_bbox[3]))
	cv2.arrowedLine(image, p1, p2, (255,0,255), 2, tipLength = 0.03)
	cv2.arrowedLine(image, p2, p1, (255,0,255), 2, tipLength = 0.03)

	cv2.imshow('image', image)




def fit_rotated_rectangle_on_thighs(image, mask, side_image, side_mask, visualize=False):

	mask = remove_unwanted_person_seg(mask)

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


		

		upper_legs_array [final_pixel_loc[0], final_pixel_loc[1],:] = [0,255,0]
		upper_legs_array [start_pixel_loc[0], start_pixel_loc[1],:] = [0,0,255]


		# if visualize:
		# 	# cv2.rectangle(upper_legs_array, (new_part_bbox[0], new_part_bbox[1]), (new_part_bbox[0] + new_part_bbox[2], new_part_bbox[1] + new_part_bbox[3]), (0,255,0), 2)
		# 	# cv2.drawContours(upper_legs_array, [box], 0, (0,255,0), 2)
		# 	cv2.imshow('upper_legs_array', upper_legs_array)
		# 	cv2.waitKey(0)
			

		


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


			# print ('min_x', min_x)
			# print ('max_x', max_x)
			# print ('start_pixel_loc[1]', start_pixel_loc[1])
			# print ('final_pixel_loc[1]', final_pixel_loc[1])

			# print ('final_point_x_min_diff', final_point_x_min_diff)
			# print ('final_point_x_max_diff', final_point_x_max_diff)
			# print ('start_point_x_min_diff', start_point_x_min_diff)
			# print ('start_point_x_max_diff', start_point_x_max_diff)

			# mid_pixel_loc1 = (new_part_bbox[1], min_x)
			# mid_pixel_loc2 = (new_part_bbox[1], max_x)
			# temp_array [final_pixel_loc[0], final_pixel_loc[1],:] = [0,255,0]
			# temp_array [start_pixel_loc[0], start_pixel_loc[1],:] = [0,0,255]
			# temp_array [mid_pixel_loc1[0], mid_pixel_loc1[1],:] = [255,255,0]
			# temp_array [mid_pixel_loc2[0], mid_pixel_loc2[1],:] = [0,255,255]
			# cv2.imshow('temp_array', temp_array)
			# cv2.waitKey(1)


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



		upper_legs_only_array [final_pixel_loc[0], final_pixel_loc[1],:] = [0,255,0]
		upper_legs_only_array [start_pixel_loc[0], start_pixel_loc[1],:] = [0,0,255]
		upper_legs_only_array [mid_pixel_loc[0], mid_pixel_loc[1],:] = [255,0,0]
		upper_legs_only_array [forth_pixel_loc[0], forth_pixel_loc[1],:] = [255,0,255]


		if visualize:
			p1 = (final_pixel_loc[1], final_pixel_loc[0])
			p2 = (start_pixel_loc[1], start_pixel_loc[0])
			# cv2.arrowedLine(mask, p1, p2, (255,0,255), 2, tipLength = 0.03)
			# cv2.arrowedLine(mask, p2, p1, (255,0,255), 2, tipLength = 0.03)
			# cv2.imshow('mask', mask)

			cv2.arrowedLine(image, p1, p2, (255,0,255), 2, tipLength = 0.03)
			cv2.arrowedLine(image, p2, p1, (255,0,255), 2, tipLength = 0.03)
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

		if visualize:
			p1 = (side_f_point[1], side_f_point[0])
			p2 = (side_s_point[1], side_s_point[0])
			

			cv2.arrowedLine(side_image, p1, p2, (255,0,255), 2, tipLength = 0.03)
			cv2.arrowedLine(side_image, p2, p1, (255,0,255), 2, tipLength = 0.03)
			cv2.imshow('side_image', side_image)





def get_measuremnets(i1, m1, i2, m2, visualize=False):
	get_front_chest_and_waist(i1, m1)
	get_side_chest_and_waist(i1, m1)
	fit_rotated_rectangle_on_thighs(i1, m1, i2, m2, visualize)


			




			

			



	
	





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








