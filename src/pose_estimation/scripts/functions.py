import cv2
import numpy as np

upper_legs_part = [128,0,128]
torse_part = [0,128,0]



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

	rec_x, rec_y, rec_w, rec_h = cv2.boundingRect(contours[index])
	cv2.rectangle(complete_body_array, (rec_x, rec_y), (rec_x + rec_w, rec_y + rec_h), 255, 2)

	if visualize_bbox:
		cv2.imshow('complete_body_array', complete_body_array)
		cv2.waitKey(1)

	return rec_x, rec_y, rec_w, rec_h




def get_waist_bounding_box(waist_mask, visualize_bbox=False):
	waist_mask = np.array(waist_mask, np.uint8)
	waist_array = cv2.cvtColor(waist_mask,cv2.COLOR_RGB2GRAY)

	(major, minor, _) = cv2.__version__.split(".")
	if (np.int(major) >= 4):
		contours, _ = cv2.findContours(waist_array, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	else:
		_, contours, _ = cv2.findContours(waist_array, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

	num_pts = 0
	index = -1
	for j in range(len(contours)):
		pts = cv2.contourArea(contours[j])
		if pts > num_pts:
			index = j
			num_pts = pts

	rec_x, rec_y, rec_w, rec_h = cv2.boundingRect(contours[index])
	cv2.rectangle(waist_array, (rec_x, rec_y), (rec_x + rec_w, rec_y + rec_h), 255, 2)

	if visualize_bbox:
		cv2.imshow('waist_array', waist_array)
		cv2.waitKey(1)

	return rec_x, rec_y, rec_w, rec_h



def get_person_front_measurements(image, mask, visualize=False):
	waist_mask = get_waist_pixels_mask(mask)
	waist_bbox = get_waist_bounding_box(waist_mask)
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
	waist_bbox = get_waist_bounding_box(waist_mask)
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

