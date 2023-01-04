import numpy as np
import cv2

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = ((img[:,:,0] > rgb_thresh[0]) & (img[:,:,1] >  rgb_thresh[1])  & (img[:,:,2] >  rgb_thresh[2]))
             
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select

# Define a function to convert from image coords to rover coords
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1]/2 ).astype(np.float)
    return x_pixel, y_pixel
    
def rock_thres(img, rgb_thresh=(110, 110, 50)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] < rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    kernel = np.ones((3,3),np.uint8)
    dilation = cv2.dilate(color_select,kernel,iterations = 2)
    
    
    # Return the binary image

    return dilation



# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixscalel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

# Define a function to map rover space pixels to world space
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))
                            
    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
    # Return the result  
    return xpix_rotated, ypix_rotated

def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    # Apply a scaling and a translation
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos
    # Return the result  
    return xpix_translated, ypix_translated


# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world

# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):
           
    Matrix = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, Matrix, (img.shape[1], img.shape[0]))# keep same size as input image
    
    mask = cv2.warpPerspective(np.ones_like(img[:, :, 0]), Matrix,
                               (img.shape[1], img.shape[0]),
                               cv2.BORDER_CONSTANT, borderValue=0)
                               
    mask = mask[35:160, 80:240]
    # Adding a black border to cropped mask to regain original shape of (160, 320)
    mask = cv2.copyMakeBorder(
        mask, 35, 0, 80, 80, cv2.BORDER_CONSTANT, value=(0, 0, 0))
                               
    return warped, mask


  
        # Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover, debug):

	# Perform perception steps to update Rover()
	# TODO: 
	# NOTE: camera image is coming to you in Rover.img
	
	if Rover.start_pos == None:
        	Rover.start_pos = Rover.pos

	image = Rover.img
	
	if Rover.perception_count == None:
        	Rover.perception_count = 0

	# 1) Define source and destination points for perspective transform


	dst_size = 5 #destination frame size 5x5 
	bottom_offset = 6
	source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
	destination = np.float32([[image.shape[1]/2 - dst_size, image.shape[0] - bottom_offset],
		      [image.shape[1]/2 + dst_size, image.shape[0] - bottom_offset],
		      [image.shape[1]/2 + dst_size, image.shape[0] - 2*dst_size - bottom_offset], 
		      [image.shape[1]/2 - dst_size, image.shape[0] - 2*dst_size - bottom_offset],
		      ])
		      
	if(debug):

		# 2) Apply perspective transform

		warped = perspect_transform(image, source, destination)

		# 3) Apply color threshold to identify navigable terrain/obstacles/rock samples


		obstacles_threshed = color_thresh(~warped)

		rock_warped = perspect_transform(image, source, destination)

		road_threshed = color_thresh(warped)

		rocks_threshed = rock_thres(rock_warped)
			   
			   
		cv2.imwrite("debugger/ "+ str(Rover.total_time) + "original.jpg", image)
		cv2.imwrite("debugger/ "+str(Rover.total_time) + "threshed.jpg", road_threshed)
		cv2.imwrite("debugger/ "+str(Rover.total_time) + "warped.jpg", warped)
		cv2.imwrite("debugger/ "+str(Rover.total_time) + "obstacles.jpg", obstacles_threshed)
		cv2.imwrite("debugger/ "+str(Rover.total_time) + "rock.jpg", rocks_threshed)
		    
	    
	else:

		# 2) Apply perspective transform

		warped, mask = perspect_transform(image, source, destination)

		# 3) Apply color threshold to identify navigable terrain/obstacles/rock samples

		road_threshed = color_thresh(warped) * mask
		
		obstacles_threshed = np.absolute(np.float32(road_threshed) - 1) * mask


		rocks_threshed = rock_thres(warped)



	# 4) Update Rover.vision_image (this will be displayed on left side of screen)warped, mask

	Rover.vision_image[:,:,0] = obstacles_threshed * 255 #obstacle color-thresholded binary image
	Rover.vision_image[:,:,1] = rocks_threshed * 255 #rock_sample color-thresholded binary image
	Rover.vision_image[:,:,2] = road_threshed * 255 #navigable terrain color-thresholded binary image


	# 5) Convert map image pixel values to rover-centric coords

	road_xpix, road_ypix = rover_coords(road_threshed) 
	road_dist, road_angles = to_polar_coords(road_xpix, road_ypix)

	obs_xpix, obs_ypix = rover_coords(obstacles_threshed)  

	rock_xpix, rock_ypix = rover_coords(rocks_threshed) 

	# 6) Convert rover-centric pixel values to world coordinates

	scale = 16
	worldsize = Rover.worldmap.shape[0]

	xpos = Rover.pos[0]
	ypos = Rover.pos[1]
	yaw =  Rover.yaw

	road_world_x,road_world_y, = pix_to_world(road_xpix, road_ypix, xpos, ypos ,yaw, worldsize, scale) 
	obs_world_x,obs_world_y, = pix_to_world(obs_xpix, obs_ypix, xpos, ypos ,yaw, worldsize, scale) 
	rock_world_x, rock_world_y = pix_to_world(rock_xpix, rock_ypix, xpos, ypos ,yaw, worldsize, scale)



	# 7) Update Rover worldmap (to be displayed on right side of screen)
		
	if ((Rover.roll < 0.30 or Rover.roll > 359.7) or (Rover.pitch < 0.3 or Rover.pitch > 359.7)):

		#Rover.worldmap[road_world_y,road_world_x,2] += 1
		#Rover.worldmap[obs_world_y,obs_world_x,0] = 255
		Rover.worldmap[road_world_y,road_world_x,2] = 255
		
		
		Rover.worldmap[rock_world_y, rock_world_x,1] = 255
		
		
		#Rover.worldmap[road_world_y,road_world_x,2] = 255
		
		Rover.worldmap[obs_world_y,obs_world_x,0] = 255

			
	if(Rover.perception_count % 200 == 0):
		nav_pix = Rover.worldmap[:,:,2] > 0
		lowqual_pix = Rover.worldmap[:,:,2] < np.mean(Rover.worldmap[nav_pix, 2]) / 2
		Rover.worldmap[lowqual_pix, 2] = 0
		
	
		
	#clear road
	is_road = Rover.worldmap[:,:,2] > 0
	Rover.worldmap[is_road,0] = 0


	# 8) Convert rover-centric pixel positions to polar coordinates
	# Update Rover pixel distances and angles

	distance, angles = to_polar_coords(road_xpix, road_ypix)
	Rover.nav_dists = road_dist
	Rover.nav_angles = road_angles
	
	
	rock_dists, rock_angles = to_polar_coords(rock_xpix, rock_ypix)
	Rover.rock_dists = rock_dists
	Rover.rock_angles = rock_angles
	Rover.perception_count +=1

	cv2.imshow('original image' , image)
	cv2.imshow('warped image' , warped)
	cv2.imshow('road threshed image', road_threshed * 255)
	cv2.imshow('obstacles threshed image', obstacles_threshed * 255)
	cv2.imshow('rocks threshed image', rocks_threshed * 255)
	


	cv2.waitKey(1)
	
	return Rover

