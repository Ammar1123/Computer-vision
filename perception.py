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
    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] > rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select

def rock_thres(img, rgb_thresh=(110, 110, 40)):
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


# Define a function to convert from image coords to rover coords
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1]/2 ).astype(np.float)
    return x_pixel, y_pixel


# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
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
           
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    
    return warped




# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):


    image = Rover.img
    dst_size = 5 
    bottom_offset = 6
    source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    destination = np.float32([[image.shape[1]/2 - dst_size, image.shape[0] - bottom_offset],
                      [image.shape[1]/2 + dst_size, image.shape[0] - bottom_offset],
                      [image.shape[1]/2 + dst_size, image.shape[0] - 2*dst_size - bottom_offset], 
                      [image.shape[1]/2 - dst_size, image.shape[0] - 2*dst_size - bottom_offset],
                      ])
    
    warped = perspect_transform(image, source, destination)

    mask = perspect_transform(np.ones_like(image)*255, source, destination)
    mask[0:60,:] = 0
    mask[:,0:50] = 0
    mask[:,270:] = 0


    rock_mask = np.copy(mask)
    rock_mask[0:110,:] = 0


    obstacles = color_thresh(~warped & mask)
    kernel = np.ones((5,5),np.uint8)
    obstacles = cv2.dilate(obstacles,kernel,iterations = 2)

    rock_warped = perspect_transform(image, source, destination)
    
    road = color_thresh(warped & mask)
    kernel = np.ones((3,5),np.uint8)
    road = cv2.erode(road,kernel,iterations = 3)
    
    rocks = rock_thres(rock_warped)

    Rover.vision_image[:,:,2] = road * 255
    Rover.vision_image[:,:,0] = obstacles * 255

    road_xpix, road_ypix = rover_coords(road)    
    obs_xpix, obs_ypix = rover_coords(obstacles)    
    rock_xpix, rock_ypix = rover_coords(rocks)
    
    scale = 10
    worldsize = 200
    xpos = Rover.pos[0]
    ypos = Rover.pos[1]
    yaw =  Rover.yaw
    
    road_world_x,road_world_y, = pix_to_world(road_xpix, road_ypix, xpos,ypos ,yaw, worldsize, scale) 
    obs_world_x,obs_world_y, = pix_to_world(obs_xpix, obs_ypix, xpos,ypos ,yaw, worldsize, scale) 
    rock_world_x, rock_world_y = pix_to_world(rock_xpix, rock_ypix, xpos,ypos ,yaw, worldsize, scale)

    Rover.worldmap[road_world_y,road_world_x,2] = 255
    Rover.worldmap[obs_world_y,obs_world_x,0] = 255
    Rover.worldmap[rock_world_y, rock_world_x,1] = 255
    
    #clear road
    is_road = Rover.worldmap[:,:,2] > 0
    Rover.worldmap[is_road,0] = 0

    distance, angles = to_polar_coords(road_xpix, road_ypix)
    Rover.nav_angles = angles

    
 
    
    
    return Rover
