import numpy as np


# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!
    
    Rover.left_samp_angles = np.where(Rover.rock_angles * 180/np.pi > -10)[0]
    
   
    # If rocks were detected within 3 meters of known sample positions
    # consider it a success and plot the location of the known
    # sample on the map
    
    if Rover.samples_collected >= 5:
        print("GO TO START")
        dist_start = np.sqrt((Rover.pos[0] - Rover.start_pos[0])**2 + (Rover.pos[1] - Rover.start_pos[1])**2)
        # Make sure we are heading in right general direction
        # TODO
        # If we are in 10 meters steer to starting point
   
        # If we are in 15 meters just stop
        if dist_start < 15.0 :
            print("15m From start")
            Rover.mode = 'stop'
            Rover.throttle = 0
            Rover.brake = Rover.brake_set
            return Rover
    
    # If we are picking_up a rock make a note of it 1 time
    if Rover.picking_up == 1:
        print("Picking Up")
        if Rover.picking_up == False:
            Rover.picking_up = True
            Rover.samples_collected += 1
        return Rover
    else:
         Rover.picking_up = False
    
    if Rover.near_sample == 1:
        print("Stopping at sample")
        Rover.mode = 'stop'
        Rover.throttle = 0
        Rover.brake = Rover.brake_set
        if Rover.vel == 0: 
            Rover.send_pickup = True
        return Rover  
        
        
    if Rover.mode == 'stuck':
        if Rover.stuck_mode == 'forward':
            print("Stuck Forward")
            Rover.throttle = 1
            #Rover.steer = 0
            Rover.stuck_counter = Rover.stuck_counter + 1
            print(Rover.stuck_counter)
            if Rover.stuck_counter > 45:
                Rover.stuck_mode = 'yaw'
                Rover.stuck_counter = 0
        elif Rover.stuck_mode == 'yaw':
            print("Stuck Yaw")
            Rover.throttle = 0
            Rover.steer = -15
            Rover.stuck_counter += 1
            if Rover.stuck_counter > 20:
                Rover.stuck_mode = 'forward'
                Rover.stuck_counter = 0

        if Rover.vel > 0.6:
            Rover.mode = 'forward'

        return Rover
        
        
    if Rover.mode == 'forward':
        if Rover.vel < 0.5:
            Rover.stuck_counter += 1
    else:
        Rover.stuck_counter = 0
        
    if Rover.stuck_counter > 90:
        Rover.mode = 'stuck'
        Rover.stuck_mode = 'forward'
        Rover.stuck_counter = 0
        
    # Get the left half of nav angles to wall crawl
    Rover.nav_angles = np.sort(Rover.nav_angles)[-int(len(Rover.nav_angles)/2):]

    # Example:
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        # Check for Rover.mode status
        if Rover.mode == 'forward':
        
            # Check for samples to collect
            if len(Rover.left_samp_angles) > 1:
                print("Approaching sample")

                if Rover.vel < 0.75: #when finding sample approach slower
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                    
                Rover.brake = 0

                Rover.steer = np.clip(np.mean(Rover.rock_angles * 180/np.pi), -15, 15)    
        
            # Check the extent of navigable terrain
            elif len(Rover.nav_angles) >= Rover.stop_forward:  
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                # Set steering to average angle clipped to the range +/- 15
                Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.nav_angles) < Rover.stop_forward:
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.mode = 'stop'

        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
    		
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                 # Check for samples to collect
                if len(Rover.left_samp_angles) > 1:
                    print("Approaching sample")
                    Rover.steer = np.clip(np.mean(Rover.rock_angles * 180/np.pi), -15, 15)            
                    Rover.throttle = Rover.throttle_set
                    Rover.brake = 0 
           
                # Now we're stopped and we have vision data to see if there's a path forward
                elif len(Rover.nav_angles) < Rover.go_forward:
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = -15 # Could be more clever here about which way to turn
                    
                # If we're stopped but see sufficient navigable terrain in front then go!
                if len(Rover.nav_angles) >= Rover.go_forward:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    Rover.mode = 'forward'
    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0
 
    
    return Rover

