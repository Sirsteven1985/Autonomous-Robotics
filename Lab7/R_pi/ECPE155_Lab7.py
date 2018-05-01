# Lab 7 using Potential Fields for Obstacle Avoidance
# Authors: Steve Guerrero & Paul Vuong
import struct
import serial
import string
import math
import sys
import time
import array
from time import sleep

# Begin program with user defined target localization with respect to robots pose
print('\nSelect a target in x,y Cartesian format \n')
dx,dy = raw_input("ENTER x y coordinates separated by space\n").split()
## Variables for localization
# user defined target location
dest_x = int(dx)
dest_y = int(dy)
true = 1
false = 0
# robots initial pose
rob_x = 0
rob_y = 0
rob_theta = 0
home = 0
mag_f = 1.1
ROB_POSE = [rob_x, rob_y, rob_theta]
count = 0;	# track the robots forward movement
big_D = 15 # max distance of IR sensor
# robots parameters
pi = 3.14159265359
L_rad = 2.92	# measured radius of left wheel
R_rad = 2.91		# measured radius of right wheel
L = 4.65		# measured center of wheel to axis of rotation
#distance squared to use for calculated target distance
dist = (((dest_x - rob_x) * (dest_x - rob_x)) + (dest_y - rob_y) * (dest_y - rob_y))
#radian angle 
rad = math.atan2((dest_y - rob_y),(dest_x - rob_x))
#calculated from input data
target_dist = math.sqrt(dist)

#initial turn angle
degrees = math.degrees(rad)
    
# Attractive vectr pot-field
att_vect = [(dest_x - rob_x), (dest_y - rob_y)]
rep_vect = [0,0]
# connection to Tiva board
port = serial.Serial("/dev/ttyS0",baudrate=115200,timeout=15.0)
print'Initializing robot pose to:', ROB_POSE
print'Distance to target is:' ,target_dist,' cm at angle:',degrees
print('ATT',att_vect)
degrees = int(degrees)
# Tiva motor control commands
turn_cmd = 0x2
fwd_cmd = 0x3
rev_cmd = 0x4
brake_cmd = 0x5
FWD_cmd = 0x6
rot_CW = 0x7
rot_CCW = 0x8
#start byte
s_B = struct.pack('!B', 0x9)
port.write(s_B)
#unfinished initial comands to tiva 1st turn toward target and drive toward target
print'init move (d,cm):', degrees, target_dist
##if(rad < pi):
##    movement = struct.pack('!Bh', rot_CCW, 0x0000)
##    port.write(movement)
##else:
##    movement = struct.pack('!Bh', rot_CW, 0x0000)
##    port.write(movement)
orig_dist = target_dist
print'init move (d,cm):', degrees, target_dist
movement = struct.pack('!Bh', turn_cmd, degrees)
port.write(movement)
rob_theta = rad
return_rad = 0
sleep(1)
FWD_comp = 3
turn_comp = 2
Q = True
degrees = 0
rad = 0

data = [0,0,0,0,0,0,0,0] # temp data array for proper data alignment

#Coefficients for IR sensor characterization
Fx5_1 = 0.00000085153
Fx4_1 = -0.00026901
Fx3_1 = 0.0333
Fx2_1 = -2.0731
Fx1_1 = 68.6697
      
Fx4_2 = -0.00000094636
Fx3_2 = 0.00072417
Fx2_2 = -0.2043
Fx1_2 = 24.6370

Lx5 = 0.000000067117
Lx4 = -0.000049075
Lx3 = 0.0134
Lx2 = -1.6890
Lx1 = 95.9798

Rx5 = 0.000000078776
Rx4 = -0.000058803
Rx3 = 0.0163
Rx2 = -2.05
Rx1 = 113.3755
FLAG = false
inch = 0
rad_ang = 0
#start_time = time.time()	# used to aquire del_t for kinematic model update
###################### MAIN LOOP ######################
while Q == True:
    
    if((FWD_comp == 3) and (FLAG == false)): # finished forward motor movement
    	#move forward
        inch = 2
        movement = struct.pack('!BH', fwd_cmd, inch)
	port.write(movement)
	print'Inching FWD'
	FWD_comp = 0
	count = count + 1
	sleep(0.1)
##	stuff = struct.pack('!BH', 0x1, 0x3)## reset MCU_CMD on tiva
##	port.write(stuff)

    ################# DATA AQCUISITION ################
    # Poll until packet of data received from tiva

    rcv = port.read(8)
    (A0) = struct.unpack("!BBBBBBBB",rcv)
	# restructure of data packet in case it's out of order
    for n in range(0,7):
        if A0[n] == 0xFF:
            pp = n
    for f in range(0,7):
        data[f] = A0[(f+pp)%9]
    # proper order of data packet:
    # Start_Byte, ADC_F, ADC_L, ADC_R, L_SPD, R_SPD, Bits(l_dir,r_dir,l_bump,r_bump)
    x_f = data[1]	# front sensor 8 bit value
    x_l = data[2]	# left sensor 8 bit value
    x_r = data[3]	# right sensor 8 bit value
    Left_bump = data[6]
    Right_bump = data[7]
    FWD_MCU = data[4]
    print(data)
    TURN_MCU = data[5]
    print'FWD CMD = ',FWD_MCU, 'TURNCMD =',TURN_MCU

    ##################################################
    if(TURN_MCU == 2):
        turn_comp = TURN_MCU
        TURN_MCU = 0
    if(FWD_MCU == 3):
        FWD_comp = FWD_MCU
        FWD_MCU = 0
    if(Left_bump == 1):
        rob_x += -(8 * math.cos(rob_theta))
	rob_y += -(8 * math.sin(rob_theta))
	rob_theta += -(pi/2)
	Left_bump = 0
##	stuff = struct.pack('!BH', 0x1, 0x5)## reset MCU_CMD on tiva
##	port.write(stuff)
	sleep(1.5)
    if ((rob_theta > pi)):
        rob_theta = rob_theta - (2*pi)
    if (rob_theta < -(pi)):
        rob_theta = rob_theta + (2*pi)
    if(Right_bump == 1):
	rob_x += -(8 * math.cos(rob_theta))
	rob_y += -(8 * math.sin(rob_theta))
	rob_theta += (pi/2)
	Right_bump = 0
##	stuff = struct.pack('!BH', 0x1, 0x5)## reset MCU_CMD on tiva
##	port.write(stuff)
	sleep(1.5)
    if ((rob_theta > pi)):
        rob_theta = rob_theta - (2*pi)
    if (rob_theta < -(pi)):
        rob_theta = rob_theta + (2*pi)
##    if(turn_comp == 2): ## Turn angle
##	rob_theta += rad_ang
##        rad_ang = 0
    if(FWD_comp == 3): ## FWD_CENT
	rob_x += (inch * math.cos(rob_theta))
	rob_y += (inch * math.sin(rob_theta))
        inch = 0
    ############ KINEMATIC MODEL #############
    # update in the robots pose after FKM update
    # keep theta in range (-pi,pi)
    if ((rob_theta > pi)):
        rob_theta = rob_theta - (2*pi)
    if (rob_theta < -(pi)):
        rob_theta = rob_theta + (2*pi)
    print'count',count,'flag',FLAG
    # update attractive vector after robot movemnet
    att_vect = [(dest_x - rob_x), (dest_y - rob_y)]
    

    ROB_POSE = [rob_x, rob_y, rob_theta]
    print'Pose now:', ROB_POSE
    
    # functions for characterizing the IR sensors and converting to distance in cm
##    if x_f < 105:	# peacewise functions for front sensor only
##        dist_f = ((Fx5_1*math.pow(x_f,4)) + (Fx4_1*math.pow(x_f,3)) + (Fx3_1*math.pow(x_f,2)) + (Fx2_1*x_f) + Fx1_1)
##    if x_f >= 105:
##        dist_f = ((Fx4_2*math.pow(x_f,3)) + (Fx3_2*math.pow(x_f,2)) + (Fx2_2*x_f) + Fx1_2)
    #temp front
    dist_f = ((Lx5*math.pow(x_f,4)) + (Lx4*math.pow(x_f,3)) + (Lx3*math.pow(x_f,2)) + (Lx2*x_f) + Lx1)

    # left sensor conversion
    #dist_l = ((Lx5*math.pow(x_l,4)) + (Lx4*math.pow(x_l,3)) + (Lx3*math.pow(x_l,2)) + (Lx2*x_l) + Lx1)
    #temp left
    dist_l = 55
    # right sensor conversion
    dist_r = ((Rx5*math.pow(x_r,4)) + (Rx4*math.pow(x_r,3)) + (Rx3*math.pow(x_r,2)) + (Rx2*x_r) + Rx1)
    
	# converting floating point values to integers
    dist_f = int(dist_f) # convert Front IR sensor distance to integer
    dist_l = int(dist_l) # convert Left IR sensor distance to integer
    dist_r = int(dist_r) # convert Right IR sensor distance to integer

    
    dist = (((dest_x - rob_x)*(dest_x - rob_x)) + ((dest_y - rob_y)*(dest_y - rob_y)))
    #calculated from input data
    target_dist = math.sqrt(dist)
    print('TARGET IS:',target_dist)
    #### ALSO CHECK IF ROBOT POSE COMPONENTS ARE GREATER THAN THE DESTINATION COMPONENTS##
    target_angle = math.atan2((dest_y - rob_y),(dest_x - rob_x))

##    tang = target_angle - rob_theta
##
    print'TANG:', target_angle

        
##    print'dist:',dist_f,dist_l,dist_r
##    print'bump', Left_bump,Right_bump
    
    lil_D = min(dist_f,dist_l,dist_r) # measured distances
    check = [dist_f,dist_l,dist_r]
    print(check)
    print('lil',lil_D)
    if (lil_D < big_D):
        FLAG = true
    else:
        FLAG = false
        
    ######################### Pot Fields ##############################
    ###################################################################
    if((FLAG == true) and (turn_comp == 2)):
        # calculate repulsive vector from IR data
    
        if (lil_D == dist_f):
            keep = 0
        elif (lil_D == dist_l):
            keep = 1
        else:
            keep = 2
       
        mag_f = float(big_D)- float(lil_D)
        mag_f = mag_f/(float(big_D))

        norm = (att_vect[0]*att_vect[0]) + (att_vect[1]*att_vect[1])
        norm = math.sqrt(norm)
        att_vect[0] = (att_vect[0]/norm)
        att_vect[1] = (att_vect[1]/norm)

        
        if(keep ==0):
            Turn = -0.6*pi
        if(keep == 1):
            Turn = pi/3
        if(keep == 2):
            Turn = (-pi/3)

        r1 = mag_f*math.cos(rob_theta - Turn)
        r2 = mag_f*math.sin(rob_theta - Turn)
        rep_vect = [r1, r2]
        sum_vect = [(att_vect[0] - rep_vect[0]), (att_vect[1] - rep_vect[1])]
        rad = math.atan2(sum_vect[1],sum_vect[0])

##        
##        if(abs(rad - rob_theta) <  abs(rob_theta - rad)):
##            rad = (rad - rob_theta)
##        else:
##            rad = (rob_theta - rad)
##            
##        print'radA =',rad
        degrees = math.degrees(rad)
        degrees = int(degrees)
        print'ATT', att_vect, 'REP', rep_vect,'SUM', sum_vect,'degrees',degrees
        movement = struct.pack('!Bh', turn_cmd, degrees)
        port.write(movement)    	
        rob_theta += rad
        return_rad += rad
        if ((rob_theta > pi)):
            rob_theta = rob_theta - (2*pi)
        if (rob_theta < -(pi)):
            rob_theta = rob_theta + (2*pi)
	rad = 0
	turn_comp = 0
	
	count = 0
	sleep(0.2)
	
	################## RE-ALIGNMENT #################
        # if the robot is aiming at the target + or - 10 degree error 
    # then robot drive forward
    # allowable error increases as robot moves forward so it doesnt dance too much
    if((FLAG == false) and (count >= 12) and (turn_comp == 2) and (lil_D >= 19) ):
##        if((abs(target_angle - rob_theta) < (.17 * (orig_dist/target_dist))) or (abs(rob_theta - target_angle) < (.17 * (orig_dist/target_dist)))):
        if((abs(target_angle - rob_theta) < .17 ) or (abs(rob_theta - target_angle) < .17 )):
            print'within range less'
        elif((abs(target_angle - rob_theta) >  6.2 ) or (abs(rob_theta - target_angle) > 6.2)):
            print'within range more'
        elif(home >= 1):
            print'ELF'
            print'new_ang =', new_angle
            if((dest_y > rob_y) and (dest_x > rob_x) and (rob_theta > 0) and (new_angle > 0)):
                if(rob_theta > new_angle):
                    rad_move = -(rob_theta - new_angle) #RAD moves
                    deg_move = math.degrees(rad_move)
                    movement = struct.pack('!Bh', turn_cmd, deg_move)
                    port.write(movement)
                    print'deg_move',deg_move
                    count = 0
                    print'CCW 00'
                    rob_theta += rad_move
                    
                else:
                    rad_move = (new_angle - rob_theta) #RAD moves
                    deg_move = math.degrees(rad_move)
                    movement = struct.pack('!Bh', turn_cmd, deg_move)
                    port.write(movement)
                    print'deg_move',deg_move                 
                    count = 0
                    print'CW 00'
                    rob_theta += rad_move
                    
            elif((dest_y > rob_y) and (dest_x < rob_x) and (rob_theta > 0) and (new_angle > 0)):
                if(rob_theta > new_angle):
                    rad_move = (rob_theta - new_angle) #RAD moves
                    deg_move = math.degrees(rad_move)
                    movement = struct.pack('!Bh', turn_cmd, deg_move)
                    port.write(movement)
                    print'deg_move',deg_move
                    count = 0
                    print'CCW 01'
                    rob_theta += rad_move
                    
                else:
                    rad_move = (new_angle - rob_theta) #RAD moves
                    deg_move = math.degrees(rad_move)
                    movement = struct.pack('!Bh', turn_cmd, deg_move)
                    port.write(movement)
                    print'deg_move',deg_move                 
                    count = 0
                    print'CW 01'
                    rob_theta += rad_move
                    
            else:#((dest_y > rob_y) and (dest_x > rob_x) and (rob_theta > 0) and (new_angle > 0)):
                rad_move = (new_angle - rob_theta) #RAD moves
                deg_move = math.degrees(rad_move)
                movement = struct.pack('!Bh', turn_cmd, deg_move)
                port.write(movement)
                print'deg_move',deg_move                 
                count = 0
                print'CW 01'
                rob_theta += rad_move
            sleep(0.2)
            count = 0
            home = home - 1
        else: # if robot is not pointing at target, then brake and calculate turning amount for robot

            new_angle = math.atan2((dest_y-rob_y),(dest_x-rob_x))
            return_rad = -(return_rad)
            deg_move = math.degrees(return_rad)
            movement = struct.pack('!Bh', turn_cmd, deg_move)
            port.write(movement)
            print'deg_move',deg_move
            count = 0
            print'pot'
            rob_theta += return_rad
            return_rad = 0
            home = home +1
            
            new_angle = math.atan2((dest_y-rob_y),(dest_x-rob_x))
          
            sleep(0.5)

    if (target_dist < 10):
        print'TARGET REACHED!!!!!!!!!!!'
        print'YAHOOOOOOOOOO!!!!!!!!!!!'
        movement = struct.pack('!Bh', 0x5,0x0000)
        port.write(movement)
        sleep(1)
        sys.exit()

