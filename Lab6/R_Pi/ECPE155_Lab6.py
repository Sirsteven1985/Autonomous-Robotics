import struct
import serial
import string
import math
import sys
import time
from time import sleep

port = serial.Serial("/dev/ttyS0",baudrate=115200,timeout=3.0)
    
Q = True
timeout = 10 #10seconds
ts = time.time()
degrees = 0
distance = 0
rad = 0
fwd_x = 0
fwd_y = 0
a_tan = 0
OA_toggle = 0

print('\n\r Pose intialized to [0 0 0]')
while Q == True:
    
    print("\n\rWelcome to the TEAM MAGA Interface.")
    print("\n\r*-America Thanks You-*")
    print("\n\rNow,choose one of the following functions.")
    print("\n(1) Turn Angle (degrees)")
    print("\n(2) Drive Forward (cm)")
    print("\n(3) Toggle Obstacle Avoidance")
    print("\n(4) Violently Self-Destruct (This ends the robot's suffering)")
    print("\n(5) Enter target destination")
    choice = int(input(''))
    data = struct.pack('!H', choice)
    port.write(data)
 
    
    if choice == 1:
        print("\nEnter degrees to turn from current position, Bad Boi: ")
        print("\nValue must be in range of |10| <= R <= |180|")
        degrees = int(input(''))
        while (degrees < 10 and degrees > -10):
            print("\nValue must be in range of |10| <= R <= |180|")
            degrees = int(input(''))

        while degrees < -180 or degrees > 180:
            print("\nValue must be in range of |10| <= R <= |180|")
            degrees = int(input(''))
            
        dat_stuff = struct.pack('!h', degrees)
        port.write(dat_stuff)
        
    elif choice == 2:
        print("Enter desired drive distance, Bad Boi: ")
        print("\nValue must be in range of 2 <= R <= 512")
        distance = int(input(''))
        while (distance < 2 or distance > 512):
            print("\nValue must be in range of 2 <= R <= 512")
            distance = int(input(''))
     
        dis_stuff = struct.pack("!h", distance)
        port.write(dis_stuff)
        
    elif choice == 3:
        print("Toggling Obstacle Avoidance ")
        OA_toggle = OA_toggle ^ 1

        if OA_toggle == 1:
            print("Obstacle Avoidance Turned Off ")
        else:
            print("Obstacle Avoidance Turned On ")
            
    elif choice == 4:
        print"\nB\nW\nA"
        sleep(0.2)
        print"A\nA\nA\nA"
        sleep(0.2)
        for i in range(15):
            print'h'
            sleep(0.2)
        print"!\n!\n!\n!"
        sleep(2)
        print"\n\nJust Kidding!"
    elif choice == 5:
        print('\nSelect a destination in x,y Cartesian format \n')
        dx,dy = raw_input("ENTER x y coordinates separated by space\n").split()
        dest_x = int(dx)
        dest_y = int(dy)
        print'\nNow moving to desired location [',dest_x,dest_y,']'
        distance = ((dest_x*dest_x) + (dest_y*dest_y))
        a_tan = math.atan2(dest_y,dest_x)
        degrees = math.degrees(a_tan)
        distance = math.sqrt(distance) + 1
        distance = int(distance)
        dis_stuff = struct.pack("!hh",int(degrees),distance)
        port.write(dis_stuff)
	sleep(2)
        
    else:
        print("\n You Goof! Enter a Valid Number(1-5)")
        
    rad += math.radians(degrees)
    ## convert input angle to radians
    fwd_x += (distance*(math.cos(rad)))
    fwd_y += (distance*(math.sin(rad)))
    print'\n\rPose is now [',int(fwd_x), int(fwd_y), rad,']'
    degrees = 0
    distance = 0
    a_tan =0
            

		
