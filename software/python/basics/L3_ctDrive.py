# Level 3 program for driving SCUTTLE and handling other tasks in parallel

# IMPORT EXTERNAL ITEMS
import time
import numpy as np # for handling matrices
import math as math
import sys

# IMPORT INTERNAL ITEMS
import L1_motors as m
import L2_speed_control as sc # closed loop control. Import speed_control for open-loop
import L2_inverse_kinematics as inv #calculates wheel parameters from chassis
import L2_kinematics as kin    # calculates chassis parameters from wheels
import L2_color_target as ct # for driving with computer vision tracking
import L1_encoder as enc # for accessing encoders directly
import L1_adc as adc # for accessing adc sensor onboard
import L2_heading as hd # for accessing compass heading
import L2_colorTrackingSC as ctsc # for accessing new speed control

start_time = time.time()

# initialize variables for control system
t0 = 0
t1 = 1
e00 = 0
e0 = 0
e1 = 0
dt = 0
de_dt = np.zeros(2) # initialize the de_dt
count = 0
# initialize variables for color tracking
#color_range = np.array([[0, 103, 137], [43, 178, 213]]) #defines the color range
color_range = np.array([[24, 67, 52], [52, 255, 255]]) #defines the color range
thetaOffset = 0

while(1):
    count += 1
    # THIS BLOCK IS FOR DRIVING BY COLOR TRACKING
    colorTarget = ct.colorTarget(color_range) # use color tracking to generate targets
    x = colorTarget[0] # x = 0, y = 1, radius = 2
    if x != None:
    	thetaOffset = ct.horizLoc(x) #grabs the angle of the target in degrees
    myThetaDot = thetaOffset * 3.14/180 *2 # achieve centering in 0.5 seconds
    myXDot = .30
    #print(myThetaDot)cd
    A = np.array([ myXDot, myThetaDot ])
    pdTargets = inv.convert(A) # convert [xd, td] to [pdl, pdr] TODO: how is pdTargets calculating pdTargets?

    print("X: ",colorTarget[0],"\t","Y: ",colorTarget[1],"\t","Radius: ",colorTarget[2],"\t")

    kin.getPdCurrent() # capture latest phi dots & update global var
    pdCurrents = kin.pdCurrents # assign the global variable value to a local var

        # THIS BLOCK UPDATES VARS FOR CONTROL SYSTEM
    t0 = t1  # assign t0
    t1 = time.time() # generate current time
    dt = t1 - t0 # calculate dt
    e00 = e0 # assign previous previous error
    e0 = e1  # assign previous error
    e1 = pdCurrents - pdTargets # calculate the latest error
    de_dt = (e1 - e0) / dt # calculate derivative of error

    print(time.time()-start_time, "loop")

    if 5 <= colorTarget[2] <= 45:
        print("Driving...")
        ctsc.driveClosedLoop(pdTargets, pdCurrents, de_dt)  # call the control system

    elif colorTarget[2] > 45:
        print("The SCUTTLE has docked.")
        m.MotorR(0)
        m.MotorL(0)
