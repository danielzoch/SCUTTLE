# IMPORT EXTERNAL ITEMS
import time
import numpy as np # for handling matrices
import math
import sys

# IMPORT INTERNAL ITEMS
import L1_motors as mtr
import L2_speed_control as sc # closed loop control. Import speed_control for open-loop
import L2_inverse_kinematics as inv #calculates wheel parameters from chassis
import L2_kinematics as kin    # calculates chassis parameters from wheels
import L2_color_target as ct # for driving with computer vision tracking
import L1_encoder as enc # for accessing encoders directly
import L1_adc as adc # for accessing adc sensor onboard
import L2_heading as hd # for accessing compass heading
import Docking_Functions as df # for accessing front and center code



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
color_range = np.array([[24, 67, 52], [52, 255, 255]]) #defines the color range
thetaOffset = 0
# initialize driving mode
mode = 0
axes = 0
axesScaled = 0
head = 0

colorTarget = ct.colorTarget(color_range) # use color tracking to generate targets

while (colorTarget[0] == None or colorTarget[1] == None or colorTarget[0] < 80 or colorTarget[1] < 85 or colorTarget[1] > 126 or colorTarget[2] < 5.8 or colorTarget[2] > 36):
    #Turn in circle
    colorTarget = ct.colorTarget(color_range)
    print("X: ",colorTarget[0],"\t","Y: ",colorTarget[1],"\t","Radius: ",colorTarget[2],"\t")
    print("Turning!")

    #fs.DriveDP(XDot, ThetaDot, time (sec))
    # fs.DriveDP(0,0.2,0.25)
    mtr.MotorL(-0.55)
    mtr.MotorR(0.55)
mtr.MotorL(0)
mtr.MotorR(0)
print("       Found Station")
colorTarget = ct.colorTarget(color_range)
print("       X: ",colorTarget[0])

#df.closeThetaOffset() # close theta offset

print("Now closing Theta Offset")
color_range = np.array([[0, 103, 137], [43, 178, 213]]) #defines the color range
colorTarget = ct.colorTarget(color_range)
# close Theta offset --
#
# !!!!!!!!!!!!! HAD TO ADD IN Y LIMITS TO THE BELOW WHILE STATEMENT !!!!!!!!!!!!!! - daniel
#
while (colorTarget[0] < 110 or colorTarget[0] > 130 or colorTarget[1] < 85 or colorTarget[1] > 126):
    if (colorTarget[0] < 110):
        m.MotorL(-0.50)
        m.MotorR(0.50)
        #fs.DriveDP(0,0.2,0.1)
    elif (colorTarget[0] > 130):
        m.MotorL(0.50)
        m.MotorR(-0.50)
        #fs.DriveDP(0,-0.2,0.1)
    colorTarget = ct.colorTarget(color_range)
    #print("X: ",colorTarget[0],"\t","Y: ",colorTarget[1],"\t","Radius: ",colorTarget[2],"\t")

m.MotorL(0)
m.MotorR(0)
print("       Facing target :)")
colorTarget = ct.colorTarget(color_range)
print("       X: ",colorTarget[0])


#df.GetInFront() #navigate to the target

# while(1):
#     count += 1
#     # THIS BLOCK IS FOR DRIVING BY COLOR TRACKING
#     colorTarget = ct.colorTarget(color_range) # use color tracking to generate targets
#     x = colorTarget[0] # x = 0, y = 1, radius = 2
#     if x != None:
#     	thetaOffset = ct.horizLoc(x) #grabs the angle of the target in degrees
#     myThetaDot = thetaOffset * 3.14/180 *2 # achieve centering in 0.5 seconds
#     myXDot = .35
#     #print(myThetaDot)cd
#     A = np.array([ myXDot, myThetaDot ])
#     pdTargets = inv.convert(A) # convert [xd, td] to [pdl, pdr] TODO: how is pdTargets calculating pdTargets?
#
#     print("X: ",colorTarget[0],"\t","Y: ",colorTarget[1],"\t","Radius: ",colorTarget[2],"\t")
#
#     kin.getPdCurrent() # capture latest phi dots & update global var
#     pdCurrents = kin.pdCurrents # assign the global variable value to a local var
#
#         # THIS BLOCK UPDATES VARS FOR CONTROL SYSTEM
#     t0 = t1  # assign t0
#     t1 = time.time() # generate current time
#     dt = t1 - t0 # calculate dt
#     e00 = e0 # assign previous previous error
#     e0 = e1  # assign previous error
#     e1 = pdCurrents - pdTargets # calculate the latest error
#     de_dt = (e1 - e0) / dt # calculate derivative of error
#
#     if 5 <= colorTarget[2] <= 45 and 95 <= colorTarget[0] <= 175:
#         print("Driving...")
#         sc.driveClosedLoop(pdTargets, pdCurrents, de_dt)  # call the control system
#
#     elif colorTarget[2] > 45:
#         print("The SCUTTLE has docked.")
#         sys.exit()
