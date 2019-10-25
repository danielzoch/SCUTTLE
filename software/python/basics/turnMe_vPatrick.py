# IMPORT EXTERNAL ITEMS
import time
import numpy as np # for handling matrices
import math
import sys

# IMPORT INTERNAL ITEMS
import L1_motors as mtr
import L2_color_target as ct # for driving with computer vision tracking


#color_range = np.array([[0, 103, 137], [43, 178, 213]]) #defines the color range
color_range = np.array([[24, 67, 52], [52, 255, 255]]) #defines the color range

#colorTarget = ct.colorTarget(color_range) # use color tracking to generate targets
#x = colorTarget[0] # x = 0, y = 1, radius = 2 TODO: plot in nodered


# while(1):
#     colorTarget = ct.colorTarget(color_range)
#     print("X: ",colorTarget[0],"\t","Y: ",colorTarget[1],"\t","Radius: ",colorTarget[2],"\t")


colorTarget = ct.colorTarget(color_range)
while (colorTarget[0] == None or colorTarget[1] == None or colorTarget[0] < 80 or colorTarget[1] < 85 or colorTarget[1] > 126 or colorTarget[2] < 5 or colorTarget[2] > 36):
    #Turn in circle
    mtr.MotorL(-0.55)
    mtr.MotorR(0.55)
    colorTarget = ct.colorTarget(color_range)
    print("X: ",colorTarget[0],"\t","Y: ",colorTarget[1],"\t","Radius: ",colorTarget[2],"\t")
    print("       Turning!")
mtr.MotorL(0)
mtr.MotorR(0)
print("       Found target...")

#Continue by CLOSING thetaOffset
colorTarget = ct.colorTarget(color_range)
while (colorTarget[0] < 110 or colorTarget[0] > 130):
    if (colorTarget[0] < 110):
        print("       Turn Left")
        mtr.MotorL(-0.55)
        mtr.MotorR(0.55)
        #fs.DriveDP(0,0.2,0.1)
    elif (colorTarget[0] > 130):
        print("       Turn Right")
        mtr.MotorL(0.55)
        mtr.MotorR(-0.55)
        #fs.DriveDP(0,-0.2,0.1)
    colorTarget = ct.colorTarget(color_range)
    print("X: ",colorTarget[0],"\t","Y: ",colorTarget[1],"\t","Radius: ",colorTarget[2],"\t")
mtr.MotorL(0)
mtr.MotorR(0)
print("       Closed Theta Offset")
colorTarget = ct.colorTarget(color_range)
print("       X: ",colorTarget[0])

