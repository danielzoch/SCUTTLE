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
#import L2_heading as hd # for accessing compass heading
import L2_360Heading as hd

#DEFINE headingDegrees
SCUTTLE_Hd = hd.get360Heading() #call compass heading function
ChargingStation_Hd = 327





FacingStation_Hd = ChargingStation_Hd - 180
90_RightofStation_Hd = FacingStation_Hd - 90
90_LeftofStation_Hd = FacingStation_Hd + 90
# North_Hd =
# East_Hd =
# South_Hd =
# West_Hd =




color_range = np.array([[0, 103, 137], [43, 178, 213]]) #defines the color range

colorTarget = ct.colorTarget(color_range) # use color tracking to generate targets
#x = colorTarget[0] # x = 0, y = 1, radius = 2 TODO: plot in nodered

###Start with "Turn in circle code
while (colorTarget[0] == None or colorTarget[1] == None or colorTarget[0] < 60 or colorTarget[1] < 85 or colorTarget[1] > 126 or colorTarget[2] < 8 or colorTarget[2] > 36):
    #Turn in circle
    colorTarget = ct.colorTarget(color_range)
    #print("X: ",colorTarget[0],"\t","Y: ",colorTarget[1],"\t","Radius: ",colorTarget[2],"\t")
    print("Turning!")
    mtr.MotorL(-0.55)
    mtr.MotorR(0.55)

print("Facing target")
mtr.MotorL(0)
mtr.MotorR(0)


# #Continue by CLOSING thetaOffset
# count = 0.4
# while (colorTarget[0] < 120 or colorTarget[0] > 130):
#     colorTarget = ct.colorTarget(color_range)
#     if (colorTarget[0] < 120):
#         mtr.MotorL(0)
#         mtr.MotorR(0.50)
#         time.sleep(count)
#         mtr.MotorL(0)
#         mtr.MotorR(0)
#     elif (colorTarget[0] > 130):
#         mtr.MotorL(0.50)
#         mtr.MotorR(0)
#         time.sleep(count)
#         mtr.MotorL(0)
#         mtr.MotorR(0)
#     else:
#         mtr.MotorL(0)
#         mtr.MotorR(0)
#     count = count - 0.05


#Next, check if its already aligned and facing the station
if (SCUTTLE_Hd != FacingStation_Hd):#Give or take a few degrees...may need changing
    #Next, check which side of the station its on based on heading
    if (SCUTTLE_Hd > FacingStation_Hd and SCUTTLE_Hd < 90_RightofStation_Hd):#"if its on the left...."
        #Determine Theta of right triangle
        Our_Theta = 90_RightofStation_Hd - SCUTTLE_Hd
        #Determine Height of right triangle based on # pixel radius size
        H = colorTarget[2] * #function based on tested values
        #Now calculate Length of right triangle
        L = H*math.cos(Our_Theta)
        #Set Heading for turning "Theta"
        TurnToTheta = 90_RightofStation_Hd
        #Set motor speeds ahead of time
        ML = -0.55
        MR = 0.55


    elif (SCUTTLE_Hd < FacingStation_Hd and SCUTTLE_Hd > 90_LeftofStation_Hd):#"if its on the right..."

        #Determine Theta of right triangle
        Our_Theta = SCUTTLE_Hd - 90_LeftofStation_Hd
        #Determine Height of right triangle based on # pixel radius size
        H = colorTarget[2] * #function based on tested values
        #Now calculate Length of right triangle
        L = H*math.cos(Our_Theta)
        #Set Heading for turning "Theta"
        TurnToTheta = 90_LeftofStation_Hd
        #Set motor speeds ahead of time
        ML = 0.55
        MR = -0.55



    #Now, turn until the SCUTTLE faces 90 degrees right of station
    count = 0.4
    while (SCUTTLE_Hd != TurnToTheta):#Give or take a few degrees... may need changing
        SCUTTLE_Hd = #Get SCUTTLE_Hd here...
        if (SCUTTLE_Hd > TurnToTheta):
            mtr.MotorL(0)
            mtr.MotorR(0.50)
            time.sleep(count)
            mtr.MotorL(0)
            mtr.MotorR(0)
        elif (SCUTTLE_Hd < TurnToTheta):
            mtr.MotorL(0.50)
            mtr.MotorR(0)
            time.sleep(count)
            mtr.MotorL(0)
            mtr.MotorR(0)
        else:
            mtr.MotorL(0)
            mtr.MotorR(0)
        count = count - 0.05



    #Now that it is facing 90 degrees from staion, move length L
    ###NOT SURE HOW TO DO THIS!
    #Now after driving Length L, turn towards station
    ### I have two options, 1. Use "turn in circle code again and waste time or 2. Repeat the same BS i used previously"
    while (colorTarget[0] == None or colorTarget[1] == None or colorTarget[0] < 60 or colorTarget[0] > 190 or colorTarget[1] < 85 or colorTarget[1] > 126 or colorTarget[2] < 8 or colorTarget[2] > 36):
        #Turn in circle
        colorTarget = ct.colorTarget(color_range)
        #print("X: ",colorTarget[0],"\t","Y: ",colorTarget[1],"\t","Radius: ",colorTarget[2],"\t")
        print("Turning!")
        mtr.MotorL(ML)
        mtr.MotorR(MR)

    print("Facing target :)")
    mtr.MotorL(0)
    mtr.MotorR(0)

    #Continue to close Theta Offset for better accuracy
    count = 0.4
    while (colorTarget[0] < 120 or colorTarget[0] > 130):
        colorTarget = ct.colorTarget(color_range)
        if (colorTarget[0] < 120):
            mtr.MotorL(0)
            mtr.MotorR(0.50)
            time.sleep(count)
            mtr.MotorL(0)
            mtr.MotorR(0)
        elif (colorTarget[0] > 130):
            mtr.MotorL(0.50)
            mtr.MotorR(0)
            time.sleep(count)
            mtr.MotorL(0)
            mtr.MotorR(0)
        else:
            mtr.MotorL(0)
            mtr.MotorR(0)
        count = count - 0.05

######IMPLEMENT DANIEL'S CODE HERE
#DONE#
