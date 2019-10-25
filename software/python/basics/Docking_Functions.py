# This code contains functions for getting the scuttle in front of the station when it is offset by a certain amount
# Created by NexTec Team 2019

# Import Internal Programs
import L1_adc as adc
import L1_motors as m
import L2_heading as head
import L2_log as log
import L2_speed_control as sc
import L2_inverse_kinematics as inv
import L2_kinematics as kin    # calculates chassis parameters from wheels
import L2_color_target as ct # for color tracking business
import L2_colorTrackingSC as ctsc # for accessing color tracking speed control

# Import External programs
import numpy as np
import time
import sys
import rcpy
import rcpy.motor as motor
import math
import threading # only used for threading functions

color_range = np.array([[24, 67, 52], [52, 255, 255]]) #defines the color range

rcpy.set_state(rcpy.RUNNING) # initialize the rcpy library

def GetTH(SH):
    # This function takes input of the Station's Heading and returns the target heading. Input and output are in 360 notation.
    # Target heading is defined as the heading SCUTTLE should have when docked with the station,
    # this is effectively the "opposite" of the station's heading.
    if (SH >= 180):
        TH = SH-180
    elif (SH < 180):
        TH = SH+180
    return TH
    
def DriveDP(xdot,tdot,st):
    #NOTE: +tdot is ccw, -tdot is cw
    #st = sleep time (how long to run)
    myVelocities = np.array([xdot, tdot]) #input your first pair
    myPhiDots = inv.convert(myVelocities)
    # INITIALIZE VARIABLES FOR CONTROL SYSTEM
    t0 = 0  # time sample
    t1 = 1  # time sample
    e00 = 0 # error sample
    e0 = 0  # error sample
    e1 = 0  # error sample
    dt = 0  # delta in time
    de_dt = np.zeros(2) # initialize the de_dt
    timestamp = time.time()
    endtime = timestamp+st
    while(time.time() < endtime):
        # THIS CODE IS FOR OPEN AND CLOSED LOOP control
        pdTargets = np.array([myPhiDots[0], myPhiDots[1]]) # Input requested PhiDots (radians/s)
        kin.getPdCurrent() # capture latest phi dots & update global var
        pdCurrents = kin.pdCurrents # assign the global variable value to a local var

        # THIS BLOCK UPDATES VARIABLES FOR THE DERIVATIVE CONTROL
        t0 = t1  # assign t0
        t1 = time.time() # generate current time
        dt = t1 - t0 # calculate dt
        e00 = e0 # assign previous previous error
        e0 = e1  # assign previous error
        e1 = pdCurrents - pdTargets # calculate the latest error
        de_dt = (e1 - e0) / dt # calculate derivative of error

        # CALLS THE CONTROL SYSTEM TO ACTION
        sc.driveClosedLoop(pdTargets, pdCurrents, de_dt)  # call on closed loop
        time.sleep(0.05) # this time controls the frequency of the controller

    m.MotorL(0)
    m.MotorR(0)

def GetH360():
    #get scaled values from L2 heading
    axes = head.getXY() # call xy function
    axesScaled = head.scale(axes) # perform scale function

    #get heading from L2 heading
    h = head.getHeading(axesScaled) # compute the heading
    headingDegrees = round(h*180/np.pi,2)

    # CONVERT 180 TO 360
    # so from N=0, E=-90, S=-180/180, W=90
    # we want N rotating CW to W to be 0-360
    H180 = headingDegrees #making life easier
    H360 = 900 #just declaring my conversion var
    if (H180 < 0 ):
        #we are between NES
        H360 = (H180 * -1)
    elif (H180 > 0):
        #we are between SWN
        H360 = (360-H180)
    return H360

def GetInFront(SH):
    Offset = 15 #Offset for DT in cm
    colorTarget = ct.colorTarget(color_range)
    # Define variables (abstract)
    #SH must be passed in as an argument, mandatory
    DT = (739.78*(colorTarget[2]**-0.962)+Offset)/100 #distance to target, (in meters), straigh line distance to station from current position, based on radius of target from camera, currently hardcoded, will be updated to pull dist from webcam
    CH = GetH360() #current heading, this is live, scuttle's compass heading in 360 notation

    #Everything in the section below is calculated based on the inputs from scuttle's sensors as defined above
    HT = 0 #heading target, the heading scuttle should have when it is directly in front of the station to dock
    HL = 0 #heading left, the heading scuttle should face if it needs to travel perpendicularly left from the station POV
    HR = 0 #heading right, the heading scuttle should face if it needs to travel perpendicularly right from the station POV
    TP = 0 #theta path, the angle between where scuttle currently is, and the direct front line from the station pov
    DP = 0 #drive path, the distance scuttle needs to travel perpendicularly to get in front of station (in meters)
    Dir = 5 #1 = left, 0 = right, will always be 0 or 1, corresponds to which side of station scuttle is on, from station pov

    temp = 0 #temp var used for misc tasks

    # calculate target heading, heading left, and heading right based on station orientation
    # CAN COMMENT THIS SECTION OUT TO MANUALLY INPUT HT,HL,HR
    if (SH >= 180):
        HT = SH-180
        HL = SH-90
        HR = SH+90
        if (HR > 360):
            temp = HR
            HR = temp - 360
    elif (SH < 180):
        HT = SH+180
        HL = SH-90
        HR = SH+90
        if (HL < 0):
            temp = HL
            HL = 360 + temp
    #DEBUG
    print("HT:",HT)
    print("HL:",HL)
    print("HR:",HR)
    # COMMENT OUT ABOVE SECTION FOR MANUAL INPUT

    # Now, based on current heading and distance to station, determine drive path
    # first need to find our theta path based on current heading and station heading
    #DEBUG
    print("CH:",CH)

    if (CH >= 180):
        temp = CH-180
    elif (CH < 180):
        temp = CH+180
    #DEBUG
    print(temp)

    if (temp > SH):
        TP = temp-SH
    elif (SH > temp):
        TP = SH-temp

    if (TP > 90):
        TP = 360-TP
    #DEBUG
    print(TP)

    if (temp < 90 and SH > 270):
        Dir = 0
        #DEBUG
        print(Dir)

    elif (SH < 90 and temp > 270):
        Dir = 1
    elif (temp > SH):
        Dir = 0
    elif (temp < SH):
        Dir = 1
    #DEBUG
    print("Theta path:",TP)

    #NOTE: TP should never be greater than 45 degrees, according to spec, so maybe report error?

    # if (TP > 45 ):
    #     print("ERROR, TP TOO BIG")
    #     sys.exit()
    # face either HR or HL
    if (Dir == 0):
        #scuttle is to right from station pov
        #face HL
        #while heading not == HL with 5 degree error, rotate scuttle clockwise
        while ( (CH > HL+4) or (CH < HL-4) ):
            DriveDP(0,-0.2,0.5)
            CH = GetH360()
            #DEBUG
            print("Dir0 While-CH:",CH)
        m.MotorL(0)
        m.MotorR(0)
    elif (Dir == 1):
        #scuttle is to left from station pov
        #face HR
        # while heading not == HR with 5 degree error, rotate scuttle
        while ( (CH > HR+4) or (CH < HR-4) ):
            DriveDP(0,0.2,0.5)
            CH = GetH360()
            #DEBUG
            print("Dir1 While-CH:",CH)
        m.MotorL(0)
        m.MotorR(0)
    #now needs to drive a specified distance dp
    TPR = math.radians(TP) 
    #DEBUG
    print("TPR:",TPR)
    DP = DT*(math.sin(TPR)) #this is the length in meters, scuttle needs to drive to get in front of station
    xdot = 0.1

    #we are now facing either HR or HL, now drive DP
    st = DP/xdot #how long we want to drive with closed loop, in seconds
    DriveDP(xdot,0,st)
    #DEBUG
    print("st:",st)

    #we are now in front of station, turn to face it
    #need to decide which way to turn, depends on Dir
    if (Dir == 0): #turn scuttle ccw
        while ( (CH < HT-4) or (CH > HT+4) ): #while ch not within 5 degrees of sh
            DriveDP(0,0.2,0.5)
            CH = GetH360()
    elif (Dir == 1): #turn scuttle cw
        while( (CH < HT-4) or (CH > HT+4) ): #while ch not within 5 degrees of sh
            DriveDP(0,-0.2,0.5)
            CH = GetH360()

    #time.sleep(0.25) # delay 0.25sec



# BEGIN: machine vision functions!

def FindStation():
    error = 0
    timestamp = time.time()
    #PWM = ScalePWM()
    colorTarget = ct.colorTarget(color_range)
    
    xdot = 0 #do not change this for just turning in a circle, leave as 0
    tdot = 0.25
    #NOTE: +tdot is ccw, -tdot is cw
    #st = sleep time (how long to run)
    myVelocities = np.array([xdot, tdot]) #input your first pair
    myPhiDots = inv.convert(myVelocities)
    # INITIALIZE VARIABLES FOR CONTROL SYSTEM
    t0 = 0  # time sample
    t1 = 1  # time sample
    e00 = 0 # error sample
    e0 = 0  # error sample
    e1 = 0  # error sample
    dt = 0  # delta in time
    de_dt = np.zeros(2) # initialize the de_dt
    
    while (colorTarget[0] == None or colorTarget[1] == None or colorTarget[0] < 110 or colorTarget[1] < 85 or colorTarget[1] > 126 or colorTarget[2] < 5 or colorTarget[2] > 36):
        #Turn in circle
        #CLOSED LOOP DRIVE START
        # THIS CODE IS FOR OPEN AND CLOSED LOOP control
        pdTargets = np.array([myPhiDots[0], myPhiDots[1]]) # Input requested PhiDots (radians/s)
        kin.getPdCurrent() # capture latest phi dots & update global var
        pdCurrents = kin.pdCurrents # assign the global variable value to a local var
        # THIS BLOCK UPDATES VARIABLES FOR THE DERIVATIVE CONTROL
        t0 = t1  # assign t0
        t1 = time.time() # generate current time
        dt = t1 - t0 # calculate dt
        e00 = e0 # assign previous previous error
        e0 = e1  # assign previous error
        e1 = pdCurrents - pdTargets # calculate the latest error
        de_dt = (e1 - e0) / dt # calculate derivative of error

        # CALLS THE CONTROL SYSTEM TO ACTION
        sc.driveClosedLoop(pdTargets, pdCurrents, de_dt)  # call on closed loop
        #CLOSED LOOP DRIVE END
        #m.MotorL(-PWM)
        #m.MotorR(PWM)
        
        colorTarget = ct.colorTarget(color_range)
        
        print("X: ",colorTarget[0],"\t","Y: ",colorTarget[1],"\t","Radius: ",colorTarget[2],"\t")
        print("       Turning!")
        if (time.time() > timestamp + 40):
            print("Time expired")
            error = 1 #Target not found within time limit
            break
    m.MotorL(0)
    m.MotorR(0)
    print("       Found target...")
    print("       X: ",colorTarget[0])
    return(error)


def CloseThetaOffset():
    colorTarget = ct.colorTarget(color_range)
    xdot = 0 #do not change this for just turning in a circle, leave as 0
    tdot = 0.25
    #NOTE: +tdot is ccw, -tdot is cw
    # INITIALIZE VARIABLES FOR CONTROL SYSTEM
    t0 = 0  # time sample
    t1 = 1  # time sample
    e00 = 0 # error sample
    e0 = 0  # error sample
    e1 = 0  # error sample
    dt = 0  # delta in time
    de_dt = np.zeros(2) # initialize the de_dt
    
    while (colorTarget[0] < 122.5 or colorTarget[0] > 132.5):
        if (colorTarget[0] < 122.5):
            print("      Turn Left")
            # CALLS THE CONTROL SYSTEM TO ACTION
            tdot = 0.15
            myVelocities = np.array([xdot, tdot]) #input your first pair
            myPhiDots = inv.convert(myVelocities)
            pdTargets = np.array([myPhiDots[0], myPhiDots[1]]) # Input requested PhiDots (radians/s)
            kin.getPdCurrent() # capture latest phi dots & update global var
            pdCurrents = kin.pdCurrents # assign the global variable value to a local var
            
            # THIS BLOCK UPDATES VARIABLES FOR THE DERIVATIVE CONTROL
            t0 = t1  # assign t0
            t1 = time.time() # generate current time
            dt = t1 - t0 # calculate dt
            e00 = e0 # assign previous previous error
            e0 = e1  # assign previous error
            e1 = pdCurrents - pdTargets # calculate the latest error
            de_dt = (e1 - e0) / dt # calculate derivative of error
            
            sc.driveClosedLoop(pdTargets, pdCurrents, de_dt)  # call on closed loop
            
        elif (colorTarget[0] > 132.5):
            print("      Turn Right")
            # CALLS THE CONTROL SYSTEM TO ACTION
            tdot = -0.15
            myVelocities = np.array([xdot, tdot]) #input your first pair
            myPhiDots = inv.convert(myVelocities)
            pdTargets = np.array([myPhiDots[0], myPhiDots[1]]) # Input requested PhiDots (radians/s)
            kin.getPdCurrent() # capture latest phi dots & update global var
            pdCurrents = kin.pdCurrents # assign the global variable value to a local var
            
            # THIS BLOCK UPDATES VARIABLES FOR THE DERIVATIVE CONTROL
            t0 = t1  # assign t0
            t1 = time.time() # generate current time
            dt = t1 - t0 # calculate dt
            e00 = e0 # assign previous previous error
            e0 = e1  # assign previous error
            e1 = pdCurrents - pdTargets # calculate the latest error
            de_dt = (e1 - e0) / dt # calculate derivative of error
            
            sc.driveClosedLoop(pdTargets, pdCurrents, de_dt)  # call on closed loop
            
        colorTarget = ct.colorTarget(color_range)
        print("       X: ",colorTarget[0])
    
    m.MotorL(0)
    m.MotorR(0)
    colorTarget = ct.colorTarget(color_range)
    print("       Closed Theta Offset")
    print("       X: ",colorTarget[0])


def straightdockvpc():
    colorTarget = ct.colorTarget(color_range)
    xdot = 0.2 #do not change this for just turning in a circle, leave as 0
    tdot = 0 #this updates below, for turning left or right but is initialized to zero
    #NOTE: +tdot is ccw, -tdot is cw
    #st = sleep time (how long to run)
    myVelocities = np.array([xdot, tdot]) #input your first pair
    myPhiDots = inv.convert(myVelocities)
    # INITIALIZE VARIABLES FOR CONTROL SYSTEM
    t0 = 0  # time sample
    t1 = 1  # time sample
    e00 = 0 # error sample
    e0 = 0  # error sample
    e1 = 0  # error sample
    dt = 0  # delta in time
    de_dt = np.zeros(2) # initialize the de_dt
    
    while 5 <= colorTarget[2] <= 45:
        #CLOSED LOOP DRIVE START
        myVelocities = np.array([xdot, tdot]) #input your first pair
        myPhiDots = inv.convert(myVelocities)
        # THIS CODE IS FOR OPEN AND CLOSED LOOP control
        pdTargets = np.array([myPhiDots[0], myPhiDots[1]]) # Input requested PhiDots (radians/s)
        kin.getPdCurrent() # capture latest phi dots & update global var
        pdCurrents = kin.pdCurrents # assign the global variable value to a local var
        # THIS BLOCK UPDATES VARIABLES FOR THE DERIVATIVE CONTROL
        t0 = t1  # assign t0
        t1 = time.time() # generate current time
        dt = t1 - t0 # calculate dt
        e00 = e0 # assign previous previous error
        e0 = e1  # assign previous error
        e1 = pdCurrents - pdTargets # calculate the latest error
        de_dt = (e1 - e0) / dt # calculate derivative of error
        # CALLS THE CONTROL SYSTEM TO ACTION
        sc.driveClosedLoop(pdTargets, pdCurrents, de_dt)  # call on closed loop
        #CLOSED LOOP DRIVE END
        colorTarget = ct.colorTarget(color_range)
        if ( (colorTarget[0] < 87.5 or colorTarget[0] > 167.5) and (colorTarget[2] < 36) ):
            CloseThetaOffset()
    m.MotorL(0)
    m.MotorR(0)
    
    
# this function is for approaching the ball head-on
def driveStraight():
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
    colorTarget = ct.colorTarget(color_range) #grab an x and radius value from color target
    thetaOffset = 0
    while(5 <= colorTarget[2] <= 45): #perform the loop until the ball is close 
        count += 1
        # THIS BLOCK IS FOR DRIVING BY COLOR TRACKING
        colorTarget = ct.colorTarget(color_range) # use color tracking to generate targets
        x = colorTarget[0] # x = 0, y = 1, radius = 2
        if x != None:
        	thetaOffset = ct.horizLoc(x) #grabs the angle of the target in degrees
        myScalar = .25
        myThetaDot = thetaOffset * 3.14/180 * 2 * myScalar # achieve centering in 0.5 seconds
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
        
        sc.driveClosedLoop(pdTargets, pdCurrents, de_dt)  # call the control system
        
    print("The SCUTTLE has docked.")
    m.MotorR(0)
    m.MotorL(0)
            
def debugradius():
    colorTarget = ct.colorTarget(color_range) # use color tracking to generate targets
    while(1):
        colorTarget = ct.colorTarget(color_range)
        print(colorTarget[0])
        print(colorTarget[2])
        