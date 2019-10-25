# L2_360Heading.py
# Team Number: 1
# Hardware TM: Hayden Bowen
# Software TM: Cameron Travis
# Date: 9/30/2019
# Code purpose: This code gets values from L2_heading, then sends this information to a nodered gui and logs to a file.

# Import Internal Programs
import L2_heading as head
import L2_log as log

# Import External programs
import numpy as np
import time

# Define Functions
def get360Heading():
    #get scaled values from L2 heading
    axes = head.getXY() # call xy function
    axesScaled = head.scale(axes) # perform scale function
    #print("scaled values:", axesScaled) # print it out

    #log scaled values to file
    #log.uniqueFile(axesScaled[0],"ASX.txt")
    #log.uniqueFile(axesScaled[1],"ASY.txt")

    #get heading from L2 heading
    h = head.getHeading(axesScaled) # compute the heading
    headingDegrees = round(h*180/np.pi,2)
    #print("H180:", headingDegrees)

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
    #print("H360:", H360)

    return(H360)
    #



    #log heading to file
    #log.uniqueFile(headingDegrees,"HD.txt")

    #time.sleep(0.25) # delay 0.25sec
#  # UNCOMMENT LOOP BELOW TO RUN AS STANDALONE PROGRAM
while 1:
    H360 = get360Heading()
    print("H360:", H360)
