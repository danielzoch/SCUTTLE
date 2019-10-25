# Draining and monitoring battery levels of SCUT

import rcpy
import rcpy.motor as motor
import time # only necessary if running this program as a loop
import numpy as np
from rcpy._adc import *


motor_r = 2 	# Right Motor
motor_l = 1 	# Left Motor
rcpy.set_state(rcpy.RUNNING)

#channel refers to left(0) or right(1)
def MotorL(speed):
    motor.set(motor_l, speed)

def MotorR(speed):
    motor.set(motor_r, speed)

def getDcJack(): # return the voltage measured at the barrel plug
    voltage = round(get_dc_jack_voltage(), 2)
    return (voltage)

# # Uncomment this section to run this program as a standalone loop
v = getDcJack()
print("startv:",v)
while ((rcpy.get_state() != rcpy.EXITING) and (v >= 7)):
    
    if rcpy.get_state() == rcpy.RUNNING:
        v = getDcJack()
        print(v)
        MotorL(-1)  
        MotorR(1)
        time.sleep(30) 
        v = getDcJack()
        print(v)
        MotorL(1)
        MotorR(-1)
        time.sleep(30) 
        v = getDcJack()
        print(v)
