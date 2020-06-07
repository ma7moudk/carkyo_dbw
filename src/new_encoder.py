#!/usr/bin/env python3

# KZOH: Changed TEN_MICRO to HUNDRED_MICRO (and variables)
# KZOH: Changed function names Thread to: TimmedTasks, EncoderHundredMicroTask, etc.
# KZOH: Removed thread and add a call to TimmedTasks at main
# pins 7, 19 -->  L, R
#####$$$$$##### Includes #####$$$$$##### 
import roslib  
import rospy
import string
import math
import numpy as np
import re
import time 
from nav_msgs.msg import Odometry

import time 
import Jetson.GPIO as GPIO
GPIO.setmode(GPIO.BOARD)

rospy.init_node('encoder_node', anonymous=True)
odom_pub = rospy.Publisher('/speed_values', Odometry, queue_size=10)
odomMsg = Odometry()

###Global values for encoders######
NumberOfInterrputsPorts = 2
ticksPerSpinL = 202
ticksPerSpinR = 202
diameter = 0.436
DistancePerCountL = (math.pi * diameter) / ticksPerSpinL
DistancePerCountR = (math.pi * diameter) / ticksPerSpinR

Prev = [0, 0]
consecutivezeros = [0, 0]
consecutiveones = [0, 0]
threshold = [2, 2]
Falling = [0, 0]
Rising = [0, 0]
Transition = [0, 0]

pin=[7, 19]  # left - right
GPIO.setup(pin,GPIO.IN,pull_up_down=GPIO.PUD_UP) #GPIO.PUD_UP / GPIO.PUD_DOWN / GPIO.PUD_OFF

OneMilliesCounter = 0

# Max possible ticks for 7 m in 1 second  = 7 /  DistancePerCount
# Max possible ticks between 2 time stamps(1/encoderfrequency s) =  7 / (encoderfrequency * DistancePerCount)

encoderfrequency = 10 #TODO (set it in one place only)
max_speed = 7.0
maxPossibleTicks = int(max_speed / (encoderfrequency* DistancePerCountL))
#For max_speed, Sensor can have maxPossibleTicks ticks between 2 time stamps.
#Time of one tick = 1/(10 * maxPossibleTicks)

# <<<<<<<< HEMA : 16 oct to solve car jumps due to encoder issues
VL = 0.0
VR = 0.0
lastVL = 0.0
lastVR = 0.0
acc_limit  = 1.1
countL = 0
countR = 0
                # max acceptable Diff bet. vl,lastvl OR vr,lastvr  #
                # max noticed decellration was 0.5 m/s per cycle in right sensor which was more reliable than left sensor 
limit = 1
                # max acceptable Diff bet. Vl,Vr OR vl,lastvl OR vr,lastvr  #
                # max limit diff in normal case was 1.054                   #
                # diff (lastvl - vl) @ problem time was 6,8,14 and 16       #
                # diff (lastvr - vr) @ problem time was normal              #
                # so limit must be > 1.02 :: 1.5 for now                    #
                # 14 Apr 2019 was 1.5 changed to solve the wrong feedback in vy .. TODO: monitor

OldTime = 0.0
OldTicksL = 0.0
OldTicksR = 0.0
def HundredMilliesTask():
    global Falling ,OldTime, OldTicksL, OldTicksR, lastVL, lastVR, VL, VR
    global countL, countR
    TicksL = Falling[0]
    TicksR = Falling[1]
    CurrentTime = time.time()
    DiffTime    = (CurrentTime - OldTime)

    lastVL = VL
    lastVR = VR
    
    VL = (TicksL-OldTicksL) * DistancePerCountL / DiffTime
    VR = (TicksR-OldTicksR) * DistancePerCountR / DiffTime

    #TODO check this algorithm and use maxPossibleTicks variable.
    if countL >= 3:
        lastVl = VL
        countL = 0

    if countR >= 3:
        lastVR = VR
        countR = 0
 
    if (VL - lastVL  > acc_limit): # problem happend here whan i tried it, not sure if it is the same for vr !
        VL = lastVL
        countL += 1
    else:
        countL = 0

    if (VR - lastVR  > acc_limit):
        VR = lastVR
        countR += 1
    else:
        countR = 0

    #publish Odometry message
    odomMsg.twist.twist.linear.x = VL
    odomMsg.twist.twist.linear.y = VR
    odomMsg.twist.twist.angular.x = TicksL-OldTicksL
    odomMsg.twist.twist.angular.y = TicksR-OldTicksR
    odomMsg.pose.pose.orientation.x = TicksL
    odomMsg.pose.pose.orientation.y = TicksR
    odomMsg.header.frame_id = 'odom'
    odomMsg.child_frame_id = 'base_footprint'
    odomMsg.header.stamp = rospy.Time.now()
    odom_pub.publish(odomMsg)

    OldTicksL = TicksL
    OldTicksR = TicksR
    OldTime = CurrentTime

Millies = 0
Secs = 0
Minutes = 0
HunderedMicros = 0
def HundredMicroTask():
    global HunderedMicros, Millies, Secs, Minutes
    HunderedMicros += 1
    if HunderedMicros >=10:
        HunderedMicros=0
        Millies += 1
        if Millies % 42 == 0.0: #TODO check why this number is 42 , why HundredMicroTask isn't called every 100MicroSec
            HundredMilliesTask()
        if Millies >=1000:
            Millies=0
            Secs += 1
            if Secs >=60:	
                Secs=0
                Minutes += 1

def EncoderHundredMicroTask():
    global Prev, consecutivezeros, consecutiveones, Falling, Rising, Transition
    for i in range(NumberOfInterrputsPorts):
        if GPIO.input(pin[i])==0:
            consecutivezeros[i]+=1
            consecutiveones[i]=0
            if Prev[i] == 1 and consecutivezeros[i] >= threshold[i]:
                Prev[i] = 0
                Falling[i]+=1
                Transition[i]+=1
        else:
            consecutivezeros[i]=0
            consecutiveones[i]+=1
            if Prev[i] == 0 and consecutiveones[i] >= threshold[i]:
                Prev[i] = 1
                Rising[i]+=1
                Transition[i]+=1

HUNDRED_MICRO = 0.0001 # units in sec
TimingError = 0
CorrectTiming = 0
WaitCounter = 0
min_time = 10.0
max_time = 0.0
def TimmedTasks():
    global min_time, max_time
    global TimingError, CorrectTiming, WaitCounter
    Start = time.time()
    Next = Start
    Next = Next + HUNDRED_MICRO
    print ("Timmes task start")

    while True:
        Now = time.time()
        Start = Now
        if (Now >= Next):	# Exceeds the time
            TimingError+=1
        else:
            CorrectTiming+=1
        while (Now < Next):
            WaitCounter +=1
            Now = time.time()

        EncoderHundredMicroTask()
        HundredMicroTask()
        Next = Next + HUNDRED_MICRO
        End = time.time()
        TimeElapsed = End - Start
        if (TimeElapsed < min_time):
            min_time = TimeElapsed
        if (TimeElapsed > max_time):
            max_time = TimeElapsed
        #print(min_time, max_time)

if __name__ == '__main__':
    TimmedTasks()
