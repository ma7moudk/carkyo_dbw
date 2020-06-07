#!/usr/bin/env python3

import time, rospy
import Jetson.GPIO as GPIO
import os
import i2cdev

MCP4725_ADD_ACC = 0x60
MCP4725_ADD_BRAKE = 0x61
dac_acc = i2cdev.I2C(MCP4725_ADD_ACC, 8)


# Time Variables
mode_time  = time.time()

rospy.init_node("carkyo_test")


def accelerate(accVal):
    global accSwitch,  accPins,last_acc 
    last_acc = accVal       ### Just For Debuging
    dacValue = accVal # Range (0 to 4095)
    H = int(dacValue / 16)
    L = int((dacValue - (16*H)) * 16)
    data_to_send = bytes([64,H,L])
    dac_acc.write(data_to_send)
    #time.sleep(0.1)


for i in range(4000):
    accelerate(i)
    time.sleep(0.1)

