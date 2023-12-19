#!/usr/bin/env python3

import rospy
import numpy as np
import random
from custom_msg_python import blockdata,actimove,actimoveslist


# this script will calibrate the conversion between camera positions and robot positions
pointcount=20
curpointstep=0
lastpoint=[0,0,0]
camdataready=False
robdataready=False
robdata=0
camdata=0
#initialize
    # when the robot is done with its motion
    # ask the camera for an estemate of its position
    # save the robot positon and the in camera robot position


def camera_callback(msg):
    camdataready=True
    camdata=msg
    stepchek()
def robot_callback(msg):
    # if the last value is not 10, (10 means robot is running)
    if(msg[6]<1):
        robdataready=True # send the robot ready mesage
        robdata=msg
        #reqest the camera estemation
        pubcam.publish([msg[0],msg[1],msg[2]])
pointlib=[]
def stepchek():
    if(camdataready and robdataready):
        #add the gathered data to the list pointlib
        camdataready=False
        robdataready=False
        tg=float[6]
        
        [tg[0],tg[1],tg[2]]=robdata[0:3]# take position from robot
        [tg[3],tg[4],tg[5]]=camdata[0:3]# take position from camera
        pointlib.append(tg)
        # generate a new point
        curpointstep+=1
        poi=generatepoint(lastpoint,200)
        #send it to the robot 
        pubrob.publish([poi[0],poi[1],poi[2],0,0,90,0.5,0.5]) # goto the point with a set rotation
    
    if(curpointstep>pointcount):
        rospy.loginfo("finishing up calibration")
        # calulate a fitting relation between the x y and z points
        # save it to a file(static can be used multiple time)
        # only need to calibrate if data is off or setup has moved
        



        
def generatepoint(lp,radius,heightrange):
    maxx=2000
    maxy=1300
    maxz=900


    gp=[random.randrange(0,maxx),random.randrange(0,maxy),random.randrange(0,maxz) ]
    tries=0
    while(np.sqrt(np.power(gp[0]-lp[0],2)+np.power(gp[1]-lp[1],2))<radius or tries>30):
       tries+=1
       gp=[random.randrange(0,maxx),random.randrange(0,maxy),random.randrange(0,maxz) ]
    return gp

subcam=0
subrob=0
pubrob=0
pubcam=0

if __name__ == '__main__':
    rospy.init_node("calibrator")
    subcam=rospy.Subscriber("/calibration/camera_pos",float[3],)
    subrob=rospy.Subscriber("/calibration/robot_pos",float[3],)
    pubcam=rospy.Publisher("/calibration/camera_active",float[3],queue_size=4)

    pubrob=rospy.Publisher("/controll/movetarget",float[9],queue_size=4)
    rospy.loginfo("i have begun")

    rospy.sleep(1)
