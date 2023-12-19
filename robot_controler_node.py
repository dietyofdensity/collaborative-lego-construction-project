#!/usr/bin/env python3.9

import urx
import numpy as np
import rospy
from dryveCode import targetPosition, dryveInit, homing, targetVelocity, profileVelocity, getPosition,setShdn
from custom_msg_python.msg import custom1, custom2

rospy.loginfo("code began")
robid="172.31.1.115"
dryveid="172.31.1.101"
rob=urx.Robot(robid)

currobotconfig=[0,0,0,0,0,0]

curdryveconfig=0


def initializeall():
    global rob
    rospy.loginfo("initializing robot")
    rob = urx.Robot(robid)
    rospy.sleep(0.1)
    rospy.loginfo("seting comunication protocol")
    rob.set_tcp((0, 0, 0.1, 0, 0, 0))
    rospy.sleep(0.1)
    rospy.loginfo("aplying payload")
    rob.set_payload(2, (0, 0, 0.1))
    rospy.sleep(0.1)
    rospy.loginfo("initializing dryve actuator")
    dryveInit()
    rospy.sleep(0.1)
    rospy.loginfo("homing dryve")
    homing()
    rospy.loginfo("fetching pose")
    currobotconfig=rob.get_pose()

    rospy.loginfo(str(currobotconfig))

    
def stopmove():
    rospy.loginfo("stopping")

def seteepos(x,y,z,xr,yr,zr,iacc,ivel):
    # this function sets the end effector to a position in our table space
    rospy.loginfo("setting information")
    edgedistance=400 #40 cm
    ur10zrotoffset=135 
    dryvepart=0
    if(abs(curdryveconfig-x)>edgedistance):
        if(x<curdryveconfig):
            dryvepart=x+edgedistance*0.5
            dryvepart=int(dryvepart)
            x-=dryvepart
        else:
            dryvepart=x-edgedistance*0.5
            dryvepart=int(dryvepart)
            x-=dryvepart


    poseactual=np.matmul(zro(ur10zrotoffset*np.pi/180),np.array([x,y,z]))
    rotactual=[-1.28,1.97,5.82]
    #pose=rob.getl()
    
    #pose[0]=poseactual[0,0]
    #pose[1]=poseactual[0,1]
    #pose[2]=poseactual[0,2]
    #pose[3]=rotactual[0]
    #pose[4]=rotactual[1]
    #pose[5]=rotactual[2]


    rospy.loginfo("the pos aray was "+str([poseactual[0,0],poseactual[0,1],poseactual[0,2]]))


    #np.matmul(zr,xyzrotationmatrix(xr,yr,zr+ur10zrotoffset*np.pi/180))
    # the ur robots base is rotated 45 degrees in relation to thet tabels global system
    #the global system is configured such that
        #the dryves main axies is the x axies eg along the table
        #the y axis is across the table
        #the z is pointed skyward
    
    
    s1=False
    #s1=True
    if(s1):
        print("doing solution 1")
        
    
    
    targetPosition(dryvepart)
    rospy.sleep(0.1)
    rob.movel((poseactual[0,0],poseactual[0,1],poseactual[0,2],rotactual[0],rotactual[1],rotactual[2]),acc=iacc,vel=ivel ,wait=False   )
    rospy.sleep(2)
    

    
    

    
def getrobopos():
       # this function sets the end effector to a position in our table space

    edgedistance=200 #20 cm
    ur10zrotoffset=45+90
    #betwen the robot and the table a from robot to table is 135, 

    zmatrix= zro(ur10zrotoffset*(np.pi/180))
    # this matrix should convert points from the robot to points of the end effector 
    
    np.matmul(zmatrix,xyzrotationmatrix())
    # the ur robots base is rotated 45 degrees in relation to thet tabels global system
    #the global system is configured such that
        #the dryves main axies is the x axies eg along the table
        #the y axis is across the table
        #the z is pointed skyward

        # get end effector matrix of the robot in the robots cordinate system
        # pre multiply it by a -45 degree x rotation matrix 
        # 
    pos=rob.get_pos()
    [a,b,y]=rob.get_orientation()
    eemat=m3x3to4x4(xyzrotationmatrix(a,b,y))
    eemat[0,3]=pos[0]
    eemat[1,3]=pos[1]
    eemat[2,3]=pos[2]

    

    return np.matmul(m3x3to4x4(zro(-ur10zrotoffset*(np.pi/180))),eemat)

    
def xyzrotationmatrix(a,b,y):
    #a=alpha=x rot,b=beta=y rot, y=gamma=z rot
    return np.matrix([[c(b)*c(y),s(a)*s(b)*c(y)-c(a)*s(y),c(a)*s(b)*c(y)+s(a)*s(y)],
                      [c(b)*s(y),s(a)*s(b)*s(y)+c(a)*c(y), c(a)*s(b)*s(y)-s(a)*c(y)],
                      [-s(b),s(a)*c(b),c(a)*c(b)]])
def xro(a):
    return np.matrix([[1,0,0],[0,c(a),-s(a)],[0,s(a),c(a)]])
def yro(a):
    return np.matrix([[c(a),0,s(a)],[0,1,0],[-s(a),0,c(a)]])
def zro(a):
    return np.matrix([[c(a),-s(a),0],[s(a),c(a),0],[0,0,1]])

def m3x3to4x4(m3x3):
    m3x4=np.matmul(np.matrix([[1,0,0],[0,1,0],[0,0,1],[0,0,0]]),m3x3)
    return np.matmul(m3x4,np.identity(4))
    



def s(a):
    return np.sin(a)
def c(a):
    return np.cos(a)    



    




def movement_callback(msg):
    rospy.loginfo(msg)
    
    rospy.loginfo("recieved a position")
    ret = msg
    pos=[ret.position.x , ret.position.y,ret.position.z]
    rot=[ret.orientation.x,ret.orientation.y,ret.orientation.z]
    acc=ret.acceleration
    vel=ret.velocity
    prio=ret.priority
    stopmove()  
    rospy.loginfo("it is valid ")
    seteepos(pos[0],pos[1],pos[2],rot[0],rot[1],rot[2],acc,vel)






if __name__ == '__main__':
    rospy.init_node("robot_mover")
    rospy.loginfo("initializing robot")
    initializeall()

    rospy.loginfo("i have begun")

    rospy.sleep(1)


    sub=rospy.Subscriber("/controll/movetarget", custom1 , callback=movement_callback)
    calibrationsub=rospy.Subscriber()
    #pub=rospy.Publisher("/data/robotpose",custom2,queue_size=1)

    rospy.loginfo("i have ended")

    rate = rospy.Rate(10)
    complete=False
    rospy.loginfo("started waiting for messages")
    rospy.spin()
        
    setShdn()





    


