#!/usr/bin/env python3 

import cv2 as cv
import numpy as np
import rospy

from custom_msg_python.msg import allbrickslist, custom, custom1

from sensor_msgs.msg import PointCloud2, Image
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header


#this node keeps track of the configuration and will controll other nodes to work in the corrct order

#configurationmode=False
#holdrobotposition=False
#poses=[[0.4,0.4,0.2],[0.7,0.4,0.2],[1.0,0.4,0.2],[1.3,0.4,0.2],[0.4,0.6,0.2],[0.7,0.6,0.2],[1.0,0.6,0.2],[1.3,0.6,0.2]]
#calibrationstep=0
#allsteps=0

robotpoints=[]
picturepoints=[]
debug=True
def capture_input(msg):
    global points
    if points==[]:
        return
    # a list of bricks have been returned
    #rospy.loginfo("recived a bloc position")
    global debug
    
    if(debug):
        diagnoseacuracy1(msg)
    
    
    
    
    if( False): # (configurationmode and holdrobotposition) or
        # the configuration brick is yellow
        
    
       
        send=msg
        
        red_small=msg.red_small
        red_large=msg.red_large
        blue_small=msg.blue_small
        blue_large=msg.blue_large
        yellow_small=msg.yelow_small
        yellow_large=msg.yelow_large

        red_small_3d=[]
        red_large_3d=[]
        blue_small_3d=[]
        blue_large_3d=[]
        yellow_small_3d=[]
        yellow_large_3d=[]
        
        for x in red_small:
            red_small_3d.append( camera_to_robot_base(getxyp(x.x,x.y)))
        for x in red_large:
            red_large_3d.append(camera_to_robot_base(getxyp(x.x,x.y)))
        for x in blue_small:
            blue_small_3d.append(camera_to_robot_base(getxyp(x.x,x.y)))
        for x in blue_large:
            blue_large_3d.append(camera_to_robot_base(getxyp(x.x,x.y)))
        for x in yellow_small:
            yellow_small_3d.append(camera_to_robot_base(getxyp(x.x,x.y)))
        for x in yellow_large:
            yellow_large_3d.append(camera_to_robot_base(getxyp(x.x,x.y)))

        send.red_small_3d=red_small_3d
        send.red_large_3d=red_large_3d
        send.blue_small_3d=blue_small_3d
        send.blue_large_3d=blue_large_3d
        send.yelow_small_3d=yellow_small_3d
        send.yelow_large_3d=yellow_large_3d
        global blocs_with_3d_points_pub
        blocs_with_3d_points_pub.publish(send)
        global first
        rospy.loginfo(str(len(yellow_large))+"yelow was")
        if(len(yellow_large)!=0 and first ):
            first=False
            
            posemsg=custom1()
            posemsg.x=float(yellow_large_3d[0][0])/1000
            posemsg.y=float(yellow_large_3d[0][1])/1000
            posemsg.z=float(yellow_large_3d[0][2])/1000+0.3
            posemsg.e1=0
            posemsg.e2=0
            posemsg.e3=0
            posemsg.e4=1
            posemsg.velocity=0.1
            posemsg.acceleration=0.1
            posemsg.priority=False



            rospy.loginfo("the message was bublished")
            rospy.loginfo("position was "+str([posemsg.x,posemsg.y,posemsg.z]))
            debtoppub.publish(posemsg)

        #turn image point into 3d camera point
       # point3d=camera_to_robot_base(getxyp(target.x,target.y))
       # logpoint=[int(point3d[0]),int(point3d[1]),int(point3d[2])]
       # rospy.loginfo("got a position of"+str(logpoint)+" for the first yelow brick")
        
        
        
        
        
        # take the next step and send it to the robot
        #message=custom1()
        #p1=poses[calibrationstep]
        #message.pose.positon.x=p1[0]
        #message.pose.positon.y=p1[1]
        #message.pose.positon.z=p1[2]
        #message.priority=True
        #calibrationstep+=1
        #holdrobotposition=False
        #if(calibrationstep>=allsteps):# iff all the steps are complete stop configuring
        #    configurationmode=False
        #    msgconfig=custom()
        #    msgconfig.configurationmode=False
        #    #pubconfig.publish(msgconfig)

        #posesetter.publish(message)

iterations=100
curentiteration=0

blue_small_count_list=[]
blue_large_count_list=[]
red_small_count_list=[]
red_large_count_list=[]
yellow_small_count_list=[]
yellow_large_count_list=[]

def diagnoseacuracy1(allbrickslist):
    global iterations
    global curentiteration

    global blue_small_count_list
    global blue_large_count_list
    global red_small_count_list
    global red_large_count_list
    global yellow_small_count_list
    global yellow_large_count_list

    expected_blue_small=4
    expected_blue_large=4
    expected_red_small=4
    expected_red_large=4
    expected_yellow_small=4
    expected_yellow_large=4

    if(curentiteration==0):
        blue_small_count_list=[]
        blue_large_count_list=[]
        red_small_count_list=[]
        red_large_count_list=[]
        yellow_small_count_list=[]
        yellow_large_count_list=[]
        
    
    if(curentiteration<iterations):
        rospy.loginfo("running iteration "+str(curentiteration))
        blue_small_count_list.append((len(allbrickslist.blue_small),float(len(allbrickslist.blue_small))/float(expected_blue_small)))
        blue_large_count_list.append((len(allbrickslist.blue_large),float(len(allbrickslist.blue_large))/float(expected_blue_large)))
        
        red_small_count_list.append((len(allbrickslist.red_small),float(len(allbrickslist.red_small))/float(expected_red_small)))
        red_large_count_list.append((len(allbrickslist.red_large),float(len(allbrickslist.red_large))/float(expected_red_large)))
    
        yellow_small_count_list.append((len(allbrickslist.yelow_small),float(len(allbrickslist.yelow_small))/float(expected_yellow_small)))
        yellow_large_count_list.append((len(allbrickslist.yelow_large),float(len(allbrickslist.yelow_large))/float(expected_yellow_large)))
        rospy.loginfo("blue small "+str((len(allbrickslist.blue_small),float(len(allbrickslist.blue_small))/float(expected_blue_large))))
        curentiteration+=1
    
    else:
        rospy.loginfo("starting new")
        average_precision=0
        average_distance_from_truth=0
        for z in blue_small_count_list:
            x,y=z
            average_precision+=y
            average_distance_from_truth+=np.abs(y-1)

        rospy.loginfo("blue small had a precision of"+str(average_precision/iterations)+" had "+str(len(blue_small_count_list)))
        
        rospy.loginfo("blue small had a average distance to expectation of of"+str(average_distance_from_truth/iterations))
        average_precision=0
        average_distance_from_truth=0
        for z in blue_large_count_list:
            x,y=z
            average_precision+=y
            average_distance_from_truth+=np.abs(y-1)
        rospy.loginfo("blue large had a precision of"+str(average_precision/iterations))
        rospy.loginfo("blue large had a average distance to expectation of of"+str(average_distance_from_truth/iterations))
        average_precision=0
        average_distance_from_truth=0
        for z in red_small_count_list:
            x,y=z
            average_precision+=y
            average_distance_from_truth+=np.abs(y-1)
        rospy.loginfo("red small had a precision of"+str(average_precision/iterations))
        rospy.loginfo("red small had a average distance to expectation of of"+str(average_distance_from_truth/iterations))
        average_precision=0
        average_distance_from_truth=0
        for z in red_large_count_list:
            x,y=z
            average_precision+=y
            average_distance_from_truth+=np.abs(y-1)
        rospy.loginfo("red large had a precision of"+str(average_precision/iterations))
        rospy.loginfo("red large had a average distance to expectation of of"+str(average_distance_from_truth/iterations))
        average_precision=0
        average_distance_from_truth=0
        for z in red_small_count_list:
            x,y=z
            average_precision+=y
            average_distance_from_truth+=np.abs(y-1)
        rospy.loginfo("yellow small had a precision of"+str(average_precision/iterations))
        rospy.loginfo("yellow small had a average distance to expectation of of"+str(average_distance_from_truth/iterations))
        average_precision=0
        average_distance_from_truth=0
        for z in yellow_large_count_list:
            x,y=z
            average_precision+=y
            average_distance_from_truth+=np.abs(y-1)
        rospy.loginfo("yellow large had a precision of"+str(average_precision/iterations))
        rospy.loginfo("yellow large had a average distance to expectation of of"+str(average_distance_from_truth/iterations))
        global debug
        debug=False



            
            



        
first=True
points=[]
wtidth=0
height=0

def getpoints( cloud):
    #rospy.loginfo("Got scan, projecting")
    cloud_points = list(pc2.read_points(cloud, skip_nans=False, field_names = ("x", "y", "z")))
    return cloud_points


def pointc(msg):
    
    #print("got a mesh")
    
    global points
    global wtidth
    global height

    
    wtidth=1920
    height=1080
    
    points=getpoints(msg)
    #rospy.loginfo("with is"+str(wtidth)+" and height is "+str(height)+" we had "+str(len(points))+" points")
    #rospy.loginfo("center point was "+str(camera_to_robot_base(getxyp(960,540-80))))
    
    

def getxyp(x,y):
    global wtidth
    global points
    return points[int(x)+int(y)*int(wtidth)]


def camera_to_robot_base(xyz):
    
    #ROBOT_CAMPOS=[float(-464),float(-655),float(1284),float(1)]
    ROBOT_CAMPOS=[float(-464),float(-655),float(1284),float(1)]

    radang=np.deg2rad(45+90)
    

    ROBOT_TO_BASE=np.array([[np.cos(radang),-np.sin(radang), 0,   0],
                             [np.sin(radang),np.cos(radang),0,    0],
                             [0,0,                          1,   0],
                             [0,            0,            0,    1]])# this matrix contains the rotation of the kanonisk basis vektore af bord basen, skrevet i robot basen
   
    
    BASE_CAPOS=np.matmul(ROBOT_TO_BASE,ROBOT_CAMPOS)
    
    hy=BASE_CAPOS[1]
    BASE_CAPOS[1]=BASE_CAPOS[0]
    BASE_CAPOS[0]=1000+hy
    #rospy.loginfo("the end effector was at"+str(BASE_CAPOS))

    CAMERA_TO_BASE=np.array([[-1,0, 0,    BASE_CAPOS[0]],
                             [0, 1, 0,    BASE_CAPOS[1]],
                             [0,0,-1,   BASE_CAPOS[2]],
                             [0,            0,            0,    1]])
    #set up the conversion from base to camere
    #CAMERA_TO_BASE = np.linalg.inv(BASE_TO_CAMERA) # invert it
    
    #print(str([int(ROBOT_CAMPOS[0]),int(ROBOT_CAMPOS[1]),int(ROBOT_CAMPOS[2])])+"robot origin of camera")
    #print(str(ROBOT_TO_BASE)+"robot to base")
    #print(str([int(BASE_CAPOS[0]),int(BASE_CAPOS[1]),int(BASE_CAPOS[2])])+"base origin of camera")
    

    post=np.array([xyz[0]*1000,xyz[1]*1000,xyz[2]*1000,1]) # set up the camera position( it is in meters, rest of the systems is in mm)

    return np.matmul(CAMERA_TO_BASE,post) # multiply the camera position vector with the Camerea_to_base transformation


   



        # start 
lasteepos=[]

def iscol(min_thres,max_thres,avg):
    icount=0
    for x in range(len(avg)):
        if (avg[x]<max_thres[x] and avg[x]>min_thres[x]):
            icount+=1
    return icount==len(avg)

def findid(theshlist_max,threshlist_min,avg):
    for x in range(len(theshlist_max)):
        if(iscol(threshlist_min[x],theshlist_max[x],avg)):
            return x
    return -1

blocs_with_3d_points_pub=0
if __name__ == '__main__':
    rospy.init_node("The_fat_conducter") #this node isolates blocks 

    rospy.loginfo("i have begun")
    
    rospy.loginfo("got to 1")

    rospy.sleep(1)
    rospy.loginfo("started waiting for updates")
    sub_image_bricks= rospy.Subscriber("/data/imagebricks/",allbrickslist,callback=capture_input)
    blocs_with_3d_points_pub= rospy.Publisher("/data/imagebricks_3D/",allbrickslist,queue_size=2)
    #pubconfig = rospy.Publisher("/info/configuration",custom,queue_size=1)
    #posereciver = rospy.Subscriber("/info/movecomplete",custom1,position_input)
    pointscriber=rospy.Subscriber("/points2",PointCloud2,callback=pointc)
    debtoppub=rospy.Publisher("/controll/movetarget",custom1,queue_size=1)

    rospy.loginfo("got to 2")
    
    #posesetter= rospy.Publisher("/controll/movetarget",custom1,queue_size=10)
    #msgconfig=custom()
    #msgconfig.configurationmode=True
    #pubconfig.publish(msgconfig)

    #message=custom1()
    #p1=poses[0]
    #message.pose.positon.x=p1[0]
    #message.pose.positon.y=p1[1]
    #message.pose.positon.z=p1[2]
    #message.priority=True
   # posesetter.publish(message)
    #calibrationstep+=1
    


    

    rospy.spin()
    rospy.loginfo("got to 3")


