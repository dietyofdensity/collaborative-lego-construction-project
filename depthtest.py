#!/usr/bin/env python3 


import cv2 as cv

import rospy
import sensor_msgs.point_cloud2 as pc2

from cv_bridge import CvBridge
from sensor_msgs.msg import PointCloud2, Image,CompressedImage
from std_msgs.msg import Header
depthmap=0
hasdepth=False
lastdepth=[]
hasbgra=False
lastbgra=[]
running=True

bridge=CvBridge()

def getpoints( cloud):
    rospy.loginfo("Got scan, projecting")
    gen = pc2.read_points(cloud, skip_nans=True, field_names=("x", "y", "z"))
    return gen

def maketablemask_plane(img,start,end):
    #print(img[400,400])
    mask=cv.inRange(img, start, end)
    #print(img[400,400])
    
    kernel = cv.getStructuringElement(cv.MORPH_RECT, (41,41))
    opening = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)
    
    
    
    global tablemaskpublisher
    tablemaskpublisher.publish(bridge.cv2_to_compressed_imgmsg(opening))
def maketablemask_cutoff(img,start,end):
    
    mask=cv.inRange(img, 0, end)
    
    kernel = cv.getStructuringElement(cv.MORPH_RECT, (41,41))
    opening = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)
    
    global tablemaskpublisher_cutof
    tablemaskpublisher_cutof.publish(bridge.cv2_to_compressed_imgmsg(opening))

procsseingmask=False
def depth(msg): # proces the depth umage
    global hasdepth
    global lastdepth
    global procsseingmask
    if(not procsseingmask):
        lastdepth=bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='passthrough')
        procsseingmask=True
    
    
    
   
    #maketablemask_cutoff(lastdepth,0,1500) # publish a mask that only removes the floor
    #hasdepth=True
    #rospy.loginfo("sent a new mask")
    
    

def bgra(msg): # proces the rgb image
    global hasbgra
    global lastbgra

    try:
        lastbgra=bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        #print("gotimage")

    except:
        #print("not an rgb image")
        s=False
    hasbgra=True
    
    


points=[]
wtidth=0
height=0
def pointc(msg):
    
    msg1=PointCloud2()
    global points
    global wtidth
    global height

    points=getpoints(msg)
    wtidth=msg1.width
    height=msg1.height

def getxyp(x,y):
    global wtidth
    global points
    return points[x+y*wtidth]

    


depthscriber=0
rgbascriber=0
pointscriber=0

tablemaskpublisher=0



if __name__ == '__main__':
    rospy.init_node("Depthcamera_tester")
    depthscriber=rospy.Subscriber("/depth_to_rgb/image_raw/compressed",CompressedImage,callback=depth)

    #rgbascriber=rospy.Subscriber("/rgb/image_raw",Image,callback=bgra)

    #pointscriber=rospy.Subscriber("/points2",PointCloud2,callback=pointc)

    tablemaskpublisher=rospy.Publisher("/binary/masks/table_plane",CompressedImage,queue_size=1)

    tablemaskpublisher_cutof=rospy.Publisher("/binary/masks/table_cutoff",CompressedImage,queue_size=1)
    # maby shift these to the compresed versions as reading raw images consumes about 3 mb of ram pr image
    
    rate=rospy.Rate(2)

    while(not rospy.is_shutdown()):
        # send a single depth mask
        if(procsseingmask==True):
            maketablemask_plane(lastdepth,1200,1500) # publisg a mask which has only the plane of the table aswell as legos
            procsseingmask=False
            rospy.loginfo("sent a new mask")
        rospy.loginfo("sleeping")
        rate.sleep()
