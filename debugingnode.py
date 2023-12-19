#!/usr/bin/env python3

import rospy
from custom_msg_python.msg import custom1


if __name__ == '__main__':
    rospy.init_node("debug_node")

    rospy.loginfo("this node wil publish to its preset topics then stop")
    pub=rospy.Publisher("/controll/movetarget",custom1,queue_size=10)
    

    rospy.sleep(1)

    rospy.loginfo("publishing")
    
    p1=custom1()
    #[-900,-500,-160]
    p1.position.x=700
    p1.position.y=700
    p1.position.z=400
    #[0.4,-1.55,0.6]
    p1.orientation.x=0
    p1.orientation.y=0
    p1.orientation.z=0
    p1.acceleration=0.1
    p1.velocity=0.1
    p1.priority=True

    pub.publish(p1)

    rospy.sleep(1)
    rate=rospy.Rate(1)
    while(True):
        pub.publish(p1)
        rate.sleep()
        rospy.loginfo("published")


    rospy.loginfo("setup has ended en")
    