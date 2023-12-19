#!/usr/bin/env python3.9

from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import numpy as np

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

from dryveCode import targetPosition, dryveInit, homing, targetVelocity, profileVelocity, getPosition,setShdn
from custom_msg_python.msg import custom1, custom2
scale=1

def setendeffector(x,y,z,e1,e2,e3,e4):
    # load an end effector position for the planner
    # positions are of type float in meter scale
        # the camera should also be in meter scale
    pose_goal = move_group.get_current_pose().pose
    pose_goal.orientation.x = e1
    pose_goal.orientation.y = e2
    pose_goal.orientation.z = e3
    pose_goal.orientation.w = e4
    pose_goal.position.x = x
    pose_goal.position.y = y
    pose_goal.position.z = z

    move_group.set_pose_target(pose_goal)

def plan_move():
    # plan the move 
        # WAS the plan sucsesful
            #

    # `go()` returns a boolean indicating whether the planning and execution was successful.
    
    
    success = move_group.go(wait=False)# if wait is false it should return true if planning was a sucsses
    if(success):
        move_group.stop()
        move_group.clear_pose_targets()

    else:
        move_group.stop()
        move_group.clear_pose_targets()
        #request a retry with an ajustment, try moving the dryve
        rospy.logerr("no valid path")

    # Calling `stop()` ensures that there is no residual movement
     
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets().
     
def plancartesian(x,y,z,e1,e2,e3,e4):
    waypoints = []
    wpose = move_group.get_current_pose().pose
    wpose.position.x = scale * x  # First move up (z)
    wpose.position.y = scale * y  # and sideways (y)
    wpose.position.z = scale * z
    waypoints.append(copy.deepcopy(wpose))

# We want the Cartesian path to be interpolated at a resolution of 1 cm
# which is why we will specify 0.01 as the eef_step in Cartesian
# translation.  We will disable the jump threshold by setting it to 0.0,
# ignoring the check for infeasible jumps in joint space, which is sufficient
# for this tutorial.
    (plan, fraction) = move_group.compute_cartesian_path(
        waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
    )  # jump_threshold

# Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan, fraction
def exeuteplan(plan):
    # first joint state must be close to first position
    move_group.execute(plan, wait=True)

robismoving=False


def xro(a):
    return np.matrix([[1,0,0],[0,c(a),-s(a)],[0,s(a),c(a)]])
def yro(a):
    return np.matrix([[c(a),0,s(a)],[0,1,0],[-s(a),0,c(a)]])
def zro(a):
    return np.matrix([[c(a),-s(a),0],[s(a),c(a),0],[0,0,1]])
def s(a):
    return np.sin(a)
def c(a):
    return np.cos(a)

curdryveconfig=0

def seteepos(x,y,z,e1,e2,e3,e4,a,v,prio,msg):
    # this function sets the end effector to a position in our table space

    # based on the cordinate in mm
    rospy.loginfo("setting information")
    edgedistance=4000 #40 cm # set higher than max distance to simulate it not moving
    ur10zrotoffset=135 
    dryvepart=0
    xinMM=int(x*1000) # as the cordinates for moveit are in meters(float) we must convert to mm

    if(abs(curdryveconfig-xinMM)>edgedistance):
        if(x<curdryveconfig):
            dryvepart=xinMM+edgedistance*0.5
            dryvepart=int(dryvepart)
            x-=dryvepart
        else:
            dryvepart=xinMM-edgedistance*0.5
            dryvepart=int(dryvepart)
            xinMM-=dryvepart
    x=float(xinMM/1000)
    

    poseactual=np.matmul(zro(ur10zrotoffset*np.pi/180),np.array([x,y,z])) # then we rotate the vector to align with the base of the robot

    rotactual=0
    if(prio):# if we are waiting until the move is done
        angle=ur10zrotoffset*np.pi/180
        qx = 0 * s(angle/2)
        qy = 0 * s(angle/2)
        qz = 1 * s(angle/2)
        qw = c(angle/2)
        
        
        rotactual=np.array([e1,e2,e3,e4])*np.array([qx,qy,qz,qw]) # rotate the vector to the correct cordinate system
    else:
        angle=-pi/2
        qx = -0.5 * s(angle/2)
        qy = 0.5 * s(angle/2)
        qz = 0 * s(angle/2)
        qw = c(angle/2)
       
        rotactual=np.array([qx,qy,qz,qw]) # set the end effector to point forward on the table
     
    
    
    if(prio): # if this has priority
        # stop all action move to the new location
        move_group.clear_pose_targets()
        move_group.stop()
        plan,fragret = plancartesian(x,y,z,rotactual[0],rotactual[1],rotactual[2],rotactual[3])
        
        #targetPosition(dryvepart)
        move_group.execute(plan, wait=False)# need to make archetecture to acount for faulty planning
    else:
        setendeffector(x,y,z,rotactual[0],rotactual[1],rotactual[2],rotactual[3])
       # targetPosition(dryvepart)
        move_group.go()
        sendmsg=custom1()
        
        pret=move_group.get_current_pose(end_effector_link="")
        sendmsg.pose.position.x=pret.positon.x
        sendmsg.pose.position.y=pret.positon.y
        sendmsg.pose.position.z=pret.positon.z
        sendmsg.pose.orientation.x=pret.orientation.x
        sendmsg.pose.orientation.y=pret.orientation.y
        sendmsg.pose.orientation.z=pret.orientation.z
        sendmsg.pose.orientation.w=pret.orientation.w
        movepup.publish(sendmsg)



    rospy.loginfo("the pos aray was "+str([poseactual[0,0],poseactual[0,1],poseactual[0,2]]))

    
 





def movement_callback(msg):
    rospy.loginfo(msg)
    
    rospy.loginfo("recieved a position")
    ret = msg
    pos=[ret.pose.position.x , ret.pose.position.y,ret.pose.position.z]
    rot=[ret.pose.orientation.x,ret.pose.orientation.y,ret.pose.orientation.z,ret.pose.orientation.w]
    acc=ret.acceleration
    vel=ret.velocity
    prio=ret.priority
    
    
    rospy.loginfo("it is valid ")


    rospy.loginfo("the position was " +str(pos)+" with orientation of "+rot)
    seteepos(pos[0],pos[1],pos[2],rot[0],rot[1],rot[2],rot[3],acc,vel,prio,msg)






if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)    
    rospy.init_node("move_group_python_interface_tutorial", anonymous=True)
    rospy.loginfo("initializing robot")


    # robot contains inf about the robots curent configuration
    robot = moveit_commander.RobotCommander()
    # contains functions for updating the obstacles and objects in the robots workspace
    scene = moveit_commander.PlanningSceneInterface()

    # set up the target of this code in order to use planning with moveit
    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    
    # displaying planned paths
    display_trajectory_publisher = rospy.Publisher(
    "/move_group/display_planned_path",
    moveit_msgs.msg.DisplayTrajectory,
    queue_size=20,)
    sub=rospy.Subscriber("/controll/movetarget", custom1 , callback=movement_callback)
    
    movepup=rospy.Publisher("/info/movecomplete",custom1,queue_size=10)

    

    #subcal=rospy.Subscriber("/controll/calibratetransform", custom1 , callback=movement_callback)
    #pubcal=rospy.Publisher("/controll/calibratetransform", custom1 , callback=movement_callback)
    
    rospy.loginfo("i have begun")
    #dryveInit()
    #targetPosition(500)
    move_group.clear_pose_targets()
    move_group.stop()


    waypoints = []

    wpose = move_group.get_current_pose().pose
    rospy.loginfo(wpose)
    wpose.position.x = 0.6 # First move up (z)
    wpose.position.y =  0.4  # and sideways (y)
    wpose.position.z =  0.4  # and sideways (y)
    waypoints.append(copy.deepcopy(wpose))


# We want the Cartesian path to be interpolated at a resolution of 1 cm
# which is why we will specify 0.01 as the eef_step in Cartesian
# translation.  We will disable the jump threshold by setting it to 0.0,
# ignoring the check for infeasible jumps in joint space, which is sufficient
# for this tutorial.
    (plan, fraction) = move_group.compute_cartesian_path(
        waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
    )  # jump_threshold

    # Note: We are just planning, n ot asking move_group to actually move the robot yet:
    #move_group.execute(plan, wait=True)




    msg=custom1()
    curdryveconfig=1000
    pos=[0.9570573921071889, 0.8562524932141415, 0.18999994659423827]

    msg.x=pos[0]
    msg.y=pos[1]
    msg.z=pos[2]
    msg.e1=0
    msg.e2=0
    msg.e3=0
    msg.e4=1

    msg.velocity=0.1
    msg.acceleration=0.1
    msg.priority=False

    seteepos(0.9570573921071889, 0.8562524932141415, 0.18999994659423827,0,0,0,1,0.1,0.1,True,msg)
    rospy.sleep(1)


    sub=rospy.Subscriber("/controll/movetarget", custom1 , callback=movement_callback)
    #pub=rospy.Publisher("/data/robotpose",custom2,queue_size=1)

    rospy.loginfo("i have ended")

    rate = rospy.Rate(10)
    complete=False
    rospy.loginfo("started waiting for messages")
    rospy.spin()





    


