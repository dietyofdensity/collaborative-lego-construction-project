#!/usr/bin/env python3

import rospy
import numpy as np
import image_block_isolator.scripts.boardprosesing as boardprosesing
from custom_msg_python import blockdata,actimove,actimoveslist,custom2

robposupdate=False # if the script has recieved position data
#from the robot since last loop
costmapupdate=False # if the script has recieved an updated costmap
# from the costmap generator since last node
activmovesupdate=False # if the active moves have changed since the last timestep
goalposupdate=False # if th curent location of all blocks in the workspace have changed

# global list of subscribers for the node
costmapsub=0
robpossub=0
activemovsub=0
goalpossub=0

# global list of last version of subscribed data
costmap=[]
robpos=[]
activemov=[]
goalpos=[]

# list of calback funktions of the subscribe nodes

def costupdater(msg):
    costmap=msg
    costmapupdate=True

def robposupdater(msg):
    robpos=msg
    robposupdate=True

def activemoveupdater(msg):
    activemov=msg
    activemoveupdate=True

def goalposupdater(msg):
    goalpos=msg
    goalposupdate=True


class state: ## the percieved states that the robot can be in
    holdingblock=False
    heldblocktype=0
    heldblockcolur=0
    goalpos=[0,0,0]
    gp2d=[0,0]
    goalrotation=[0,0,0,0]
    userobotpos=True
    goalrelavit=True
    statetype=0
    flatid=0

    def __init__(self,hold,htarg,targetpos,targetrot,org,move_relative,st,flat_id) -> None:
        self.holdingblock=hold
        self.heldblocktype=htarg[0]
        self.heldblockcolur=htarg[1]

        self.goalpos=targetpos
        self.goalrotation=targetrot
        self.userobotpos=org
        self.goalrelavit=move_relative
        self.statetype=st
        self.flatid=flat_id
        
        
        pass
    


class action: ## the actions it can take in order to move between two states
    statesource=0 # the state that this action can be taken to
    statetarget=0 # the state that this action moves toward


class transitions: # the persieved cost of a certain action in any given state
    lastcost=0


class observables:
    gripper_distance=0
    gripper_presure=0
    robot_position=0
    robot_orientation=0
    last_asumed_state=0
    pick_in_progres=False
    place_in_progress=False
    hand_eye_offset=0

class observationfunk: # a function which caluculates the probabilety of being in a state based on the curent observables
    cost=0
    def calculate(stated,observablesd):
        statetypec=stated.statetype
class handcontents:
    brickcolor=0
    bricktype=0
    holding=False

class costmap:
    mapimg=[]
    size=[100,100,1]
    rezolution=0.1
    origen=[0,0,0]
    def globaltopix(self,point):

        opt=(point-self.origen)/self.rezolution
        op=[int(opt[0]),int(opt[1]),int(opt[2])]
        if op[0]<0:
            op[0] = 0
        if op[1]<0:
            op[1] = 0
        if op[2]<0:
            op[2] = 0

        if op[0]>self.size[0]:
            op[0] = self.size[0]-1
        if op[1]>self.size[1]:
            op[1] = self.size[1]-1
        if op[2]>self.size[2]:
            op[2] = self.size[2]-1


        return op
    def local_to_cost(self,localpos):
        return self.mapimg[localpos[0],localpos[1]]



def state_probabilety_funk(sate:state,observables:observables):
    gript=70
     
    #by default we are not holding
    heldp=0.1
    notheldp=0.9
    # if we are curentyl holding an object 
    if(observables.gripperdistance<gript):
        # we have something in the hand but is it the right thing for this state
        if(handcont.brickcolor==sate.heldblockcolur and handcont.bricktype==sate.heldblocktype):
            #we are holding the right brick
            heldp=0.9
            notheldp=0.1

    
    # and this state requires that something is held 

    #if this is a state of type 0 (not holding anything moving towards a pickup position)
        #how far we are from the nearest object
        #weather we are holding anything or not 
        #
    prob=0
    if(sate.statetype==0):

        # get the minimum distance to state 2 targets 
        max_distance=0
        max_id=0
        min_distance=0
        min_id=0
        avg=0

        for x in range(len(widestate[1])):
            dist=distance(observables.robotposition,widestate[1][x].goalpos)
            if(dist<min_distance):
                min_distance=dist
                #min_id=x
            avg+=widestate[1][x].goalpos
        
        avgdist=avg/len(widestate[1])

        xval=min_distance/avg
        #since both distances is linked to the location of the blocks
        # as the robot gets closer to the target the difference will become larger unless th
            # CURVE APROXIMATED AS A 3RD ORDER PLOLYNOMIAL
        dprob=np.power(xval,3)*0.1908+np.power(xval,2)*-1.0606+xval*1.9146-0.0149
        # when the xval aproaches a suficiently high number it means we are probably in this state



        prob=notheldp*dprob

    #if this is a state of type 1 (curently ready for pickup of blocktype x)
        # how close we are to said object
        # what percentage of the active objects are curently of said object type
        # weather or not we are holding
    if(sate.statetype==1):

        
        dist=distance(observables.robotposition,sate.goalpos)
        dprob=0.8858*np.power(0.0981,dist)
        prob=notheldp*dprob

    #if this is a state of type 2 (curently holding blocktype x, moving toward placement)
        # how far we are from the desired position
        # wheather or not we are holding the corect object
    if(sate.statetype==2):
        max_distance=0
        max_id=0
        min_distance=0
        min_id=0
        avg=0

        for x in range(len(widestate[1])):
            # limit the code to only look at states which require the curently held block
            if(handcont.bricktype==widestate[3][x].heldblocktype and handcont.brickcolor==widestate[3][x].heldblockcolur):
                dist=distance(observables.robotposition,widestate[3][x].goalpos)
                if(dist<min_distance):
                    min_distance=dist
                    #min_id=x
                avg+=widestate[1][x].goalpos
        
        avgdist=avg/len(widestate[1])

        xval=min_distance/avg
        # greater distance higher probabilety
        dprob=np.power(xval,3)*0.1908+np.power(xval,2)*-1.0606+xval*1.9146-0.0149
        prob=heldprob*dprob

    #if this is a state of type 3 (curently ready to place the block at the goal position)
        # how close we are to the goal position
        # weather or not we are holding the correct object
    if(sate.statetype==3):
        if(handcont.bricktype==sate.heldblocktype and handcont.brickcolor==sate.heldblockcolur):
            dist=distance(observables.robotposition,sate.goalpos)
            dprob=0.8858*np.power(0.0981,dist)
            prob=notheldp*dprob





   
    heldprob=heldp
    
    


    
    # position based validation
        # how far are we from the states target position
        # how far are we from the states target rotation
    # what is the chance that the human is curently trying to acuire this state


    # does the robot meet the requirements for holding to be in this state 

    print("calculating state")
    return 0
def evalueate_state_state_cost(stateposorg,stateposgoal,costmap,sampledistance):
        #calculate the cost of travel over a 2d cost map from one state to another
        dist=np.sqrt(np.power(stateposorg[0]-stateposgoal[0])+
                     np.power(stateposorg[1]-stateposgoal[1])+
                     np.power(stateposorg[2]-stateposgoal[2]))
        dir1=np.array([(stateposgoal[0]-stateposorg[0]),(stateposgoal[1]-stateposorg[1]),(stateposgoal[2]-stateposorg[2])])/dist
        dir2=-1*dir1


        loops=int(dist/2*sampledistance)
        cost=0
        for x in range(loops):
            poi1=stateposorg+dir1*sampledistance*x
            poi2=stateposgoal+dir2*sampledistance*x
            pixcord1=costmap.globaltopix(poi1)
            pixcord2=costmap.globaltopix(poi2)
            c1=costmap.local_to_cost(pixcord1)*sampledistance
            c2=costmap.local_to_cost(pixcord2)*sampledistance
            cost=cost+c1+c2
        #calculate the jointspace distnce of the action
        joinstpacecost=0

        cost+=joinstpacecost
        return cost

def evaluateloopcost(self,looppath,costmap,sampledistance):
        # calculate a loop the cost of an action series
        # loop path is a array containing the indexes in the state array ordered by sequence
        # first is the start point last is the end piont [1,2,3,4,1]
        # if a state/action is likely to be executed by the human this will bw acounted for by adding an inate cost to the state
        # based on the likelyhood. making the robot focus on tasks not being executed by the human
    print("calculating loop")
    cost=0
    for x in range(len(looppath)-1):
        porg=self.states[looppath[x]]
        pgoal=self.states[looppath[x+1]]
        c1= self.evalueate_state_state_cost(porg,pgoal,costmap,sampledistance)
        cost+=c1



#actions
def move_toward():
    print("moving")
def wait_period():
    print("waiting")
def pick_up():
    print("picking up")
def place_on_plate():
    print("adding to construction")
def put_down_brick():
    print("dropping brick") # block was no longer needed for the build

# functions used to keep track of what state the robot thinks its in 
laststateid=0
laststatecertainty=0

factor=0.1

nextstateid=0
nextstatecertainty=0
# keep goint with the last state until next states certainty is higher

States=0
actions=0

handcont=handcontents()



if __name__ == '__main__':

    rospy.init_node("POMDP_code")

    rospy.loginfo("POMDP was initialized")
    costmapsub=rospy.Subscriber("/data/costmap",boardprosesing.costmapcl,callback=costupdater)
    robpossub=rospy.Subscriber("/data/robotpose",custom2,callback=costupdater)
    activemovsub=rospy.Subscriber("/data/actimoves",actimoveslist,callback=costupdater)
    goalpossub=rospy.Subscriber("/data/allblocks",boardprosesing.costmapcl,callback=costupdater)
    
    rospy.sleep(1)

    rate = rospy.Rate(15)
    while(not rospy.is_shutdown):

        # based on known positions of bricks
        # curently active bricks 
        # curently active placement positions
            # populate the states function
            # populate the actions function
        
        # populate backwards,
            # there is a new state for each placement position, add these to the states with both pre and post placement
            # for each type of block that needs to be placed , find all instances of the blocktype in the goalpos array
            # which contains all known block positions

            #
        activetypes=[] 
        #shapes are 1 2 and 3
            # 1 is 2x2x1
            # 2 is 2x4x1
            # 3 is 2x2x2
        #colors are 1, 2, 3 and 4
            # 1 green
            # 2 red
            # 3 blue
            # 4 yelow
            # 5 grey
        cflat=0

        flatstate=[]
        widestate=[[],[],[],[]]


        for x in activemov.listing:

            #build a state 3
            # position to place a block of a specific type
            amovtyp=x.block.blocktype
            amovecol=x.block.blockcolor
            rotation=np.pi/2*float(x.block.rotation)
            pos=[x.block.xpos,x.block.ypos,x.block.zpos]
            size=[x.block.xlen,x.block.ylen,x.block.zlen]
            cstate1= state(True,[amovtyp,amovecol],pos,rotation,True,True,3,cflat)
            flatstate.append[cstate1]
            widestate[3].append[cstate1]
            cflat+=1




            if not activetypes.__contains__([amovtyp,amovecol]):
                activetypes.append([amovtyp,amovecol])
        for x in activetypes:

            # build a state 2 
            #  state after having grabbed a block
            rotation=[0,0,0,0]
            pos=[0,0,-50]
            cstate2=state(True,x,pos,rotation,False,True,2,cflat)
            flatstate.append(cstate2)
            widestate[2].append(cstate2)
            cflat+=1
        gmove=[]
        for x in (goalpos):
            
            k=goalpos[x]
            this_move=[x.block.blocktype,x.block.blockcolor]
            if activetypes.__contains__(this_move):
                # if this block is the same type as any of the active types
                # build a state 1 
                pos=[x.block.xpos,x.block.ypos,x.block.zpos]
                rotation=x.block.rotation
                cstate3=state(False,this_move,pos,rotation,True,False,1,cflat)
                flatstate.append(cstate3)
                widestate[1].append(cstate3)
                cflat+=1

        # states will only need to be recalculated if the global or active moves are updated 

        




        # the entire system contains only two actions which are easily distinct
        # the robot wil simply use the map and the distance between positions to calculate cost 
        # therefore we can caluculate the state with the highest probabilety first 

        obs=observables()
        maxprob=0
        max_id=0
        for x in range(len(flatstate)):
            prob=state_probabilety_funk(flatstate[x],obs) # returns the probabilety of it being this state
            if(prob>maxprob):
                maxprob=prob
                max_id=x






        
        tempnextstateid=0 # the curent most probable action based on functions

        if(tempnextstateid==nextstateid):# increase certainty for next action
            laststatecertainty*=(1-factor)
            nextstatecertainty*=(1+factor)
        elif(tempnextstateid==laststateid):
            nextstatecertainty=(1-factor)
            laststatecertainty*=(1+factor)
        else:
            #if it is nither of the two then it must be a new state
            #reset the certainty variables and set a new nextstateid
            laststatecertainty=1-factor
            nextstatecertainty=factor
            nextstateid=tempnextstateid


        if(laststatecertainty<nextstatecertainty):
            laststateid=nextstateid
            laststatecertainty=nextstatecertainty
            nextstatecertainty=0
            nextstateid=tempnextstateid

        # act as we would in this state 
        cstate=flatstate[nextstateid]
        cstateype=cstate.statetype
        if(0==cstateype):
            print("0")
            # we are in state 0 and looking for a block to pick up 
            # therefore we are trying to move toward position that has a brick to pick up.
            # all of the active positions are marked in the state 1 list 
            # in order to find the most optimal one 
            lowest=10000
            low_id=0
            for x in range(len(widestate[1])):
                cost=evalueate_state_state_cost(robpos,widestate[1][x].targetpos,costmap,0.1)
                if(cost<lowest):
                    low_id=x
                    lowest=cost
            # construct a move comand toward this one

            targetpose=()


            
            



        elif(1==cstateype):
            print("1")
            # we are crently placed above the position for pick up
            # based on imagery chek if there is an ofset between where the gripper should be
                # do hand eye cordinationx
            if(not observables.pick_in_progres):
                pick_up()

            # if a picup is not initiated 
                # start of pick up the brick
            # else wait until the program is done

            # if the robots area in the costmap is beyond a cetain cost
                # halt as the human might be too close.


        elif(2==cstateype):
            print("2")
            # loop over place positions
                # find the one with the lowest cost
            lowest=10000
            low_id=-1
            for x in range(len(widestate[2])):
                if(widestate[2][x].heldblocktype==handcont.bricktype and widestate[2][x].heldblockcolur==handcont.brickcolor and handcont.holding):

                    cost=evalueate_state_state_cost(robpos,widestate[1][x].targetpos,costmap,0.1)
                    if(cost<lowest):
                        low_id=x
                        lowest=cost
            if(low_id==-1):
                put_down_brick()
            else:
                #build a move comand 


                move_toward()
        
            # if there are no entries in the array 
                # drop this brick 
            

        elif(3==cstateype):
            print("3")
            # we are crently placed above the position for place
            # based on imagery chek if there is an ofset between where the gripper should be
                # do hand eye cordination
            # if a picup is not initiated 
                # start of place the brick
            # else wait until the program is done
            robotarecost=0
            threshold=200
            if (robotarecost>threshold):
                if(not observables.pick_in_progres):
                    place_on_plate()



            # if the robots area in the costmap is beyond a cetain cost
                # halt as the human might be too close.




        rospy.sleep(rate)


def distance(x,y):
    xc=x[0]-y[0]
    yc=x[1]-y[1]
    zc=x[2]-y[2]
    return np.sqrt((xc*xc)+(yc*yc)+(zc*zc))


