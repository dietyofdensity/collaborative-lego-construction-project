#!/usr/bin/env python3 


import numpy as np
import rospy
from custom_msg_python.msg import blockdata


constructionstring=("0;2;0;-157.6875*19.1*-141.8125;270;2*1*2;5*1*6;2"
"X1;2;0;-157.6875*19.1*-46.5625;270;2*1*2;5*1*12;3"+
"X2;0;1;-141.8125*0*-141.8125;0;4*1*2;5*0*6;-1"+
"X3;0;1;-141.8125*0*-46.5625;0;4*1*2;5*0*12;-1"+
"X4;1;1;-110.0625*19.1*-141.8125;270;4*1*2;7*1*6;2"+
"X5;1;1;-125.9375*19.1*-30.6875;90;4*1*2;6*1*13;3"+
"X6;2;1;-125.9375*38.2*-94.1875;0;4*1*2;6*2*9;4*5"+
"X7;2;0;-110.0625*19.1*-141.8125;0;2*1*2;8*1*6;2"+
"X8;2;0;-110.0625*19.1*-46.5625;0;2*1*2;8*1*12;3"+
"X9;2;1;-30.6875*57.3*-94.1875;180;4*1*2;12*3*9;13*11*6"+
"X10;2;1;-78.3125*57.3*-78.3125;0;4*1*2;9*3*10;6*12*13"+
"X11;0;1;-14.8125*38.2*-110.0625;180;4*1*2;13*2*8;16"+
"X12;0;1;-14.8125*38.2*-46.5625;180;4*1*2;13*2*12;17"+
"X13;2;1;1.0625*38.2*-78.3125;180;4*1*2;14*2*10;17*16"+
"X14;0;0;-30.6875*0*-141.8125;0;2*1*2;13*0*6;-1"+
"X15;0;0;-30.6875*0*-46.5625;0;2*1*2;13*0*12;-1"+
"X16;1;1;1.0625*19.1*-141.8125;270;4*1*2;14*1*6;14"+
"X17;1;1;-14.8125*19.1*-30.6875;90;4*1*2;13*1*13;15")


class move:
    tempid=0
    colorid=0
    sizeid=0
    rotation=0
    worldpos=[0,0,0]
    grid_size=[0,0,0]
    grid_pos=[0,0,0]
    pre_requisites=[]
    
    def __init__(self,movestring:str):
        split1=movestring.split(";")
        self.tempid=int(split1[0])
        self.colorid=1+int(split1[1])
        self.sizeid=int(split1[2])
        split_2=split1[3].split("*") # world position index
        self.worldpos=[float(split_2[0]),float(split_2[1]),float(split_2[2])]
        self.rotation=int(float(split1[4]))
        split_3=split1[5].split("*")
        self.grid_size=[int(split_3[0]),int(split_3[2]),int(split_3[1])]
        split_4=split1[6].split("*")
        self.grid_pos=[int(split_4[0]),int(split_4[2]),int(split_4[1])]
        split_5=split1[7].split("*")
        prelist=[]
        for x in split_5:
            prelist.append(int(x))
        self.pre_requisites=prelist
        #preload the rotation into the size 
        turns=self.rotation/90
        if(turns%2==1):
            tsize=self.grid_size[0]
            self.grid_size[0]=self.grid_size[1]
            self.grid_size[1]=tsize

        if(turns==1):
            self.grid_pos[1]-=self.grid_size[1]-1

        if(turns==2):
            p=1
            self.grid_pos[0]-=self.grid_size[0]-1
            self.grid_pos[1]-=self.grid_size[1]-1
        if(turns==3):
            self.grid_pos[0]-=self.grid_size[0]-1
            #self.grid_pos[1]-=self.grid_size[1]-1

        
        


completed_moves_list=[]
posible_moves_list=[]
possible_moves_class=[]

moves=[]
logic_boards_view=[]
logic_boards_depth=[]
boardsize=[20,20,10]

boardheight_confidence=[] 
boardcolor_confidence=[]


def moves_initialize():
    global constructionstring
    global moves
    global boardcolor_confidence
    global boardheight_confidence
    
    boardheight_confidence=np.zeros((20,20,5),float) # contains the a float between 0 and 1 of how confident it in each possible color of the cell
    boardcolor_confidence=np.zeros((20,20,4),float) # contains the a float between 0 and 1 of how confident it in each possible color of the cell



    split_1=constructionstring.split("X")
    for x in split_1:
        moves.append(move(x))


def loadmoves(activemoves:list,xlen:int,ylen:int,zlen:int):
    colorswatch=[] # based on the color ids from our construction
    classmoves=[] # find the coresponding move class for each move id
    for x in activemoves:
        for y in range(len(moves)):
            if(moves[y].tempid==x):
                classmoves.append(moves[y])
                break
    
    space=np.zeros((xlen,ylen,zlen),int)
    for idx in classmoves:
        org=idx.grid_pos
        size=idx.grid_size
        
        # based on the rotation swatch the x and z axis
        for x in range(size[0]):
            for y in range(size[1]):
                for z in range(size[2]):
                    space[org[0]+x,org[1]+y,org[2]+z]=int(idx.colorid)
                    # occupy the space with the color 
    return space

def gettopview(space:np.array,xlen:int,ylen:int,zlen:int):
    view=np.zeros((xlen,ylen))
    height=np.zeros((xlen,ylen))
    for x in range(xlen):
        for y in range(ylen):
            for z in np.flip(range(zlen)): # we read the z axis from the top down
                if(view[x,y]==0): # if it is equal to zero it is not set yet
                    if(space[x,y,z]!=0): # if the space is not empty
                        view[x,y]=space[x,y,z] # set the collor
                        height[x,y]=z # set the height it was found at 
    return (view,height)

def compare(view_1,view_2):
    return compare(view_1[0],view_2[0],view_1[1],view_2[1])

def compare(view_1,view_2,height_1,height_2):
    if(view_1.shape==view_2.shape):
        shape=view_1.shape
        targetcount=shape[0]*shape[1]
        count=0
        for x in range(shape[0]):
            for y in range(shape[1]):
                if(view_1[x,y]==view_2[x,y] and height_1[x,y]==height_2[x,y]):
                    count+=1
        if(count==targetcount):
            return True
    return False

def compare_probabilety(view_1_absolute,view_2_probable,height_1_absolute,height_2_probable,space:np.array,probmove:move):
    if(view_1_absolute.shape==view_2_probable.shape):
        shape=view_1_absolute.shape
        #print(shape)
        targetcount=shape[0]*shape[1]
        boardprob=0
        print(view_1_absolute)
        for x in range(shape[0]):
            for y in range(shape[1]):
                
                zid=int(view_1_absolute[x,y])
                colprob=space[x,y,zid]  # what is the probabilety that this cell has the correct collor
                depth=np.abs(height_1_absolute[x,y]-height_2_probable[x,y])/5 # what is the probability that this cell has the correct height
                depth=1-depth*depth
                boardprob+=depth*colprob

        return boardprob/(20*20)
    return 0
def compare_probabilety2(view_1_absolute,view_2_probable,height_1_absolute,height_2_probable,space:np.array,probmove:move):
    # use a gradian distance to lower the effect of cells far away from the chagened cells 
    fallofradius=2
    moveorg=probmove.grid_pos
    if(view_1_absolute.shape==view_2_probable.shape):
        shape=view_1_absolute.shape
        #print(shape)
        targetcount=shape[0]*shape[1]
        boardprob=0
        totalweight=0

        depthweight=0.3

        for x in range(shape[0]):
            for y in range(shape[1]):
                dist= abs(x-moveorg[0])+abs(y-moveorg[1]) #manhatan distance 
                dist2= abs(x-moveorg[0]+probmove.grid_size[0])+abs(y-moveorg[1]+probmove.grid_size[1])
                lower=np.min([dist,dist2])# take the min value 
                weight=float(fallofradius-lower)/float(fallofradius)
                totalweight+=weight

                
                #print(view_1_absolute[x,y])
                zid=int(view_1_absolute[x,y])
                colprob=space[x,y,zid]  # what is the probabilety that this cell has the correct collor
                depth=np.abs(height_1_absolute[x,y]-height_2_probable[x,y])/5 # what is the probability that this cell has the correct height
                depth=1-depth*depth*depthweight
                boardprob+=depth*colprob*weight

        return boardprob/totalweight
    return 0
def compare_probabilety3(view_1_absolute,view_2_probable,height_1_absolute,height_2_probable,space:np.array,probmove:move):
    # use a bounding box to limit the are of expected change 
    
    if(view_1_absolute.shape==view_2_probable.shape):
        shape=view_1_absolute.shape
        #print(shape)
        targetpos=probmove.grid_pos
        boundsize=4
        boardprob=0
        # set up bounds
        lowerx=targetpos[0]-boundsize
        lowery=targetpos[1]-boundsize
        upperx=targetpos[0]+boundsize+probmove.grid_size[0]
        uppery=targetpos[1]+boundsize+probmove.grid_size[1]
        #limit to the bounds of the array
        if(lowerx<0):lowerx=0
        if(lowery<0):lowery=0
        if(upperx>shape[0]-1):upperx=shape[0]-1
        if(uppery>shape[1]-1):uppery=shape[1]-1
        
        #establish spaces
        xspace=range(shape[0])[lowerx:upperx]
        uspace=range(shape[1])[lowery:uppery]
        celcount=0
        for x in xspace:
            for y in uspace:
                #print(view_1_absolute[x,y])
                zid=int(view_1_absolute[x,y])
                colprob=space[x,y,zid]  # what is the probabilety that this cell has the correct collor
                depth=np.abs(height_1_absolute[x,y]-height_2_probable[x,y])/5 # what is the probability that this cell has the correct height
                depth=1-depth*depth
                boardprob+=depth*colprob
                celcount+=1

        return boardprob/celcount
    return 0
def compare_probabilety4(view_1_absolute,view_2_probable,height_1_absolute,height_2_probable,space:np.array,probmove):
    # use multiplikative probbabilety and a bounding box
    
    if(view_1_absolute.shape==view_2_probable.shape):
        shape=view_1_absolute.shape
        #print(shape)
        targetpos=probmove.grid_pos
        boundsize=2
        boardprob=0
        # set up bounds
        lowerx=targetpos[0]-boundsize
        lowery=targetpos[1]-boundsize
        upperx=targetpos[0]+boundsize+probmove.grid_size[0]
        uppery=targetpos[1]+boundsize+probmove.grid_size[1]
        #limit to the bounds of the array
        if(lowerx<0):lowerx=0
        if(lowery<0):lowery=0
        if(upperx>shape[0]-1):upperx=shape[0]-1
        if(uppery>shape[1]-1):uppery=shape[1]-1
        
        #establish spaces
        xspace=range(shape[0])[lowerx:upperx]
        uspace=range(shape[1])[lowery:uppery]
        celcount=0
        for x in xspace:
            for y in uspace:
                #print(view_1_absolute[x,y])
                zid=int(view_1_absolute[x,y])
                colprob=space[x,y,zid]  # what is the probabilety that this cell has the correct collor
                colprob=0.5+colprob*0.5
                depth=np.abs(height_1_absolute[x,y]-height_2_probable[x,y])/5 # what is the probability that this cell has the correct height
                depth=1-depth*depth*0.3
                boardprob*=depth*colprob
                celcount+=1

        return boardprob
    return 0
def compare_probabilety5(view_1_absolute,view_2_probable,height_1_absolute,height_2_probable,space:np.array):
    # use multiplikative probbabilety  
    
    if(view_1_absolute.shape==view_2_probable.shape):
        shape=view_1_absolute.shape
        #print(shape)
        targetcount=shape[0]*shape[1]
        boardprob=0
        for x in range(shape[0]):
            for y in range(shape[1]):
                #print(view_1_absolute[x,y])
                zid=int(view_1_absolute[x,y])
                colprob=space[x,y,zid]  # what is the probabilety that this cell has the correct collor
                depth=np.abs(height_1_absolute[x,y]-height_2_probable[x,y])/5 # what is the probability that this cell has the correct height
                depth=1-depth*depth
                boardprob*=depth*colprob

        return boardprob
    return 0
def compare_probabilety6(view_1_absolute,view_2_probable,height_1_absolute,height_2_probable,space:np.array,probmove:move):
    # use a bounding box to limit the area and a gradient
    # use a gradian distance to lower the effect of cells far away from the chagened cells 
    
    
    if(view_1_absolute.shape==view_2_probable.shape):
        shape=view_1_absolute.shape
        #print(shape)
        targetpos=probmove.grid_pos
        boundsize=4
        
        fallofradius=3
        boardprob=0
        # set up bounds
        lowerx=targetpos[0]-boundsize
        lowery=targetpos[1]-boundsize
        upperx=targetpos[0]+boundsize+probmove.grid_size[0]
        uppery=targetpos[1]+boundsize+probmove.grid_size[1]
        #limit to the bounds of the array
        if(lowerx<0):lowerx=0
        if(lowery<0):lowery=0
        if(upperx>shape[0]-1):upperx=shape[0]-1
        if(uppery>shape[1]-1):uppery=shape[1]-1
        
        #establish spaces
        xspace=range(shape[0])[lowerx:upperx]
        uspace=range(shape[1])[lowery:uppery]
        
        totalweight=0
        for x in xspace:
            for y in uspace:
                dist= abs(x-targetpos[0])+abs(y-targetpos[1]) #manhatan distance 
                dist2= abs(x-targetpos[0]+probmove.grid_size[0])+abs(y-targetpos[1]+probmove.grid_size[1])
                lower=np.min([dist,dist2])# take the min value 
                weight=float(fallofradius-lower)/float(fallofradius)
                totalweight+=weight
                #print(view_1_absolute[x,y])
                zid=int(view_1_absolute[x,y])
                colprob=space[x,y,zid]  # what is the probabilety that this cell has the correct collor
                depth=np.abs(height_1_absolute[x,y]-height_2_probable[x,y])/5 # what is the probability that this cell has the correct height
                depth=1-depth*0.6
                boardprob+=colprob*weight*depth
                

        return boardprob/totalweight
    return 0


def compare_probabilety7(view_1_absolute,view_2_probable,height_1_absolute,height_2_probable,space:np.array,probmove:move):
    # use a bounding box to limit the area and a gradient
    # use a gradian distance to lower the effect of cells far away from the chagened cells 
    
    
    if(view_1_absolute.shape==view_2_probable.shape):
        shape=view_1_absolute.shape
        boundsize=0
        targetpos=probmove.grid_pos
        lowerx=targetpos[0]-boundsize
        lowery=targetpos[1]-boundsize
        upperx=targetpos[0]+boundsize+probmove.grid_size[0]
        uppery=targetpos[1]+boundsize+probmove.grid_size[1]
        #limit to the bounds of the array
        if(lowerx<0):lowerx=0
        if(lowery<0):lowery=0
        if(upperx>shape[0]-1):upperx=shape[0]-1
        if(uppery>shape[1]-1):uppery=shape[1]-1
        
        count=0
        false=0
        maxfalse=2
        xspace=range(shape[0])[lowerx:upperx]
        uspace=range(shape[1])[lowery:uppery]
        cells=0
        for x in xspace:
            for y in uspace:
                zid=int(view_1_absolute[x,y])
                colprob=space[x,y,zid]  
                cells+=1
                if(colprob==space[x,y,zid]>0.9):
                    count+=1
                else:
                    false+=1
        return (maxfalse-false)/maxfalse
    return 0

def compare_probabilety8(view_1_absolute,view_2_probable,height_1_absolute,height_2_probable,space:np.array,probmove:move):
    # use a bounding box to limit the area and a gradient
    # use a gradian distance to lower the effect of cells far away from the chagened cells 
    
    
    if(view_1_absolute.shape==view_2_probable.shape):
        shape=view_1_absolute.shape
        
        
        
        count=0
        false=0
        maxfalse=2
        cells=0
        for x in range(shape[0]):
            for y in range(shape[1]):
                zid=int(view_1_absolute[x,y])
                colprob=space[x,y,zid]  
                cells+=1
                if(colprob==space[x,y,zid]>0.9):
                    count+=1
                else:
                    false+=1
        return (maxfalse-false)/maxfalse
    return 0


def load_from_msg(msg): # recives a message of type blockdata
    
    unfolded_color=msg.fieldcolors
    unfolded_height=msg.fieldheight
    xwidth=msg.height
    ywidth=msg.width
    
    view=np.zeros((xwidth,ywidth))
    height=np.zeros((xwidth,ywidth))
    global boardcolor_confidence
    global boardheight_confidence

    for x in range(xwidth):
        for y in range(ywidth):
            col=unfolded_color[foldxy(x,y,xwidth,ywidth)]
            dep=unfolded_height[foldxy(x,y,xwidth,ywidth)]
            # update the confidence map
            for z in range(4):
                if(col==z):
                    
                    if(boardcolor_confidence[x,y][z]<1):
                    
                        boardcolor_confidence[x,y][z]*=1.5
                        boardcolor_confidence[x,y][z]+=0.3
                    if(boardcolor_confidence[x,y][z]>1): # make shure it does not exeed one
                        boardcolor_confidence[x,y][z]=1
                    
                else:
                    boardcolor_confidence[x,y][z]*=0.5
            for z in range(5):
                if(dep==z):
                    if(boardheight_confidence[x,y][z]<1):
                        boardheight_confidence[x,y][z]*=1.5
                        boardheight_confidence[x,y][z]+=0.3
                    
                    if(boardheight_confidence[x,y][z]>1):
                        boardheight_confidence[x,y][z]=1
                else:
                    boardheight_confidence[x,y][z]*=0.5
            # pick the color with highest confidence values
            linec=np.argmax( boardcolor_confidence[x,y])
            lined=np.argmax(boardheight_confidence[x,y])
            #
            view[x,y]=linec
            height[x,y]=lined
    #rospy.loginfo(view)
    #rospy.loginfo(height)
    
    return (view,height)

def foldxy(x,y,xlen,ylen):
    return(x+y*xlen)

def update_expectations(complete_moves,posible_moves):
    global logic_boards_view
    global logic_boards_depth

    logic_boards_view=[]
    logic_boards_depth=[]
    t1,t2=gettopview( loadmoves(complete_moves,boardsize[0],boardsize[1],boardsize[2]),boardsize[0],boardsize[1],boardsize[2])
    logic_boards_view.append(t1)
    logic_boards_depth.append(t2)
    #print(logic_boards)
    for x in posible_moves:
        trymoves=[]
        trymoves.extend(complete_moves)
        trymoves.append(x)
        c1,c2=gettopview(loadmoves(trymoves,boardsize[0],boardsize[1],boardsize[2]),boardsize[0],boardsize[1],boardsize[2])
        logic_boards_view.append(c1)
        logic_boards_depth.append(c2)
        print(trymoves)

def valid_board_scan(msg):
    # got a valid scan from the board
    rospy.loginfo("got a message")
    #calculate which moves are possible
    posible_moves_list=find_next_steps(completed_moves_list,moves) 
    
    #update the expected boards based on the curent complete and posible moves
    update_expectations(completed_moves_list,posible_moves_list)
    # load the view and height from the message
    board_message=load_from_msg(msg)
    
    global logic_boards_view
    global logic_boards_depth
    boardtrue=-1
    global boardcolor_confidence
    global boardheight_confidence
    probabileties=[]
    
    for x in range(len(logic_boards_view)):
        #print(x)
        view_pos=logic_boards_view[x]
        height_pos=logic_boards_depth[x]
        
        view_read,height_read=board_message
        movecoppy=0
        rospy.loginfo("expected board"+str(view_pos))
        rospy.loginfo("percieved board"+str(view_read))

        if(x==0):
            movecoppy=possible_moves_class[0]
            probabileties.append(compare_probabilety8(view_pos,view_read,height_pos,height_read,boardcolor_confidence,movecoppy))
        else:
            movecoppy=possible_moves_class[x-1]
            probabileties.append(compare_probabilety7(view_pos,view_read,height_pos,height_read,boardcolor_confidence,movecoppy))
        
        
    # loop over the logic_boards where [0] is the curent based on complete moves
    #for i in np.flip(range(len(logic_boards))): # starting from last in order chek the active board last
     #   if(compare(board_message,logic_boards[i])):
      #      boardtrue=i #return the id if it matches a board in the logic board
       #     break
    
    maxval=np.max(probabileties)
    maxid=np.argmax(probabileties)
    



    
    if(maxval>0.4 and maxid!=0): # if 0 then nothing has changed, if -1 then a pice is misplaced, 
        # if it is above 0 we move the posible id to the complete moves 
        completed_moves_list.append(posible_moves_list[maxid-1])
        rospy.loginfo("move" + str(posible_moves_list[maxid-1])+" was loged as complete")
    rospy.logerr("somthing went wrong, perhaps a wrong move was made")
    rospy.logwarn("each move had a "+str(probabileties))
    rospy.logwarn("possible moves are "+str(posible_moves_list))
    rospy.logwarn("completed moves are "+str(completed_moves_list))
    rospy.logwarn(" retived ids were "+ str(maxid)+" and had a probabilety of "+str(maxval))
    
    if(boardtrue==-1):
        rospy.logerr("somthing went wrong, perhaps a wrong move was made")
        rospy.logwarn("each move had a "+str(probabileties))
        rospy.logwarn("possible moves are "+str(posible_moves_list))
        rospy.logwarn("completed moves are "+str(completed_moves_list))


    posible_moves_list=find_next_steps(completed_moves_list,moves)

def find_next_steps(completed_steps,moves):
    
    # Convert steps into a dictionary for easy lookup
    steps_dict = {}
    for line in moves:
        
        step_id = line.tempid
        dependencies = line.pre_requisites
        steps_dict[step_id] = {'dependencies': dependencies, 'completed': False}

    # Find all possible next steps
    possible_next_steps = []
    for step_id, step_data in steps_dict.items():
        if not step_data['completed'] and (step_data['dependencies'] == [-1] or all(dep in completed_steps for dep in step_data['dependencies'])):
            possible_next_steps.append(step_id)

    # Filter out completed steps
    possible_next_steps = [step_id for step_id in possible_next_steps if step_id not in completed_steps]
    global possible_moves_class
    possible_moves_class=[]
    for x in possible_next_steps:
        for y in range(len(moves)):
            if(moves[y].tempid==x):
                possible_moves_class.append(moves[y])
                break
    rospy.logerr(possible_next_steps)
    return possible_next_steps

availible_moves_sub=0
if __name__ == '__main__':
    rospy.init_node("movecheker") #this node isolates blocks 
    #clearcheksub= rospy.Subscriber("/info/buildplate/obstruction ",bool) # returns true when nither the human nor the robot are near are obstructing the build plate
    availible_moves_sub=rospy.Subscriber("/data/boardview/",blockdata,callback=valid_board_scan)
    moves_initialize()
    rospy.loginfo("started waiting for callbacks")
    rospy.spin()
    



    
    