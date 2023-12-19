#!/usr/bin/env python3
import rospy
import numpy as np
import cv2 as cv
import cv_bridge 
from custom_msg_python.msg import allbrickslist, blockdata
from sensor_msgs.msg import Image,CompressedImage
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Pose2D
import boardprosesing as bp

# to do, find out which datatype the k4a rosnode topics are 
bridge=cv_bridge.CvBridge()

#Creates a mask that removes everything, but the area marked of for the pile of lego bricks.
def find_pile(img):
    cache = img.copy()
    iout=cv.cvtColor(img,cv.COLOR_RGB2BGR)
    blur = cv.bilateralFilter(iout, 9, 75, 75)
    hsv = cv.cvtColor(blur, cv.COLOR_BGR2HSV)
    #cv.imshow('frame', hsv);cv.waitKey()

    white_max = np.array([255, 255, 255]);white_min = np.array([200, 200, 200])

    threshold = cv.inRange(blur, white_min, white_max)
    #cv.imshow('frame', threshold);cv.waitKey()
    kernel = np.ones((3, 3), np.uint8)
    erosion = cv.erode(threshold, kernel, iterations=3)
    dilation = cv.dilate(erosion, kernel, iterations=3)
    edges = cv.Canny(dilation, 50, 200)
    #cv.imshow('frame', edges);cv.waitKey()
    contours, hierarchy = cv.findContours(edges, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)
    # print(len(contours))
    #debugimg(edges)

    for con in range(len(contours)):
        cnt = contours[con]
        perimeter = cv.arcLength(cnt, True)

        if 2100>perimeter>1800: #Checks all contours to find the one that corresponds to the tape, that marks the pile area.
            rect = cv.minAreaRect(cnt) # Makes a rectangle that contains all the other most points of the contour, the minimum area rectangle.
            box = cv.boxPoints(rect) # Finds corner points of rectangle.
            box = np.int0(box)
            ed = cv.drawContours(img, [box], 0, (0, 0, 255), 2)
            filled = cv.fillConvexPoly(ed, np.array([box], 'int32'), (0, 0, 255))
            #cv.imshow('frame', filled);cv.waitKey()
            red = np.array([0, 0, 255])
            rospy.loginfo(filled.shape)
            mask = cv.inRange(filled, red, red)
            #cv.imshow('frame', mask);cv.waitKey()
            result = cv.bitwise_and(cache, cache, mask=mask)
            #cv.imshow('frame', result);cv.waitKey(0)
            break

    return result
    

def lego_pile_recognition(img):
      # The array of colors go blue, then red, then yellow.
    # Categories is the big array used to send the needed information on. It has several arrays inside it:
    # Categories --> array for each color --> array for short and long bricks--> array for each brick of that size and color --> has the midpoint and angle of the brick.
    bluelong=[];blueshort=[];redlong=[];redshort=[];yellowlong=[];yellowshort=[]
    categories =[[bluelong,blueshort],[redlong,redshort],[yellowlong,yellowshort]]

    color = 0 # For appending into color arrays
    size = 0 # For appending into size arrays

    resize = find_pile_new(img) #Applies a mask to the image to only look at the pile.
    #debugimg(resize)
    iout=cv.cvtColor(resize,cv.COLOR_RGB2BGR)
    blur = cv.bilateralFilter(iout, 9, 75, 75)
    #cv.imshow('original', blur); cv.waitKey()

    # Thresholds
    #blue_max = np.array([255, 215, 140]);red_max = np.array([195, 190, 255]);yellow_max = np.array([230, 255, 255])
    #blue_min = np.array([190, 120, 50]);red_min = np.array([130, 130, 220]);yellow_min = np.array([140, 220, 220])
    
    #blue_max = np.array([140, 215, 255]);red_max = np.array([255, 190, 195]);yellow_max = np.array([255, 255, 230])
    #blue_min = np.array([50, 120, 190]);red_min = np.array([220, 130, 130]);yellow_min = np.array([220, 220, 140])
    
    #blue_max = np.array([255, 180, 140]);red_max = np.array([155, 170, 255]);yellow_max = np.array([230, 255, 255])
    #blue_min = np.array([175, 90, 40]);red_min = np.array([55, 70, 190]);yellow_min = np.array([140, 220, 220])
    
    #blue_max = np.array([140, 180, 255]);red_max = np.array([255, 170, 155]);yellow_max = np.array([255, 255, 230])
    #blue_min = np.array([40, 90, 175]);red_min = np.array([190, 70, 55]);yellow_min = np.array([220, 220, 140])

    blue_max = np.array([142, 215, 255]);red_max = np.array([255, 190, 195]);yellow_max = np.array([255, 255, 230])
    blue_min = np.array([38, 90, 171]);red_min = np.array([190, 70, 55]);yellow_min = np.array([211, 220, 106])


    threshold_max = [blue_max, red_max, yellow_max]
    threshold_min = [blue_min, red_min, yellow_min]

    for i,j in zip(threshold_max, threshold_min): #Runs for every pair of thresholds.

        threshold= cv.inRange(blur, np.array(j), np.array(i)) #thresholds for each color brick.
        #cv.imshow('frame', threshold);cv.waitKey()

        kernel = np.ones((3, 3), np.uint8)
        erosion = cv.erode(threshold, kernel, iterations=2) #Opening to remove noise.
        dilation = cv.dilate(erosion, kernel, iterations=2)

        edges = cv.Canny(dilation, 50, 200)
        #cv.imshow('frame', edges);cv.waitKey()
        #debugimg(edges)
        contours, hierarchy = cv.findContours(edges, cv.RETR_CCOMP, cv.CHAIN_APPROX_SIMPLE)

        for con in range(len(contours)): #Runs through all contours for each color
            cnt = contours[con]
            if hierarchy[0][con][2] < 0:  #Ensures non-closed contours are not looked at, since they can not be bricks.
                rospy.logwarn('This contour is not closed')
            elif len(contours[con])<5:
                rospy.logwarn('Too small')
            else:
                (x, y), (major, minor), angle = cv.fitEllipse(cnt) #Finds the midpoint and angle of the bricks.
                #print((x, y), angle, (major, minor))
                perimeter = cv.arcLength(cnt, True) #Finds perimeter of contour.
                #print(perimeter)
                brick = [[x, y], [angle]]

                # Sorts by size of perimeter and dimensions of the contour
                if 125 < perimeter < 160 and 20 < major < 40 and 47 < minor < 65:
                    categories[color][size].append(brick)
                elif 75 < perimeter < 110 and 20 < major < 40 and 23 < minor < 40:
                    categories[color][size + 1].append(brick)
                else:
                    rospy.logwarn('This is not a Lego Brick')

        color += 1 # Changes color array that is appended Blue --> Red --> Yellow

        #print(categories)
    return categories


def lego_pile_recognition_draw(img):
    # The array of colors go blue, then red, then yellow.
    # Categories is the big array used to send the needed information on. It has several arrays inside it:
    # Categories --> array for each color --> array for short and long bricks--> array for each brick of that size and color --> has the midpoint and angle of the brick.
    bluelong=[];blueshort=[];redlong=[];redshort=[];yellowlong=[];yellowshort=[]
    categories =[[bluelong,blueshort],[redlong,redshort],[yellowlong,yellowshort]]

    drawcolors=[(0,0,255),(0,255,255),(0,255,0),(255,255,0),(255,0,0),(255,0,255)]

    color = 0 # For appending into color arrays
    size = 0 # For appending into size arrays

    # Picture preprocessing
    resize = find_pile_new(img) #Applies a mask to the image to only look at the pile.
    
    #print(len(resize))
    if type(resize) == int:
        return
    if len(resize) == 1 :
        print('No pile, something is in the way!')
        return
    cache = resize.copy()
    blur = cv.bilateralFilter(resize, 9, 75, 75)
    #cv.imshow('frame', blur); cv.waitKey()

    blue_max = np.array([255, 215, 142]);red_max = np.array([195, 190, 255]);yellow_max = np.array([230, 255, 255])
    blue_min = np.array([171, 90, 38]);red_min = np.array([55, 70, 190]);yellow_min = np.array([106, 220, 211])

    threshold_max = [blue_max, red_max, yellow_max]
    threshold_min = [blue_min, red_min, yellow_min]

    for i,j in zip(threshold_max, threshold_min): #Runs for every pair of thresholds.

        threshold= cv.inRange(blur, np.array(j), np.array(i)) #thresholds for each color brick.
        #cv.imshow('frame', threshold);cv.waitKey()

        kernel = np.ones((3, 3), np.uint8)
        erosion = cv.erode(threshold, kernel, iterations=2) #Opening to remove noise.
        dilation = cv.dilate(erosion, kernel, iterations=2)

        edges = cv.Canny(dilation, 50, 200)
        #cv.imshow('frame', edges);cv.waitKey()
        contours, hierarchy = cv.findContours(edges, cv.RETR_CCOMP, cv.CHAIN_APPROX_SIMPLE)

        for con in range(len(contours)): #Runs through all contours for each color
            cnt = contours[con]
            if hierarchy[0][con][2] < 0:  #Ensures non-closed contours are not looked at, since they can not be bricks.
                print('This contour is not closed')
            elif len(contours[con])<5:
                print('Too small')
            else:
                (x, y), (major, minor), angle = cv.fitEllipse(cnt) #Finds the midpoint and angle of the bricks.
                #print((x, y), angle, (major, minor))
                x = int(x-10)
                y = int(y+40)
                center = (x, y)
                angle=str(int(angle))

                perimeter = cv.arcLength(cnt, True) #Finds perimeter of contour.
                #print(perimeter)
                rect = cv.minAreaRect(cnt)
                box = cv.boxPoints(rect)
                #print(box)
                boxt = np.int0(box)

                brick = [[x, y], [angle]]

                # Sorts by size of perimeter and dimensions of the contour
                if 125 < perimeter < 160 and 20 < major < 45 and 47 < minor < 65:
                    categories[color][size].append(brick)
                    cv.drawContours(resize, [boxt], 0, drawcolors[color], 2)
                    length=cv.norm(box[0],box[1])
                    if length> 35:
                        p1x = int((box[1][0] + box[2][0]) / 2);p1y = int((box[1][1] + box[2][1]) / 2)
                        p2x = int((box[3][0] + box[0][0]) / 2);p2y = int((box[3][1] + box[0][1]) / 2)
                    else:
                        p1x = int((box[0][0] + box[1][0]) / 2);p1y = int((box[0][1] + box[1][1]) / 2)
                        p2x = int((box[2][0] + box[3][0]) / 2);p2y = int((box[2][1] + box[3][1]) / 2)
                    line = cv.line(resize, (p1x,p1y), (p2x,p2y), drawcolors[color], 2)
                    cv.putText(resize,angle,center,cv.FONT_HERSHEY_PLAIN,2,(0,0,255),2,cv.LINE_AA)
                elif 75 < perimeter < 110 and 20 < major < 45 and 20 < minor < 45:
                    #print('small')
                    categories[color][size + 1].append(brick)
                    cv.drawContours(resize, [boxt], 0, drawcolors[color+3], 2)
                    p1x = int((box[1][0] + box[2][0]) / 2);p1y = int((box[1][1] + box[2][1]) / 2)
                    p2x = int((box[3][0] + box[0][0]) / 2);p2y = int((box[3][1] + box[0][1]) / 2)
                    line = cv.line(resize, (p1x,p1y), (p2x,p2y), drawcolors[color+3], 2)
                    cv.putText(resize, angle, center, cv.FONT_HERSHEY_PLAIN, 2, (0, 0, 255), 2, cv.LINE_AA)
                else:
                    print('This is not a Lego Brick')

        color += 1 # Changes color array that is appended Blue --> Red --> Yellow
        #cv.imshow('frame', line);cv.waitKey()

        #print(categories)
    return resize


def lego_pile_recognition_sort(img):
        # The array of colors go blue, then red, then yellow.
    # Categories is the big array used to send the needed information on. It has several arrays inside it:
    # Categories --> array for each color --> array for short and long bricks--> array for each brick of that size and color --> has the midpoint and angle of the brick.
    bluelong=[];blueshort=[];redlong=[];redshort=[];yellowlong=[];yellowshort=[]
    categories =[[bluelong,blueshort],[redlong,redshort],[yellowlong,yellowshort]]

    color = 0 # For appending into color arrays
    size = 0 # For appending into size arrays

    # Picture preprocessing
    resize = find_pile_new(img) #Applies a mask to the image to only look at the pile.
    #print(len(resize))
    if type(resize) == int:
        return 0
    if len(resize) == 1 :
        print('No pile, something is in the way!')
        return 0
    blur = cv.bilateralFilter(resize, 9, 75, 75)
    #cv.imshow('frame', blur); cv.waitKey()

    # Thresholds (in BGR)
    #Early day thresholds
    #blue_max = np.array([255, 215, 140]);red_max = np.array([195, 190, 255]);yellow_max = np.array([230, 255, 255])
    #blue_min = np.array([190, 120, 50]);red_min = np.array([130, 130, 220]);yellow_min = np.array([140, 220, 220])
    # Midday thresholds
    #blue_max = np.array([255, 206, 142]);red_max = np.array([166, 174, 255]);yellow_max = np.array([208, 255, 255])
    #blue_min = np.array([171, 102, 38]);red_min = np.array([102, 111, 211]);yellow_min = np.array([106, 227, 211])
    #Later day thresholds
    #blue_max = np.array([255, 180, 140]);red_max = np.array([155, 170, 255]);yellow_max = np.array([230, 255, 255])
    #blue_min = np.array([175, 90, 40]);red_min = np.array([55, 70, 190]);yellow_min = np.array([140, 220, 220])

    blue_max = np.array([255, 215, 142]);red_max = np.array([195, 190, 255]);yellow_max = np.array([230, 255, 255])
    blue_min = np.array([171, 90, 38]);red_min = np.array([55, 70, 190]);yellow_min = np.array([106, 220, 211])

    threshold_max = [blue_max, red_max, yellow_max]
    threshold_min = [blue_min, red_min, yellow_min]

    for i,j in zip(threshold_max, threshold_min): #Runs for every pair of thresholds.

        threshold= cv.inRange(blur, np.array(j), np.array(i)) #thresholds for each color brick.
        #cv.imshow('frame', threshold);cv.waitKey()

        kernel = np.ones((3, 3), np.uint8)
        erosion = cv.erode(threshold, kernel, iterations=2) #Opening to remove noise.
        dilation = cv.dilate(erosion, kernel, iterations=2)

        edges = cv.Canny(dilation, 50, 200)
        #cv.imshow('frame', edges);cv.waitKey()
        contours, hierarchy = cv.findContours(edges, cv.RETR_CCOMP, cv.CHAIN_APPROX_SIMPLE)

        for con in range(len(contours)): #Runs through all contours for each color
            cnt = contours[con]
            if hierarchy[0][con][2] < 0:  #Ensures non-closed contours are not looked at, since they can not be bricks.
                print('This contour is not closed')
            elif len(contours[con])<5:
                print('Too small')
            else:
                (x, y), (major, minor), angle = cv.fitEllipse(cnt) #Finds the midpoint and angle of the bricks.
                #print((x, y), angle, (major, minor))
                perimeter = cv.arcLength(cnt, True) #Finds perimeter of contour.
                #print(perimeter)
                brick = [[x, y], [angle]]

                # Sorts by size of perimeter and dimensions of the contour
                if 125 < perimeter < 160 and 20 < major < 45 and 47 < minor < 65:
                    categories[color][size].append(brick)
                elif 75 < perimeter < 110 and 20 < major < 45 and 20 < minor < 45:
                    categories[color][size + 1].append(brick)
                else:
                    print('This is not a Lego Brick')

        color += 1 # Changes color array that is appended Blue --> Red --> Yellow

        print(categories)
    return categories


def find_pile_new(img):
    cache = img.copy()
    blur = cv.bilateralFilter(img, 9, 75, 75)
    #cv.imshow('frame', hsv);cv.waitKey()

    white_max = np.array([255, 255, 255]);white_min = np.array([205, 205, 205])

    threshold = cv.inRange(blur, white_min, white_max)
    #cv.imshow('frame', threshold);cv.waitKey()
    kernel = np.ones((3, 3), np.uint8)
    erosion = cv.erode(threshold, kernel, iterations=3)
    dilation = cv.dilate(erosion, kernel, iterations=3)
    edges = cv.Canny(dilation, 50, 200)
    #cv.imshow('frame', edges);cv.waitKey()
    contours, hierarchy = cv.findContours(edges, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)
    # print(len(contours))
    count = 0


    for con in range(len(contours)):
        cnt = contours[con]
        perimeter = cv.arcLength(cnt, True)
        #print(perimeter)

        if len(contours[con]) > 5: # only for testing since real program will have mask
            (x, y), (major, minor), angle = cv.fitEllipse(cnt)
        if 2100>perimeter>1800 and 620>major>580 and 710>minor>680: #Checks all contours to find the one that corresponds to the tape, that marks the pile area.
            rect = cv.minAreaRect(cnt) # Makes a rectangle that contains all the other most points of the contour, the minimum area rectangle.
            box = cv.boxPoints(rect) # Finds corner points of rectangle.
            box = np.int0(box)
            ed = cv.drawContours(img, [box], 0, (0, 0, 255), 2)
            filled = cv.fillConvexPoly(ed, np.array([box], 'int32'), (0, 0, 255))
            #cv.imshow('frame', filled);cv.waitKey()
            red = np.array([0, 0, 255])
            mask = cv.inRange(filled, red, red)
            #cv.imshow('frame', mask);cv.waitKey()
            result = cv.bitwise_and(cache, cache, mask=mask)
            #cv.imshow('frame', result);cv.waitKey(0)
            count = 1
            break
    if count == 0:
        return count
    return result
def allblockrecognision(img):
     # The array of colors go blue, then red, then yellow.
    # Categories is the big array used to send the needed information on. It has several arrays inside it:
    # Categories --> array for each color --> array for short and long bricks--> array for each brick of that size and color --> has the midpoint and angle of the brick.
    bluelong=[];blueshort=[];redlong=[];redshort=[];yellowlong=[];yellowshort=[]
    categories =[[bluelong,blueshort],[redlong,redshort],[yellowlong,yellowshort]]

    color = 0 # For appending into color arrays
    size = 0 # For appending into size arrays

    # Picture preprocessing
    # Picture preprocessing
    resize = img
    #rospy.loginfo(type(img))
    blur = cv.bilateralFilter(resize, 9, 75, 75)
    hsv = cv.cvtColor(blur, cv.COLOR_BGR2HSV)

    # Thresholds
    blue_max = np.array([120, 190, 255]);red_max = np.array([19, 218, 255]);yellow_max = np.array([40, 220, 255])
    blue_min = np.array([107, 80, 190]);red_min = np.array([0, 50, 187]);yellow_min = np.array([15, 0, 125])
    threshold_max = [blue_max, red_max, yellow_max]
    threshold_min = [blue_min, red_min, yellow_min]

    for i,j in zip(threshold_max, threshold_min): #Runs for every pair of thresholds.

        threshold= cv.inRange(hsv, np.array(j), np.array(i)) #thresholds for each color brick.
      #  cv.imshow('frame', threshold);cv.waitKey()

        kernel = np.ones((3, 3), np.uint8)
        erosion = cv.erode(threshold, kernel, iterations=2) #Opening to remove noise.
        dilation = cv.dilate(erosion, kernel, iterations=2)

        edges = cv.Canny(dilation, 50, 200)
       # cv.imshow('frame', edges);cv.waitKey()
        contours, hierarchy = cv.findContours(edges, cv.RETR_CCOMP, cv.CHAIN_APPROX_SIMPLE)

        for con in range(len(contours)): #Runs through all contours for each color
            cnt = contours[con]
            if hierarchy[0][con][2] < 0:  #Ensures non-closed contours are not looked at, since they can not be bricks.
                rospy.loginfo('This contour is not closed')
            else:
                (x, y), (major, minor), angle = cv.fitEllipse(cnt) #Finds the midpoint and angle of the bricks.
        #        print((x, y), angle, (major, minor))
                perimeter = cv.arcLength(cnt, True) #Finds perimeter of contour.
         #       print(perimeter)
                brick = [[x, y], [angle]]

                # Sorts by size of perimeter and dimensions of the contour
                if 125 < perimeter < 160 and 20 < major < 45 and 47 < minor < 65:
                    categories[color][size].append(brick)
                elif 75 < perimeter < 110 and 20 < major < 45 and 20 < minor < 45:
                    categories[color][size + 1].append(brick)
                else:
                    print('This is not a Lego Brick')

        color += 1 # Changes color array that is appended Blue --> Red --> Yellow

        rospy.loginfo(categories)
    return categories



active=False
gotmask=False
def imagelinker(msg): # this function finds availible blocks in the draw pile
    
    global active
    global gotmask

    if(gotmask==True):
        if(active==False):
            active=True

            print("recived image")
            global imagebrickpublisher
            img=bridge.compressed_imgmsg_to_cv2(msg)
            # read the image
    
            #cv.imshow('frame', binary_mask);cv.waitKey()
            global mask_plane
            if(mask_plane==[]):
                return

            if(False):
                imagedata= cv.bitwise_and(img, img, mask=mask_cutof)
                bricks=allblockrecognision(imagedata) # scan the entire workspace 
            else:        
       
                imagedata= cv.bitwise_and(img, img, mask=mask_plane)
                bricks=lego_pile_recognition_sort(imagedata)
                if type(bricks) != int: 
                    brick_publish_message=messagemakerallbrick( bricks)
                    imagebrickpublisher.publish(brick_publish_message) # publisg the allblockdata message
                    rospy.loginfo("sent a bricklist")
                # calculate the board state
                global depthmap
                rgbmap,depthmp,miny,minx,xpix,ypix=bp.board_brick_recognition(imagedata,depthmap)
        
        
                img20x20=img[0:20,0:20]
                #debugboard(img20x20,rgbmap)
                try:
                    boardmsg=boardview_message_maker(rgbmap,depthmp,miny,minx,xpix,ypix)
                    imageboardpublisher.publish(boardmsg)
                    rospy.loginfo("sent a boardimage")
                except:
                    rospy.logwarn(type(rgbmap))
                
                active=False
                gotmask=False

        
        #debugimg(bp.debugreturn)
    else:
        rospy.loginfo("ignored an image")


         
    
   
    


def messagemakerallbrick(array):
    foundbricks=0
    lengths=[]
    msg= allbrickslist()
    x=array[0]
    small_list=[]
    for y in x[0]:
        pose= Pose2D()
        pose.x=y[0][0]
        pose.y=y[0][1]
        pose.theta=y[1][0]
        small_list.append(pose)
        foundbricks+=len(small_list)
        lengths.append(len(small_list))
    large_list=[]
    for y in x[1]:
        pose= Pose2D()
        pose.x=y[0][0]
        pose.y=y[0][1]
        pose.theta=y[1][0]
        large_list.append(pose)
        foundbricks+=len(large_list)
        lengths.append(len(large_list))
    msg.blue_small=small_list
    msg.blue_large=large_list
    x=array[1]
    small_list=[]
    for y in x[0]:
        pose= Pose2D()
        pose.x=y[0][0]
        pose.y=y[0][1]
        pose.theta=y[1][0]
        small_list.append(pose)
        foundbricks+=len(small_list)
        lengths.append(len(small_list))
    large_list=[]
    for y in x[1]:
        pose= Pose2D()
        pose.x=y[0][0]
        pose.y=y[0][1]
        pose.theta=y[1][0]
        large_list.append(pose)
        foundbricks+=len(large_list)
        lengths.append(len(large_list))
    msg.red_small=small_list
    msg.red_large=large_list
    x=array[2]
    small_list=[]
    for y in x[0]:
        pose= Pose2D()
        pose.x=y[0][0]
        pose.y=y[0][1]
        pose.theta=y[1][0]
        small_list.append(pose)
        foundbricks+=len(small_list)
        lengths.append(len(small_list))
    large_list=[]
    for y in x[1]:
        pose= Pose2D()
        pose.x=y[0][0]
        pose.y=y[0][1]
        pose.theta=y[1][0]
        large_list.append(pose)
        foundbricks+=len(large_list)
        lengths.append(len(large_list))
    msg.yelow_small=small_list
    msg.yelow_large=large_list
    rospy.loginfo("found "+str(foundbricks)+" bricks in the photo")
    rospy.loginfo("found "+str(lengths)+" list had shape")
    

    #blå rød gul grøn grå
    return msg

def boardview_message_maker(rgb,depth,miny:int,minx:int,xpix:int,ypix:int):
    #print("messageboards")
    #print(rgb)
    #print(depth)
    
    message=blockdata()
    fc=[]
    fd=[]
    for x in range(len(rgb)):
        fc.append(int(rgb[x]))
    for x in range(len(depth)):
        fd.append(int(depth[x]))

    message.fieldcolors=fc
    message.fieldheight=fd

    message.height=20
    message.width=20
    message.x_origen=int(minx)
    message.y_origen=int(miny)
    message.x_pix_pr_cell=int(xpix)
    message.y_pix_pr_cell=int(ypix)
    return message 
mask_plane=[]
def maskupdate_plane(msg):
    global mask_plane
    global active
    global gotmask
    if(active==False and gotmask==False):
    
        mask = bridge.compressed_imgmsg_to_cv2(msg)
        thresh_max = np.array([255, 255, 255]);thresh_min = np.array([1, 1, 1])
        mask_plane = cv.inRange(mask, 1, 255)
        gotmask=True
    
    

mask_cutof=[]

def maskupdate_cutof(msg):
    global mask_cutof
    global configurationmode
    mask = bridge.compressed_imgmsg_to_cv2(msg)
    thresh_max = np.array([255, 255, 255]);thresh_min = np.array([1, 1, 1])
    mask_cutof = cv.inRange(mask, 1, 255)

depthmap=[]
def depthmapupdate(msg):
    
    global depthmap
    depthmap = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='passthrough')
    #depthmap = np.array(deptimg, dtype=np.float32)
    print(depthmap[0,0])
    
def debugboard(img,board):
    image=img
    colors=[[0,255,0],[0,0,255],[255,0,0,],[255,255,0]]
    for x in range(20):
        for y in range(20):
            #print(board[int(foldxy(x,y,20,20))])
           


            col=colors[int(board[int(foldxy(x,y,20,20))])]
            image[x,y][0]=col[0]
            image[x,y][1]=col[1]
            image[x,y][2]=col[2]
    debugimg(image)



def debugimg(img):
    global debugpub
    image_message = bridge.cv2_to_imgmsg(img, encoding="bgr8")
    debugpub.publish(image_message)

def foldxy(x,y,xlen,ylen):
    return(x+y*xlen)


    


    



curimage=[]
cur_mask_plane=[]
cur_mask_cutof=[]

cameralistner_rgb=0
cameralistner_depth=0
masklistner1=0
masklistner2=0

imagebrickpublisher=0
debugpub=0
configurationmode=True
imageboardpublisher=0
if __name__ == '__main__':
    rospy.init_node("blockfinder_node") #this node isolates blocks 

    rospy.loginfo("i have begun")
    cameralistner_rgb=rospy.Subscriber("/rgb/image_raw/compressed",CompressedImage,queue_size=1,callback=imagelinker)
    cameralistner_depth=rospy.Subscriber("/depth_to_rgb/image_raw/compressed",CompressedImage,callback=depthmapupdate)
    
    imagebrickpublisher=rospy.Publisher("/data/imagebricks/",allbrickslist,queue_size=1)
    imageboardpublisher=rospy.Publisher("/data/boardview/",blockdata,queue_size=1)
    
    masklistner1=rospy.Subscriber("/binary/masks/table_plane",CompressedImage,callback=maskupdate_plane)
   # masklistner2=rospy.Subscriber("/binary/masks/table_cutoff",CompressedImage,callback=maskupdate_cutof)

    debugpub=rospy.Publisher("rgb/image/debug",Image,queue_size=1)
    

    rospy.sleep(1)
    rospy.loginfo("started waiting for updates")
    rospy.spin()


#This part is used for testing.

#Resizes window to make it easier to see images when using imshow.
