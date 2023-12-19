#!/usr/bin/env python3 

import cv2
import mediapipe as mp
import os
import numpy as np
import rospy 
from cv_bridge import CvBridge
from sensor_msgs.msg import PointCloud2, Image
from std_msgs.msg import Header
from custom_msg_python.msg import custom2

bride=CvBridge()
mp_pose=mp.solutions.pose
mp_drawing=mp.solutions.drawing_utils

first=True

def imgget(msg):
    global imglast
    global imgnow
    global first
    global armmask
    if(first):
        imglast=bride.imgmsg_to_cv2(msg)
        first=False
        

    
    imgnow=bride.imgmsg_to_cv2(msg)

# Setup mediapipe
    with mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5) as pose:
        if(True):
            # Read the images
            
            frame = imgnow

            
            image1 = imglast

        # Recolor image to RGB
            image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            image
            image.flags.writeable = False

        # Make detection
            results = pose.process(image)

        # Recolor back to BGR
            image.flags.writeable = True
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            armmask.publish(bride.cv2_to_imgmsg( image))

        # Extract landmarks, if it tracks anything
            try:
                landmarks = results.pose_landmarks.landmark

            # Eetting the right coords
                WristCx = int(landmarks[mp_pose.PoseLandmark.LEFT_WRIST.value].x * image.shape[1])
                WristCy = int(landmarks[mp_pose.PoseLandmark.LEFT_WRIST.value].y * image.shape[0])
                ElbowCx = int(landmarks[mp_pose.PoseLandmark.LEFT_ELBOW.value].x * image.shape[1])
                ElbowCy = int(landmarks[mp_pose.PoseLandmark.LEFT_ELBOW.value].y * image.shape[0])
            
                PinkyCx = int(landmarks[mp_pose.PoseLandmark.LEFT_PINKY.value].x * image.shape[1])
                PinkyCy = int(landmarks[mp_pose.PoseLandmark.LEFT_PINKY.value].y * image.shape[0])
                IndexCx = int(landmarks[mp_pose.PoseLandmark.LEFT_INDEX.value].x * image.shape[1])
                IndexCy = int(landmarks[mp_pose.PoseLandmark.LEFT_INDEX.value].y * image.shape[0])

            # Putting the x&y coords in arrays
                WristXY = [WristCx, WristCy]
                ElbowXY = [ElbowCx, ElbowCy]
            
                PinkyXY = [PinkyCx, PinkyCy]
                IndexXY = [IndexCx, IndexCy]

            # setting the box size in pixels
                box_size = (250, 250)

            # Calculate the starting point for the region of interest (ROI) for WristXY
                wrist_roi_start_x = max(0, WristXY[0] - box_size[0] // 2)
                wrist_roi_start_y = max(0, WristXY[1] - box_size[1] // 2)

            # Define the region of interest (ROI) for WristXY based on the box size
                wrist_roi = image1[wrist_roi_start_y:wrist_roi_start_y + box_size[1], wrist_roi_start_x:wrist_roi_start_x + box_size[0]]

                # Resize the overlay image to match the box size and creating the mask
                resized_wrist_overlay = cv2.resize(wrist_roi, (box_size[0], box_size[1]))
                wrist_mask = resized_wrist_overlay[:, :, 3] / 255.0 if resized_wrist_overlay.shape[2] == 4 else None

                # Blend the images using the mask for the X&Y and updating the blended roi with the X&Y
                blended_wrist_roi = cv2.addWeighted(wrist_roi, 1 - wrist_mask, resized_wrist_overlay[:, :, :3], wrist_mask, 0) if wrist_mask is not None else wrist_roi
                image[wrist_roi_start_y:wrist_roi_start_y + box_size[1], wrist_roi_start_x:wrist_roi_start_x + box_size[0]] = blended_wrist_roi

                # doing the same
                elbow_roi_start_x = max(0, ElbowXY[0] - box_size[0] // 2)
                elbow_roi_start_y = max(0, ElbowXY[1] - box_size[1] // 2)

                # Define the region of interest (ROI) for ElbowXY based on the box size
                elbow_roi = image1[elbow_roi_start_y:elbow_roi_start_y + box_size[1], elbow_roi_start_x:elbow_roi_start_x + box_size[   0]]

                # Resize the overlay image to match the box size
                resized_elbow_overlay = cv2.resize(elbow_roi, (box_size[0], box_size[1]))

                # Create a mask using the alpha channel of the overlay image (if exists)
                elbow_mask = resized_elbow_overlay[:, :, 3] / 255.0 if resized_elbow_overlay.shape[2] == 4 else None

            # Blend the images using the mask for ElbowXY
                blended_elbow_roi = cv2.addWeighted(elbow_roi, 1 - elbow_mask, resized_elbow_overlay[:, :, :3], elbow_mask, 0) if elbow_mask is not None else elbow_roi

            # Update the frame with the blended ROI for ElbowXY
                image[elbow_roi_start_y:elbow_roi_start_y + box_size[1], elbow_roi_start_x:elbow_roi_start_x + box_size[0]] = blended_elbow_roi

                pinky_roi_start_x = max(0, PinkyXY[0] - box_size[0] // 2)
                pinky_roi_start_y = max(0, PinkyXY[1] - box_size[1] // 2)

                pinky_roi = image1[pinky_roi_start_y:pinky_roi_start_y + box_size[1], pinky_roi_start_x:pinky_roi_start_x + box_size[0]]
            
                resized_pinky_overlay = cv2.resize(pinky_roi, (box_size[0], box_size[1]))
            
                pinky_mask = resized_pinky_overlay[:, :, 3] / 255.0 if resized_pinky_overlay.shape[2] == 4 else None
            
                blended_pinky_roi = cv2.addWeighted(pinky_roi, 1 - pinky_mask, resized_pinky_overlay[:, :, :3], pinky_mask, 0) if pinky_mask is not None else pinky_roi
            
                image[pinky_roi_start_y:pinky_roi_start_y + box_size[1], pinky_roi_start_x:pinky_roi_start_x + box_size[0]] = blended_pinky_roi
            
                index_roi_start_x = max(0, IndexXY[0] - box_size[0] // 2)
                index_roi_start_y = max(0, IndexXY[1] - box_size[1] // 2)

                index_roi = image1[index_roi_start_y:index_roi_start_y + box_size[1], index_roi_start_x:index_roi_start_x + box_size[0]]
            
                resized_index_overlay = cv2.resize(index_roi, (box_size[0], box_size[1]))
            
                index_mask = resized_index_overlay[:, :, 3] / 255.0 if resized_index_overlay.shape[2] == 4 else None
            
                blended_index_roi = cv2.addWeighted(index_roi, 1 - index_mask, resized_index_overlay[:, :, :3], index_mask, 0) if index_mask is not None else index_roi
            
                image[index_roi_start_y:index_roi_start_y + box_size[1], index_roi_start_x:index_roi_start_x + box_size[0]] = blended_index_roi
            
            except:
                pass

        # Render detections
            mp_drawing.draw_landmarks(image, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)

        # Displays the image
            #armmask.publish(bride.cv2_to_imgmsg( image))

        

        


    
armmask=0



if __name__ == '__main__':
    rospy.init_node("armmasker_node")

    rgbascriber=rospy.Subscriber("/rgb/image_raw",Image,callback=imgget)

    armmask=rospy.Publisher("/data/arm/mask",Image,queue_size=1)
    
    armposesetter=rospy.Publisher("/data/arm/position",custom2,queue_size=1)

    # maby shift these to the compresed versions as reading raw images consumes about 3 mb of ram pr image
    


    rospy.spin()



mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose

## Directory containing the first set of images
image_directory = '/home/nicolaj/Desktop/Images/Cam-Input'

# Directory containing the second set of images
image1_directory = '/home/nicolaj/Desktop/Images'

# List all image files in the first directory
image_files = [f for f in os.listdir(image_directory) if f.endswith(('.jpg', '.jpeg', '.png'))]

# List all image files in the second directory
image1_files = [f for f in os.listdir(image1_directory) if f.endswith(('.jpg', '.jpeg', '.png'))]

imglast=[]
imgnow=[]


cv2.destroyAllWindows()
