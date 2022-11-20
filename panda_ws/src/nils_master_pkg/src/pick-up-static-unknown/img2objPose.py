#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from math import atan2, cos, sin, sqrt, pi
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Twist 
from gaussian import *

class Image2Object(object):
    def __init__(self):
        # Params
        self.br = CvBridge()
        self.loop_rate = rospy.Rate(10)
        self.printed = False

        #Object detector
        self.image = None
        self.image_message = None
        self.objsToGrabTransformedSend = []
        self.iterations = 0

        #Messenger
        self.action = ""
        self.msg2Controller = ""
        self.awaitingResponse = True

        #Coordinates Msg
        self.msg = Twist()

        # Publishers
        self.pub = rospy.Publisher('/image_raw_but_better', Image,queue_size=10)
        self.pubM = rospy.Publisher('/camera2controller', String, queue_size=10)
        self.PubCoor = rospy.Publisher('/objCoordinates', Twist, queue_size=10)
        self.PubWC = rospy.Publisher('/objWidthC', Float64, queue_size=10)

        # Subscribers
        rospy.Subscriber("/image_raw",Image,self.callback)
        rospy.Subscriber("/controller2camera",String, self.callbackMsg)

        # Gaussian
        self.pWidth = 0
        
    def callbackMsg(self, data):
        self.action = data.data

    '''
    Mission Plan for picking up objects at unknown locations
    '''
    def switch(self, lang):
        if lang == "Are you ready?":
            self.msg2Controller = "Ready!"
        else:
            if (len(self.objsToGrabTransformedSend) < 1):
                self.msg2Controller = "No objects visible"

            elif lang == "Awaiting coordinates" and len(self.objsToGrabTransformedSend) > 0:
                self.printed = False
                px = self.objsToGrabTransformedSend[0][0]
                py = self.objsToGrabTransformedSend[0][1]
                pz = self.objsToGrabTransformedSend[0][2]
                oz = self.objsToGrabTransformedSend[0][4]
                self.pubCoordinates(px, py, pz, 0.0, 0.0, oz, 0.0)
                self.msg2Controller = "Coordinates are being published..."

            elif lang == "Good job gripper... We did it!":
                self.msg2Controller = "Likewise!"
            
        self.pubM.publish(self.msg2Controller)

    '''
    Publish Coordinates
    '''
    def pubCoordinates(self, px, py, pz, ox, oy, oz, ow):
        self.msg.linear.x = px
        self.msg.linear.y = py
        self.msg.linear.z = pz
        self.msg.angular.x = 0.0
        self.msg.angular.y = 0.0
        self.msg.angular.z = oz
        self.PubCoor.publish(self.msg)

    '''
    Publish Coordinates
    '''
    def pubWidth(self, width):
        
        self.PubWC.publish(Float64(width))

    '''
    Draws the orientation of the object
    '''
    def drawAxis(self, img, p_, q_, color, scale):
        p = list(p_)
        q = list(q_)
        
        ## [visualization1]
        angle = atan2(p[1] - q[1], p[0] - q[0]) # angle in radians
        hypotenuse = sqrt((p[1] - q[1]) * (p[1] - q[1]) + (p[0] - q[0]) * (p[0] - q[0]))
        
        # Here we lengthen the arrow by a factor of scale
        q[0] = p[0] - scale * hypotenuse * cos(angle)
        q[1] = p[1] - scale * hypotenuse * sin(angle)
        cv2.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), color, 3, cv2.LINE_AA)
        
        # create the arrow hooks
        p[0] = q[0] + 9 * cos(angle + pi / 4)
        p[1] = q[1] + 9 * sin(angle + pi / 4)
        cv2.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), color, 3, cv2.LINE_AA)
        
        p[0] = q[0] + 9 * cos(angle - pi / 4)
        p[1] = q[1] + 9 * sin(angle - pi / 4)
        cv2.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), color, 3, cv2.LINE_AA)
        ## [visualization1]

    '''
    Calculates the orientation of the objects
    '''
    def getOrientation(self, pts, img):
        # Construct a buffer used by the pca analysis
        sz = len(pts)
        data_pts = np.empty((sz, 2), dtype=np.float64)
        for i in range(data_pts.shape[0]):
            data_pts[i,0] = pts[i,0,0]
            data_pts[i,1] = pts[i,0,1]
        
        # Perform PCA analysis
        mean = np.empty((0))
        mean, eigenvectors, eigenvalues = cv2.PCACompute2(data_pts, mean)
        
        # Store the center of the object
        cntr = (int(mean[0,0]), int(mean[0,1]))
        ## [pca]
        
        ## [visualization]
        # Draw the principal components
        cv2.circle(img, cntr, 3, (255, 0, 255), 2)
        p1 = (cntr[0] + 0.02 * eigenvectors[0,0] * eigenvalues[0,0], cntr[1] + 0.02 * eigenvectors[0,1] * eigenvalues[0,0])
        p2 = (cntr[0] - 0.02 * eigenvectors[1,0] * eigenvalues[1,0], cntr[1] - 0.02 * eigenvectors[1,1] * eigenvalues[1,0])
        self.drawAxis(img, cntr, p1, (255, 255, 255), 1)
        self.drawAxis(img, cntr, p2, (255, 255, 255), 5)
        
        angle = atan2(eigenvectors[0,1], eigenvectors[0,0]) # orientation in radians
        ## [visualization]
        
        # Label with the rotation angle
        label = str(round(angle,2)) + " rad"
        textbox = cv2.rectangle(img, (cntr[0]+40, cntr[1]-25), (cntr[0] + 140, cntr[1] + 10), (255,255,255), -1)
        cv2.putText(img, label, (cntr[0]+40, cntr[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 1, cv2.LINE_AA)
        
        return angle

    '''
    Object detector that creates an array with the tranformed x and y coordinates (https://automaticaddison.com/how-to-determine-the-orientation-of-an-object-using-opencv/)
    '''
    def callback(self, msg):
        # Length of cropped image width: 1.448m (1230pixels) height: 0.192m (170)

        #Intialize
        objsToGrab = []
        objsToGrabTransformed = []
        width = []
        prevObjVisible = 0
        self.image = self.br.imgmsg_to_cv2(msg)
        oz = 0.0

        # Get image dimensions 1544, 2064
        (h, w) = self.image.shape[:2]
        (cX, cY) = (w // 2, h // 2)

        # Rotate Image by 1 degree
        M = cv2.getRotationMatrix2D((cX, cY), -1, 1.0)
        rotated = cv2.warpAffine(self.image, M, (w, h))

        # Add black rectangles everywhere except for the converyor belt
        cv2.rectangle(rotated, (0,0), (2064, 930), (0,0,0), -1)
        cv2.rectangle(rotated, (0,0), (550, 1544), (0,0,0), -1)
        cv2.rectangle(rotated, (0,1100), (2064, 1544), (0,0,0), -1)
        cv2.rectangle(rotated, (1780,0), (2064, 1544), (0,0,0), -1)

        # Gaussain Blur to remove noise
        blurred = cv2.GaussianBlur(rotated, (7, 7), 0)

        # Thresholding
        ret,imgt = cv2.threshold(blurred,30,255,cv2.THRESH_BINARY) 

        # Remove more noise using erosion and dialation
        kernel = np.ones((5, 5), np.uint8)
        img_erosion = cv2.erode(imgt, kernel, iterations=3)
        img_dilation = cv2.dilate(img_erosion, kernel, iterations=3)

        # Find contours of the found objects and grab them
        cnts = cv2.findContours(img_dilation, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours = cnts[0]

        # Use contours to filter relevant objects
        for i in range(len(contours)):
            M = cv2.moments(contours[i])    
            top_point = tuple(contours[i][contours[i][:,:,1].argmin()][0])
            bottom_point = tuple(contours[i][contours[i][:,:,1].argmax()][0])
            x,y,w,h = cv2.boundingRect(contours[i])
            #print(f"Object width: {round(w*(1448/1230),4)} height: {round(h*(192/170),4)}")
            if M['m00'] != 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                if cx > 2780 or cx < 600:
                    pass
                elif cv2.contourArea(contours[i]) > 7000 or cv2.contourArea(contours[i]) < 2000:
                    pass
                elif h > 150:
                    pass
                elif cx > 850 and cx < 1050 and cy < 970 and cy > 900:
                    pass
                else:
                    cv2.drawContours(self.image, [contours[i]], -1, (255, 255, 255), 2)
                    cv2.circle(self.image, (cx, cy), 7, (255, 255, 255), -1)
                    cv2.putText(self.image, str(i), (cx - 50, cy),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 2)
                    #print(f'obj#= {i} x= {"{:.2f}".format(cx)} y= {"{:.2f}".format(cy)}')

                    approx = cv2.approxPolyDP(contours[i],0.01*cv2.arcLength(contours[i],True),True)
                    #print(f"Object  {i} has a len(approx) of {len(approx)}.")
                    if len(approx) <= 10:
                        # Find the orientation of each shape
                        oz = self.getOrientation(contours[i], self.image) + 1.5707

                    objsToGrab.append((cx, cy, 0, i, oz))

        objsToGrab.sort(key=lambda x: x[0], reverse=True)
        #print(f"There are {len(objsToGrab)} objects available to grab.")
        objVisible = len(objsToGrab)
        prevObjVisible = objVisible

        #Transformation
        if objVisible == 0 or (objVisible != prevObjVisible):
            self.objsToGrabTransformedSend = []
            self.iterations = 0
        else:
            for i in range(len(objsToGrab)):
                xPixel = objsToGrab[i][0] - 550
                yPixel = objsToGrab[i][1] - 930

                #I am switching the x and y axis here
                xMeter = yPixel * (1.448/1230)
                yMeter = xPixel * (0.192/170)
                zMeter = objsToGrab[i][2]

                width = objsToGrab[i][4]*(1448/1230)
                oz = objsToGrab[i][4]
                #if not self.printed:
                    #print(f'obj#= {i} x= {"{:.2f}".format(xMeter)} y= {"{:.2f}".format(yMeter)} z= {"{:.2f}".format(zMeter)} oz= {"{:.2f}".format(oz)}')
                    #self.printed = True

                objsToGrabTransformed.append((xMeter, yMeter, zMeter, i, oz))

            if self.iterations>11:
               self.objsToGrabTransformedSend = objsToGrabTransformed
            #    self.pWidth = self.objsToGrabTransformedSend[0][4]

            self.iterations+=1
        
        self.image_message = self.br.cv2_to_imgmsg(self.image, encoding="passthrough")

    def start(self):
        rospy.loginfo("Timing images")
        while not rospy.is_shutdown():
            if self.image_message is not None:
                self.pub.publish(self.image_message)
                self.pubWidth(self.pWidth)
                

            self.switch(self.action)
            self.loop_rate.sleep()

if __name__ == '__main__':
    rospy.init_node("img2objPose", anonymous=True)
    my_node = Image2Object()
    my_node.start()