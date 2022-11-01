#!/usr/bin/env python
import py_compile
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String
from geometry_msgs.msg import Twist 
import cv2
import numpy as np

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

        # Subscribers
        rospy.Subscriber("/image_raw",Image,self.callback)
        rospy.Subscriber("/controller2camera",String, self.callbackMsg)
        
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
                px = self.objsToGrabTransformedSend[0][0]
                py = self.objsToGrabTransformedSend[0][1]
                pz = self.objsToGrabTransformedSend[0][2]
                self.pubCoordinates(px, py, pz, 0.0, 0.0, 1.0, 0.0)
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
        self.PubCoor.publish(self.msg)

    '''
    Object detector that creates an array with the tranformed x and y coordinates
    '''
    def callback(self, msg):
        # Length of cropped image width: 1.448m (1230pixels) height: 0.192m (170)

        #Intialize
        objsToGrab = []
        objsToGrabTransformed = []
        prevObjVisible = 0
        self.image = self.br.imgmsg_to_cv2(msg)

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
        img_erosion = cv2.erode(imgt, kernel, iterations=5)
        img_dilation = cv2.dilate(img_erosion, kernel, iterations=6)

        # Find contours of the found objects and grab them
        cnts = cv2.findContours(img_dilation, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours = cnts[0]

        # Use contours to filter relevant objects
        for i in range(len(contours)):
            M = cv2.moments(contours[i])    
            top_point = tuple(contours[i][contours[i][:,:,1].argmin()][0])
            bottom_point = tuple(contours[i][contours[i][:,:,1].argmax()][0])
            objHeight = bottom_point[1] - top_point[1]
            if M['m00'] != 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                if cx > 2780 or cx < 600:
                    print("The object is too far.")
                elif cv2.contourArea(contours[i]) > 7000 or cv2.contourArea(contours[i]) < 2000:
                    pass
                elif objHeight > 150:
                    pass
                elif cx > 850 and cx < 1050 and cy < 970 and cy > 900:
                    pass
                else:
                    cv2.drawContours(self.image, [contours[i]], -1, (255, 255, 255), 2)
                    cv2.circle(self.image, (cx, cy), 7, (255, 255, 255), -1)
                    cv2.putText(self.image, str(i), (cx - 50, cy),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 2)
                    #print(f'obj#= {i} x= {"{:.2f}".format(cx)} y= {"{:.2f}".format(cy)}')
                    objsToGrab.append((cx, cy, 0, i))

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
                if not self.printed:
                    print(f'obj#= {i} x= {"{:.2f}".format(xMeter)} y= {"{:.2f}".format(yMeter)} z= {"{:.2f}".format(zMeter)}')

                objsToGrabTransformed.append((xMeter, yMeter, zMeter, i))

            if self.iterations>10:
                self.objsToGrabTransformedSend = objsToGrabTransformed

            self.iterations+=1
        
        self.image_message = self.br.cv2_to_imgmsg(self.image, encoding="passthrough")
        self.printed = True

    def start(self):
        rospy.loginfo("Timing images")
        while not rospy.is_shutdown():
            if self.image_message is not None:
                self.pub.publish(self.image_message)

            self.switch(self.action)
            self.loop_rate.sleep()

if __name__ == '__main__':
    rospy.init_node("img2objPose", anonymous=True)
    my_node = Image2Object()
    my_node.start()