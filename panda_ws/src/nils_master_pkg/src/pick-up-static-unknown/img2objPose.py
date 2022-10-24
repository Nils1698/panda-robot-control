#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import numpy as np

class Image2Object(object):
    def __init__(self):
        # Params
        self.image = None
        self.image_message = None
        self.br = CvBridge()
        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(10)

        # Publishers
        self.pub = rospy.Publisher('/image_raw_but_better', Image,queue_size=10)

        # Subscribers
        rospy.Subscriber("/image_raw",Image,self.callback)
        

    def callback(self, msg):
        self.image = self.br.imgmsg_to_cv2(msg)
        # Length of cropped image width: 1.448m height: 0.192m
        # Cropping an image

        # Get image dimensions
        (h, w) = self.image.shape[:2]
        (cX, cY) = (w // 2, h // 2)

        # Rotate Image by 1 degree
        M = cv2.getRotationMatrix2D((cX, cY), -1, 1.0)
        rotated = cv2.warpAffine(self.image, M, (w, h))

        # Crop image
        cropped_image = rotated[930:1100, 550:1780]

        # Gaussain Blur to remove noise
        blurred = cv2.GaussianBlur(cropped_image, (7, 7), 0)

        # Thresholding
        ret,imgt = cv2.threshold(blurred,30,255,cv2.THRESH_BINARY)

        # Remove more noise using erosion and dialation
        kernel = np.ones((5, 5), np.uint8)
        img_erosion = cv2.erode(imgt, kernel, iterations=4)
        img_dilation = cv2.dilate(img_erosion, kernel, iterations=4)

        # Find contours of the found objects and grab them
        contours, hierarchy = cv2.findContours(img_dilation, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 1:
            print(f"There are {len(contours)} objects visible.\nTake some objects of the conveyor to start.")
        elif len(contours) == 0:
            print(f"Put an object on the table to start.")
        elif cv2.contourArea(contours[0]) > 7000:
            print(f"The object is too big.")
        else:
            for i in range(len(contours)):
                M = cv2.moments(contours[i])
                if M['m00'] != 0:
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                    if cx > 1000 or cx < 50:
                        print("The object is too far.")
                    else:
                        cv2.drawContours(cropped_image, [contours[i]], -1, (255, 255, 255), 2)
                        cv2.circle(cropped_image, (cx, cy), 7, (255, 255, 255), -1)
                        cv2.putText(cropped_image, str(i), (cx - 10, cy),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                        print(f"x: {cx} y: {cy}")
        

        #cv2.drawContours(cropped_image, contours, -1, (255, 255, 255), 3)

        self.image = cropped_image

        self.image_message = self.br.cv2_to_imgmsg(self.image, encoding="passthrough")

    def start(self):
        rospy.loginfo("Timing images")
        while not rospy.is_shutdown():
            if self.image_message is not None:
                self.pub.publish(self.image_message)
            self.loop_rate.sleep()

if __name__ == '__main__':
    rospy.init_node("img2objPose", anonymous=True)
    my_node = Image2Object()
    my_node.start()