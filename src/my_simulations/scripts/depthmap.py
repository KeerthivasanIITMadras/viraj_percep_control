#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()
pub = rospy.Publisher('/depth', Image, queue_size=10)


def callback1(msg):
    global flag1
    global imgL
    imgL = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    imgL = cv2.cvtColor(imgL, cv2.COLOR_BGR2GRAY)
    flag1 = True


def callback2(msg):
    global flag2
    global imgR
    imgR = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    imgR = cv2.cvtColor(imgR, cv2.COLOR_BGR2GRAY)
    flag2 = True


def talker():
    global imgL
    global imgR
    global flag1
    global flag2
    flag1 = False
    flag2 = False
    rospy.init_node('depthmap', anonymous=True)
    rospy.Subscriber('contour_right', Image, callback1)
    rospy.Subscriber('contour_left', Image, callback2)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if flag1 and flag2:
            stereo = cv2.StereoBM_create(numDisparities=16,
                                        blockSize=15)
            disparity = stereo.compute(imgL, imgR)
            depth_img = bridge.cv2_to_imgmsg(disparity, encoding="16SC1")
            pub.publish(depth_img)
            rate.sleep()


if __name__ == '__main__':
    talker()
