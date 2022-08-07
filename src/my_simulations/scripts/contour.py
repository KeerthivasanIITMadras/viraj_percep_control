#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import math
import sensor_msgs.point_cloud2 as pcl2
from sensor_msgs.msg import PointCloud2
import std_msgs.msg

bridge = CvBridge()

pub1 = rospy.Publisher("/contour_right", Image, queue_size=10)
pub2 = rospy.Publisher("/contour_left", Image, queue_size=10)
pcl_pub = rospy.Publisher("/my_pcl_topic", PointCloud2)


def image_callback1(msg):
    try:
        cv2_img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")    #converting ros image message to numpy array
        cv2_img = cv2.rotate(cv2_img, cv2.ROTATE_180)   #inverting the image, here i am doing to for better pointcloud image(i got it from trial and error)
        print("Received image")
    except CvBridgeError:
        pass
    #performing contour detection
    img_gray = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(img_gray, 240, 255, cv2.THRESH_BINARY)
    contours, hierarchy = cv2.findContours(
        image=thresh, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)
    cv2.drawContours(image=cv2_img, contours=contours, contourIdx=-1,
                     color=(0, 255, 0), thickness=2, lineType=cv2.LINE_AA)
    image_message = bridge.cv2_to_imgmsg(cv2_img, encoding="bgr8")
    pub1.publish(image_message)
    K = np.array([[277, 0, 160], [
        0, 277, 120], [0, 0, 1]])
    cp = math.cos(0.45)
    sp = math.sin(0.45)
    n = np.array([[0], [0], [1]])   #world coordinate z axis
    R = np.array([[1, 0, 0], [0, cp, sp], [0, -sp, cp]])    #;itch rotation matrix 
    nc = np.transpose(R.dot(n))
    Kinv = np.linalg.inv(K)
    camera_xyz = []
    # applying the transformation to the uv pixels obtained from the contour variable and converting to camera coordinates
    for cnt in contours:
        approx = cv2.approxPolyDP(cnt, 0.009 * cv2.arcLength(cnt, True), True)
        n = approx.ravel()
        i = 0

        for j in n:
            if(i % 2 == 0):
                x = n[i]
                y = n[i + 1]
                uv = np.array([[y], [x], [1]])
                mul1 = Kinv.dot(uv)
                mul2 = nc.dot(mul1)
                cp = math.cos(-0.45)
                sp = math.sin(-0.45)
                R = np.array([[1, 0, 0], [0, cp, sp], [0, -sp, cp]])
                t = R.dot(1.18*mul1/mul2)
                camera_xyz.append([t[0], t[1], t[2]])
            i = i + 1

    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'camera_link' 
    # create pcl from points
    scaled_polygon_pcl = pcl2.create_cloud_xyz32(header, camera_xyz)
    # publish
    pcl_pub.publish(scaled_polygon_pcl)

#this callback function for getting the left image of the stereo camera and performing contour detection
def image_callback2(msg):
    try:
        cv2_img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        print("Received image")
    except CvBridgeError:
        pass
    img_gray = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(img_gray, 240, 255, cv2.THRESH_BINARY)
    contours, hierarchy = cv2.findContours(
        image=thresh, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)
    cv2.drawContours(image=cv2_img, contours=contours, contourIdx=-1,
                     color=(0, 255, 0), thickness=2, lineType=cv2.LINE_AA)
    image_message = bridge.cv2_to_imgmsg(cv2_img, encoding="bgr8")
    pub2.publish(image_message)


def talker():
    rospy.init_node('contour', anonymous=True)
    rospy.Subscriber("/stereo_camera/right/image_raw", Image, image_callback1)
    rospy.Subscriber("/stereo_camera/left/image_raw", Image, image_callback2)
    rospy.spin()


if __name__ == '__main__':
    talker()
