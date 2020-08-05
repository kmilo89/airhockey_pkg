#!/usr/bin/env python3

import cv2
import numpy as np
import rospy
import time
from PIL import Image
import io
from sensor_msgs.msg import CompressedImage, Image
from airhockey_pkg.msg import camera
from cv_bridge import CvBridge

def draw(mask, color, image, _area):
    found_object = False
    x = 0
    y = 0
    area_object = 0
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for c in contours:
        area = cv2.contourArea(c)
        if area > _area:
            found_object = True
            m = cv2.moments(c)
            if m["m00"] == 0:
                m["m00"] = 1
            x = int(m["m10"]/m["m00"])
            y = int(m["m01"]/m["m00"])
            cv2.circle(image, (x, y), 7, color, -1)
            font = cv2.FONT_HERSHEY_SIMPLEX
            new_contourn = cv2.convexHull(c)
            cv2.drawContours(image, [new_contourn], 0, color, 3)
            cv2.putText(image, 'x:{}, y:{}'.format(x, y), (x+10, y), font,  0.4, (0, 0, 0), 1, cv2.LINE_AA)
            area_object = area
    return image, x, y, found_object, area_object

def colors_detect(img):
    blue = (255, 0, 0)
    red = (0, 0, 255)
    new_img.height, new_img.width = img.shape[:2]
    new_img.header.stamp = rospy.Time.now()
    data.found_disk = False
    data.found_puck = False
    redLow1 = np.array([0, 100, 20], np.uint8)
    redHigh1 = np.array([10, 255, 255], np.uint8)
    redLow2 = np.array([170, 100, 20], np.uint8)
    redHigh2 = np.array([179, 255, 255], np.uint8)
    blueLow = np.array([100, 100, 20], np.uint8)
    blueHigh = np.array([125, 255, 255], np.uint8)
    frameHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    maskRed1 = cv2.inRange(frameHSV, redLow1, redHigh1)
    maskRed2 = cv2.inRange(frameHSV, redLow2, redHigh2)
    maskRed = cv2.add(maskRed1, maskRed2)
    maskBlue = cv2.inRange(frameHSV, blueLow, blueHigh)
    img, data.pos_puck.x, data.pos_puck.y, data.found_puck, data.area_puck = draw(maskBlue, blue, img, 4000)
    img, data.pos_disk.x, data.pos_disk.y, data.found_disk, data.area_disk = draw(maskRed, red, img, 4000)
    #new_img.data = cv2.imencode('.jpg', img)[1].tostring()
    new_img.data = bridge. cv2_to_imgmsg(img, "bgr8")
    pub_img.publish(new_img)
    pub.publish(data)
    cv2.imshow('frame', img)
    cv2.waitKey(1)

def callbackCam(msg):
    nparr = np.frombuffer(msg.data, np.uint8)
    img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
    colors_detect(img)
    
def main():
    global pub
    global pub_img
    global data
    global new_img
    global bridge
    bridge = CvBridge()
    rospy.init_node('colors_detect_node', anonymous=False)
    rospy.loginfo("Initialization of colors detect node")
    rospy.Subscriber('/usb_cam/image_raw/compressed', CompressedImage, callbackCam)
    pub = rospy.Publisher('puck_disk_pos', camera, queue_size=1)
    pub_img = rospy.Publisher('puck_disk_img', Image, queue_size=1)
    data = camera()
    new_img = Image()
    rospy.spin()

if __name__ == "__main__":
    main()