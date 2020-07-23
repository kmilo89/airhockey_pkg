#!/usr/bin/env python3

import cv2
import numpy as np
import rospy
import time
from PIL import Image
import io
from sensor_msgs.msg import CompressedImage


def callbackCam(msg):
    img = msg.data
    nparr = np.fromstring(img, np.uint8)
    img1 = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
    cv2.imshow('frame', img1)
    cv2.waitKey(1)
    #rospy.loginfo(img1)
    
def main():
    rospy.init_node('send_pos_disk_node', anonymous=False)
    rospy.loginfo("Initialization of colors detect node")
    rospy.Subscriber('/usb_cam/image_raw/compressed', CompressedImage, callbackCam)
    rospy.spin()

if __name__ == "__main__":
    main()






""" # Convert to ROS message.
        bridge = CvBridge()
        msg = bridge.cv2_to_imgmsg(img)

    #### Create CompressedIamge ####
           msg = CompressedImage()
           msg.header.stamp = rospy.Time.now()
           msg.format = "jpeg"
           msg.data = np.array(cv2.imencode('.jpg', image_np)[1]).tostring()
           # Publish new image
           self.image_pub.publish(msg)
"""
