#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image, CameraInfo
import cv2
import cv2.cv as cv
import cv_bridge
from cv_bridge import CvBridge, CvBridgeError


def talker():
    capture = cv2.VideoCapture(1)
    ret, frame = capture.read()
    bridge = CvBridge()
    pub = rospy.Publisher('UI122xLE_C/image_raw', Image)

    rospy.init_node('UI122xLE_C_Emulator')
    while not rospy.is_shutdown():
        ret, frame = capture.read()
        frame = cv.fromarray(frame)      
   
        image_message = bridge.cv_to_imgmsg(frame, encoding="bgr8")
        pub.publish(image_message)
        #rospy.sleep(1.0)


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

