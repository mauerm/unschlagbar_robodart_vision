#! /usr/bin/env python

PACKAGE='robodart'
import roslib
#roslib.load_manifest(PACKAGE)
import rospy
import numpy as np
import sys
import cv2
import cv
from std_srvs.srv import Empty
from robodart_vision.srv import Point
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class Robodart_vision():
  def __init__(self):
    #cv2.namedWindow("Image window", 1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("UI122xLE_C/image_raw", Image, self.receive_image_from_camera_callback)
    

  def receive_image_from_camera_callback(self, data):
    try:
      cv_image = self.bridge.imgmsg_to_cv(data, "bgr8")
      self.frame = cv_image
      cv.ShowImage("cam window", cv_image)
      cv.WaitKey(3)
    except CvBridgeError, e:
      print e
      

  def take_reference_picture(self, req):
    self.last_reference_picture = self.frame
    
    #cv.ShowImage("Image", currentFrame)

    self.showPicWithCircles(self.last_reference_picture)
    return []
    

  def get_bullseye_center_offset(self, req):
    print "get_bullseye_center_offset()..."
    return [0.1, 0.2]

  def get_dart_center_offset(self, req):
    print "get_dart_center_offset()..."
    return [0.1, 0.2]

  def showPicWithCircles(self, pic, windowName="Circles"):
    #parameters for ObserveDartboard
    dp = 1           
    minDist = 100       
    param1 = 40       
    param2 = 130      
    minRadius = 5    #Minimum circle radius.
    maxRadius = 1000  #Maximum circle radius.


    currentFrame = np.asarray(pic)
    frameGrey = cv2.cvtColor(currentFrame, cv2.COLOR_RGBA2GRAY)
    frameBlur = cv2.GaussianBlur(frameGrey, (0,0), 2)
    circles =  cv2.HoughCircles(frameBlur, cv2.cv.CV_HOUGH_GRADIENT, dp, minDist, np.array([]), param1, param2, minRadius, maxRadius)  
   
    if circles is not None:
      for c in circles[0]:
        cv2.circle(currentFrame, (c[0],c[1]),c[2], (0,255,0),2)
        cv2.circle(currentFrame,(c[0],c[1]),1, (0,255,0),2)
    showImage = cv.fromarray(currentFrame)
    
    cv.ShowImage(windowName, showImage)


if __name__ == '__main__':

  rospy.init_node('robodart_vision_node')

  try:
    my_robodart_vision = Robodart_vision()

  except rospy.ROSInterruptException: pass

  ref_pic  = rospy.Service('robodart_vision/take_reference_picture', Empty, my_robodart_vision.take_reference_picture)
  bullseye = rospy.Service('robodart_vision/get_bullseye_center_offset', Point, my_robodart_vision.get_bullseye_center_offset)
  dart     = rospy.Service('robodart_vision/get_dart_center_offset', Point, my_robodart_vision.get_dart_center_offset)
  
  print "All Services ready"

  

  rospy.spin()
  
