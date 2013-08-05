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
from sensor_msgs.msg import Image, CameraInfo
import threading

class Robodart_vision():
  lock =threading.Lock()
  ticker = False

  BGsample = 30 #number of frames to gather BG samples (average) from at start of capture
  circle_sample= 20 #number of Circles for calculate center
  #parameters for ObserveDartboard
  dp = 1 #Inverse ratio of the accumulator resolution to the image resolution. For example, if dp=1 , the accumulator has the same resolution as the input image. If dp=2 , the accumulator has half as big width and height.
  minDist = 10 #Minimum distance between the centers of the detected circles. If the parameter is too small, multiple neighbor circles may be falsely detected in addition to a true one. If it is too large, some circles may be missed.
  param1 = 40 #First method-specific parameter. In case of CV_HOUGH_GRADIENT , it is the higher threshold of the two passed to the Canny() edge detector (the lower one is twice smaller).
  param2 = 130 #Second method-specific parameter. In case of CV_HOUGH_GRADIENT , it is the accumulator threshold for the circle centers at the detection stage. The smaller it is, the more false circles may be detected. Circles, corresponding to the larger accumulator values, will be returned first.
  minRadius = 80 #Minimum circle radius.
  maxRadius = 190#Maximum circle radius.
  #parameters for calculate
  camera_width = 640
  camera_height = 480
  
  board_radius = 156 #in pixel
  board_radius_m = 0.5

  """
  This value is calculated by calling:
  rosservice call /robodart_vision/get_bullseye_center_offset
  
  Devide the 3rd value shown by robodart_vision node for the corresponding circle by
  the measured radius of the circle.
  """
  pixel_per_meter = 2925.029107981

  """
  If the gripper is positioned correctly above the bullseye.
  This value is returned by:
  rosservice call /robodart_vision/get_bullseye_center_offset
  """
  camera_dart_offset = [-0.0876930442617,0.0788041485691]

  def __init__(self):
    #cv2.namedWindow("Image window", 1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("UI122xLE_C/image_raw", Image, self.receive_image_from_camera_callback)
    self.cam_info  = rospy.Subscriber("UI122xLE_C/camera_info", CameraInfo, self.update_cam_info)
  

  def setTicker(self, value):
    self.lock.acquire()
    self.ticker = value
    self.lock.release()

  def receive_image_from_camera_callback(self, data):
    try:
      cv_image = self.bridge.imgmsg_to_cv(data, "bgr8")
      self.frame = cv_image
      showImage = cv.CreateMat(cv_image.rows / 4, cv_image.cols / 4, cv.CV_8UC3)
      cv.Resize( cv_image, showImage);
      cv.ShowImage("cam window", showImage)
      self.setTicker(True)
      cv.WaitKey(3)
    except CvBridgeError, e:
      print e
      
  def update_cam_info(self, data):
    self.width = data.width
    self.height = data.height

  def take_reference_picture(self, req):
    if self.ticker == True:      
      print "take_reference_picture()..."
      self.last_reference_picture = self.frame
      self.setTicker(False)
      
      #cv.ShowImage("Image", currentFrame)

      cv.SaveImage("refpic.png", self.last_reference_picture)

      self.showPicWithCircles(self.last_reference_picture)
    return []
    

  def get_bullseye_center_offset(self, req):

    print "get_bullseye_center_offset()..."
    if self.ticker == True:    
      currentFrame = self.frame
      circles = self.detect_circles(currentFrame)

      xMid = self.width / 2
      yMid = self.height / 2
      
      if len(circles) > 1:
        xAvg = 0
        yAvg = 0
        for c in circles:
          xAvg += c[0]
          yAvg += c[1]
        xAvg = xAvg / len(circles)
        yAvg = yAvg / len(circles)

      else :
        xAvg = circles[0][0]
        yAvg = circles[0][1]

      xDif = xMid - xAvg
      yDif = yMid - yAvg
      
      self.setTicker(False)
    return [-xDif/self.pixel_per_meter + self.camera_dart_offset[0], yDif/self.pixel_per_meter + self.camera_dart_offset[1]]

  def get_dart_center_offset(self, req):
    print "get_dart_center_offset()..."
    return [0.1, 0.2]

  def detect_circles(self, image, draw=True):
    image = np.asarray(image)
    frameGrey = cv2.cvtColor(image, cv2.COLOR_RGBA2GRAY)
    frameBlur = cv2.GaussianBlur(frameGrey, (0,0), 2)
    circles = cv2.HoughCircles(frameBlur, cv2.cv.CV_HOUGH_GRADIENT, self.dp, self.minDist, np.array([]), self.param1, self.param2, self.minRadius, self.maxRadius)
    if circles is not None:
      for c in circles[0]:
        print c

    if draw == True:
      for c in circles[0]:
        cv2.circle(image, (c[0],c[1]),c[2], (0,255,0),2)
        cv2.circle(image,(c[0],c[1]),1, (0,255,0),2)
      
      image = cv.fromarray(image)
      cv.SaveImage("CircleImage.png", image)
      small = cv.CreateMat(image.rows / 4, image.cols / 4, cv.CV_8UC3)    
      cv.Resize( image, small);
      cv.ShowImage("Circles", small)
        

    return circles[0]

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
        print c
    showImage = cv.fromarray(currentFrame)
    cv.SaveImage("CircleImage.png", showImage)
    small = cv.CreateMat(showImage.rows / 4, showImage.cols / 4, cv.CV_8UC3)    
    cv.Resize( showImage, small);
    cv.ShowImage(windowName, small)


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
  
