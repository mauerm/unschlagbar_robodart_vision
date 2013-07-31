#!/usr/bin/env python
import roslib
import thread
import cv
import cv2
#import cv2.cv as cv
import numpy as np
from copy import copy
import threading
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

#roslib.load_manifest('robodart_control')
import time
class ObserveDartboard(threading.Thread):

  BGsample = 30 # number of frames to gather BG samples (average) from at start of capture
  
   
  def __init__(self, camSource):
    threading.Thread.__init__(self)    
#    self.capture = cv2.VideoCapture(0)
#    self.ret, self.frame = self.capture.read()
#    width = self.capture.get(3)
#    height = self.capture.get(4)
#    print 'size = ' + str(width) + ' x ' + str(height)  
    self.bridge = CvBridge()
    self.frame = 0

  def receive_image_from_camera_callback(self, data):
    try:
      cv_image = self.bridge.imgmsg_to_cv(data, "bgr8")
      self.frame = cv_image
     
    except CvBridgeError, e:
      print e
      
#    cv.ShowImage("Image window", cv_image)
#    cv.WaitKey(3)
    print time.time()
    
      

    
        
  def get_bullseye_center_offset(self):
    
    return 0.01, 0.012
  
  def take_reference_picture(self):
    cv.ShowImage("Image window", self.frame)
    return True

  def get_dart_center_offset(self):
    return 0.01, 0.015
             
  def run(self):
    while True:
      cv2.waitKey(10000)
      my_robodart_vision.take_reference_picture()
    '''
    self.getAverage()
    self.background = copy(self.average)
    self.diffFrame = copy(self.average)
    while True:
      self.ret, self.frame = self.capture.read()
      self.getCircles()
      if self.circles is not None:
        for c in self.circles[0]:
            # draw the circle outline
            cv2.circle(self.frame, (c[0],c[1]),c[2], (0,255,0),2)
            # draw the circle center
            cv2.circle(self.frame,(c[0],c[1]),1, (0,255,0),2)
      cv2.imshow("camera", self.frame)
      
      self.getAverage()
      cv2.imshow("average (20 frames)", self.average)
      
      cv2.absdiff(self.background, self.average, self.diffFrame)
      cv2.imshow("diff", self.diffFrame)
      
      cv2.imshow("background", self.background)  
      self.k = cv2.waitKey(33)  
      '''



  def getAverage(self):
    
    avg1 = np.float32(self.frame) # 32 bit accumulator
    for i in range(20): self.ret, self.frame = self.capture.read() # dummy to warm up sensor
    for i in range(self.BGsample):
      ret, frame = self.capture.read()
      cv2.accumulateWeighted(frame,avg1,0.5)
      res = cv2.convertScaleAbs(avg1)
    
    self.average = cv2.cvtColor(res, cv2.COLOR_RGBA2GRAY)
 
  def getCircles(self):
    #parameters for ObserveDartboard
    dp = 1            #Inverse ratio of the accumulator resolution to the image resolution. For example, if dp=1 , the accumulator has the same resolution as the input image. If dp=2 , the accumulator has half as big width and height.
    minDist = 100    #Minimum distance between the centers of the detected circles. If the parameter is too small, multiple neighbor circles may be falsely detected in addition to a true one. If it is too large, some circles may be missed.
    param1 = 40       #First method-specific parameter. In case of CV_HOUGH_GRADIENT , it is the higher threshold of the two passed to the Canny() edge detector (the lower one is twice smaller).
    param2 = 130      #Second method-specific parameter. In case of CV_HOUGH_GRADIENT , it is the accumulator threshold for the circle centers at the detection stage. The smaller it is, the more false circles may be detected. Circles, corresponding to the larger accumulator values, will be returned first.
    minRadius = 5    #Minimum circle radius.
    maxRadius = 1000  #Maximum circle radius.
  
    self.frameGrey = cv2.cvtColor(self.frame, cv2.COLOR_RGBA2GRAY)
    self.frameBlur = cv2.GaussianBlur(self.frameGrey, (0,0), 2)
    self.circles =  cv2.HoughCircles(self.frameBlur, cv2.cv.CV_HOUGH_GRADIENT, dp, minDist, np.array([]), param1, param2, minRadius, maxRadius)

if __name__ == '__main__':
#    ic = image_converter()
  rospy.init_node('robodart_vision', anonymous=True)  

  cv.NamedWindow("Image window", 1)
  my_robodart_vision = ObserveDartboard(0)
  image_sub = rospy.Subscriber("UI122xLE_C/image_raw", Image, my_robodart_vision.receive_image_from_camera_callback)

  my_robodart_vision.start()  

  try:
      rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv.DestroyAllWindows()

