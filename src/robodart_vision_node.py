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
import os


class Robodart_vision():
  lock =threading.Lock()
  ticker = False

  BGsample = 30 #number of frames to gather BG samples (average) from at start of capture
  circle_sample= 20 #number of Circles for calculate center
  #parameters for ObserveDartboard
  dp = 1 #Inverse ratio of the accumulator resolution to the image resolution. For example, if dp=1 , the accumulator has the same resolution as the input image. If dp=2 , the accumulator has half as big width and height.
  minDist = 10 #Minimum distance between the centers of the detected circles. If the parameter is too small, multiple neighbor circles may be falsely detected in addition to a true one. If it is too large, some circles may be missed.
  param1 = 40 #First method-specific parameter. In case of CV_HOUGH_GRADIENT , it is the higher threshold of the two passed to the Canny() edge detector (the lower one is twice smaller).
  param2 = 300 #Second method-specific parameter. In case of CV_HOUGH_GRADIENT , it is the accumulator threshold for the circle centers at the detection stage. The smaller it is, the more false circles may be detected. Circles, corresponding to the larger accumulator values, will be returned first.
  minRadius = 80 #Minimum circle radius.
  maxRadius = 700#Maximum circle radius.
  #parameters for calculate

  threshold_value = 110
  
  board_radius = 156 #in pixel
  board_radius_m = 0.5

  LiveCaptureSize = [1400, 880]

  """
  This value is calculated by calling:
  rosservice call /robodart_vision/get_bullseye_center_offset
  
  Devide the 3rd value shown by robodart_vision node for the corresponding circle by
  the measured radius of the circle.
  """
  pixel_per_meter = 2983.668489245

  """
  If the gripper is positioned correctly above the bullseye.
  This value is returned by:
  rosservice call /robodart_vision/get_bullseye_center_offset
  """
  camera_dart_offset = [0.309408827795,-0.0363646297808]

  dartboard_radius_meter = 0.23 # Radius der Scheibe in Meter
  dartboard_radius_pixel = 642 # Radius der Scheibe in Pixel, wird spaeter aus pixel_per_meter berechnet


  def __init__(self):
    #cv2.namedWindow("Image window", 1)
    cv2.namedWindow("test", 1 )
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("UI122xLE_C/image_raw", Image, self.receive_image_from_camera_callback)
    self.cam_info  = rospy.Subscriber("UI122xLE_C/camera_info", CameraInfo, self.update_cam_info)
    
  
  ''' ============================ '''
  ''' Sets the Ticker, THREADSAVE! '''
  ''' ============================ '''
  def setTicker(self, value):
    self.lock.acquire()
    self.ticker = value
    self.lock.release()

  ''' ================================= '''
  ''' Callback for Raw_Image_Node (ros) '''
  ''' ================================= '''
  def receive_image_from_camera_callback(self, data):
    try:
      cv_image = self.bridge.imgmsg_to_cv(data, "bgr8")
      self.frame = cv_image
      showImage = cv.CreateMat(self.LiveCaptureSize[1], self.LiveCaptureSize[0], cv.CV_8UC3)
      cv.Resize( cv_image, showImage);
      cv.ShowImage("cam window", showImage)
      self.setTicker(True)
      cv.WaitKey(3)
    except CvBridgeError, e:
      print e
      
  ''' =============================== '''
  ''' Callback for CamInfo_Node (ros) '''
  ''' =============================== '''
  def update_cam_info(self, data):
    self.width = data.width
    self.height = data.height

  ''' ========================================== '''
  ''' Saves the Currentframe as ReferencePicture '''
  ''' ========================================== '''
  def take_reference_picture(self, req):
    if self.ticker == True:      
      print "take_reference_picture()..."
      self.last_reference_picture = self.frame
      self.setTicker(False)
      
      #cv.ShowImage("Image", currentFrame)

      cv.SaveImage("refpic.png", self.last_reference_picture)

      self.showPicWithCircles(self.last_reference_picture)
    return []
    
  ''' ================================================================== '''
  ''' Calculates the Offset from the Bullseye to the Center of the Image '''
  ''' ================================================================== '''
  def get_bullseye_center_offset(self, req):

    print "get_bullseye_center_offset()..."
    if self.ticker == True:    
      currentFrame = self.frame
      circles = self.detect_circles(currentFrame)

      xMid = self.width / 2
      yMid = self.height / 2
      
      avg = self.getAverageCircleMiddle(circles)

      xDif = xMid - avg[0]
      yDif = yMid - avg[1]
      
      self.setTicker(False)
    return [-xDif/self.pixel_per_meter - self.camera_dart_offset[0], yDif/self.pixel_per_meter - self.camera_dart_offset[1]]

  ''' ============================================================== '''
  ''' Calculates the Offset from the Dart to the Center of the Image '''
  ''' ============================================================== '''
  def get_dart_center_offset(self, req):
    

    #xMid = self.width / 2
    #yMid = self.height / 2

    xMid = 1280
    yMid = 960

  
    print "get_dart_center_offset()..."
    #Test -------------------- 
    #currentFrame = self.frame
    currentFrame = cv.LoadImageM("refpic.jpg")
    circles = self.detect_circles(currentFrame, False)
    currentFrame = np.asarray(currentFrame)
    currentFrame = cv2.cvtColor(currentFrame, cv2.COLOR_RGBA2GRAY)
    
    currentFrame2 = cv.LoadImageM("refpic1.jpg")    
    circles2 = self.detect_circles(currentFrame2, False)     
    currentFrame2 = np.asarray(currentFrame2)
    currentFrame2 = cv2.cvtColor(currentFrame2, cv2.COLOR_RGBA2GRAY)

    currentFrame = cv.fromarray(currentFrame)
    currentFrame2 = cv.fromarray(currentFrame2)
    
    
    length = 2 * self.dartboard_radius_pixel
    lengthY1 = length
    lengthX1 = length
    lengthY2 = length
    lengthX2 = length

    #Get Subimage1
    avg = self.getAverageCircleMiddle(circles)
    
    xStart = avg[0] - self.dartboard_radius_pixel
    yStart = avg[1] - self.dartboard_radius_pixel

    if (xStart + lengthX1 > currentFrame.cols):
      lengthX1 = currentFrame.cols - xStart

    if (yStart + lengthY1 > currentFrame.rows):
      lengthY1 = currentFrame.rows - yStart

    print "--------------------------------------------------------"
    print "Start at: ", xStart, "x", yStart
    print "Length is: ", lengthX1, "x", lengthY1
    print "So end is at: ", xStart+lengthX1, "x", yStart+lengthY1
    print "My res is: ", currentFrame.cols, "x", currentFrame.rows
    print "--------------------------------------------------------"


    subframe = cv.GetSubRect(currentFrame, (int(xStart), int(yStart), int(lengthX1), int(lengthY1)))
    

    #Get Subimage2
    avg2 = self.getAverageCircleMiddle(circles2)
    
    xStart2 = avg2[0] - self.dartboard_radius_pixel
    yStart2 = avg2[1] - self.dartboard_radius_pixel
    
    if (xStart2 + lengthX2 > currentFrame2.cols):
      lengthX2 = currentFrame2.cols - xStart2

    if (yStart2 + lengthY2 > currentFrame2.rows):
      lengthY2 = currentFrame2.rows - yStart2

    print "--------------------------------------------------------"
    print "Start at: ", xStart2, "x", yStart2
    print "Length is: ", lengthX2, "x", lengthY2
    print "So end is at: ", xStart2+lengthX2, "x", yStart2+lengthY2
    print "My res is: ", currentFrame2.cols, "x", currentFrame2.rows
    print "--------------------------------------------------------"
    
    subframe2 = cv.GetSubRect(currentFrame2, (int(xStart2), int(yStart2), int(lengthX2), int(lengthY2)))   


    if subframe.cols != subframe2.cols or subframe.rows != subframe2.rows:
      minCols = min(subframe.cols, subframe2.cols)
      minRows = min(subframe.rows, subframe2.rows)

      subframeTemp = cv.CreateMat(minRows, minCols, cv.CV_8UC1)
      subframe2Temp = cv.CreateMat(minRows, minCols, cv.CV_8UC1)

      cv.Resize( subframe, subframeTemp);
      cv.Resize( subframe2, subframe2Temp);
    
      subframe = subframeTemp
      subframe2 = subframe2Temp

    cv.SaveImage("sub1.png", subframe)
    cv.SaveImage("sub2.png", subframe2)
    

    #Get Dif Image
    div = cv.CreateMat(subframe.rows, subframe.cols, cv.CV_8UC1)
    cv.AbsDiff(subframe, subframe2, div)
    
    '''
    div = cv.CreateMat(currentFrame.rows, currentFrame.cols, cv.CV_8UC1)
    cv.AbsDiff(currentFrame, currentFrame2, div)
    '''
    
    cv.SaveImage("div1.jpg", div)

    div = np.asarray(div)    

    #cv2.threshold(currentFrame, bitmap, self.threshold_value, 255, 3)
    bitmap = cv2.threshold(div, self.threshold_value, 255, cv2.THRESH_BINARY)

        
    bitmapPic = cv.fromarray(bitmap[1])
    cv.SaveImage("div_threshold.jpg", bitmapPic)     


    counter = 0
    sumX = 0
    sumY = 0    

    for col in range(bitmapPic.cols):
      for row in range(bitmapPic.rows):
        if bitmapPic[row, col] != 0.0:        
          #print bitmapPic[row,col], '  at: ', row, 'x', col
          counter = counter + 1
          sumX = sumX + col
          sumY = sumY + row

    if counter == 0:
      raise Exception("The images are exactly the same after the Trashhold! Maybe adjust Trashhold")

    xPos = sumX / counter
    yPos = sumY / counter

    xPos = xPos + xStart
    yPos = yPos + yStart

    print 'arrow at: ', xPos, 'x', yPos

    xOffset = xMid - xPos
    yOffset = yMid - yPos
  
    xOffsetMeter = xOffset / self.pixel_per_meter
    yOffsetMeter = yOffset / self.pixel_per_meter
  
    #print bitmapPic.getData()



    #cv2.waitKey(20)
    
    #end test---------------------'''
    return [xOffsetMeter - self.camera_dart_offset[0], yOffsetMeter - self.camera_dart_offset[1]]

  ''' ========================================== '''
  ''' Returns an Array with all Detected Circles '''
  ''' ========================================== '''
  def detect_circles(self, image, draw=True):
    image = np.asarray(image)
    frameGrey = cv2.cvtColor(image, cv2.COLOR_RGBA2GRAY)
    frameBlur = cv2.GaussianBlur(frameGrey, (0,0), 2)
    circles = cv2.HoughCircles(frameBlur, cv2.cv.CV_HOUGH_GRADIENT, 
                               self.dp, self.minDist, np.array([]), self.param1, 
                               self.param2, self.minRadius, self.maxRadius)
    if circles is None:
      raise Exception("I didn't find any Circles in the Image.")

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

      #cv.ShowImage("Circles", small)

    return circles[0]


  ''' ========================================================== '''
  ''' Returns an Array (aka Point) with the Average CircleMiddle '''
  ''' ========================================================== '''
  def getAverageCircleMiddle(self, circles):
    if (circles == None):
      return None

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

    return [xAvg, yAvg]

  ''' =============================================== '''
  ''' Displayes the Picture with all detected Circles '''
  ''' OBSOLETED!!! USE detect_circles instead         '''
  ''' =============================================== '''
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
