#! /usr/bin/env python
from docutils.math.latex2mathml import math

PACKAGE='robodart_vision'
import roslib
#roslib.load_manifest(PACKAGE)
import rospy
import numpy as np
import sys
import cv2
import cv
from std_srvs.srv import Empty
from robodart_vision.srv import Point, SetOffset
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
import threading
import os
import time
import datetime


class Robodart_vision():
   
  ''' ==================================================================== '''
  ''' Dynamic Parameters. These may have to be adjusted at the RTL Studios '''
  ''' ==================================================================== '''



  """
  This value is calculated by calling:
  rosservice call /robodart_vision/get_bullseye_center_offset
  
  Devide the 3rd value shown by robodart_vision node for the corresponding circle by
  the measured radius of the circle.
  The last Value from about 10 Meter was: 2983.668489245
  
  Outer radius
  """
  circle_radius = [0, 0.225, 0.185, 0.1625, 0.1405, 0.1185, 0.097, 0.075, 0.053, 0.03055]
  pixel_per_meter_10m = (1180.505 + 1181.329 + 1177.209) / 3
  pixel_per_meter_11m = (1076.34 + 1085.169)/2

  pixel_per_meter_generalprobe = (32 / circle_radius[9] + 57 / circle_radius[8] + 77 / circle_radius[7] + 103 / circle_radius[6] + 124 / circle_radius[5] +
                                  149 / circle_radius[4] + 171 / circle_radius[3] + 196 / circle_radius[2] + 237 / circle_radius[1]) / 9
  print pixel_per_meter_generalprobe
  
  pixel_per_meter = pixel_per_meter_11m
  #pixel_per_meter = 3543.216

  #3meter test 15.11.13
  pixel_per_meter = 645.96

  """
  This value is calculated by get_dart_center_offset and is set by robodart_control
  """
  #10m offset [0.309408827795,-0.0363646297808]
  camera_dart_offset = [0.0, 0.0]


  
  ''' Treshhold for the Bitmapimage from the Div image during the Arrow detectin '''
  ''' If nothing is seen, you have to lower this value, if there is to much noise, you have to increas this value '''
  #threshold_value = 60
  threshold_value = 30
    

  ''' Treshhold for the Bitmapimage from the Div image during the Arrow detectin '''
  ''' If nothing is seen, you have to lower this value, if there is to much noise, you have to increas this value '''
  erode_treshold = 100
  

  ''' The Publishrate for the Live picture in picture Image. '''
  ''' It means that you publishes every n-th frame. Use it if this function takes to long '''
  ''' The publish funktion is not Performance optimal '''
  PublishRate = 1


  ''' The size of the Live Image showen by the Node itself'''
  ''' HINT: FOR THE PUBLISHED STREAM TAKE THE VALUES StreamSize and StreamPiPSize below '''
  LiveCaptureSize = [1400, 880]

  ''' This is the total size of the Published stream '''
  StreamSize      = [640, 480]

  ''' This is the size of the smaller live Image in the upper left corner of the live stream'''
  ''' Hint: Don't make this value to high becouse it costs a lot of performance with the current implementation '''
  StreamPiPSize   = [160, 120]
  
  ''' StreamTicker: Shows the current status of the Stream 

  Valid values are:
    0 - The originall image takes the whole Stream
    1 - The originall image is showen in the upper left corner, an image of the Resent event is showen in the middle
  It can be set by setStreamStatus(value). This function is Thread Save
  '''
  streamTicker = 1

  ''' ==================================================================== '''
  ''' ====================== END DYNAMIC PARAMETERS ====================== '''
  ''' ==================================================================== '''

  lock =threading.Lock()
  ticker = False


  lock2 = threading.Lock()
  eventImage = None
  eventType = cv.CV_8UC3
  counter = 0  

  dartboard_radius_meter = 0.23 #0.23 # Radius der Scheibe in Meter
  dartboard_radius_pixel = 0 # Radius der Scheibe in Pixel, wird spaeter aus pixel_per_meter berechnet

  last_reference_picture = None
  
  package_dir = None


  def __init__(self):
    #cv2.namedWindow("Image window", 1)
    
    
    timestamp = str(datetime.datetime.now())
    self.package_dir = roslib.packages.get_pkg_dir(PACKAGE) + '/temp_images/'+ timestamp + '_'
    print "Package dir = ", self.package_dir
    
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("UI122xLE_C/image_raw", Image, self.receive_image_from_camera_callback)
    self.cam_info  = rospy.Subscriber("UI122xLE_C/camera_info", CameraInfo, self.update_cam_info)
    self.stream   = rospy.Publisher('robodart_vision/small_video_stream', Image)
    self.dartboard_radius_pixel = self.pixel_per_meter * self.dartboard_radius_meter
    self.set_circle_parameter_default()
    
    
  

  ''' ============================ '''
  ''' Sets the Ticker, THREADSAVE! '''
  ''' ============================ '''
  def setTicker(self, value):
    self.lock.acquire()
    self.ticker = value
    self.lock.release()


  ''' ================================== '''
  ''' Sets the StreamStatus, THREADSAVE! '''
  ''' Valid values are:                  '''
  ''' 0 - The originall image takes the  '''
  '''     whole Stream                   '''
  ''' 1 - The originall image is showen  '''
  '''     in the upper left corner, an   '''
  '''     image of the Resent event is   '''
  '''     showen in the middle           '''
  ''' ================================== '''
  def setStreamStatus(self, value):
    self.lock2.acquire()
    self.streamTicker = value
    self.lock2.release()


  ''' ================================= '''
  ''' Callback for Raw_Image_Node (ros) '''
  ''' ================================= '''
  def receive_image_from_camera_callback(self, data):
    try:
      cv_image = self.bridge.imgmsg_to_cv(data, "bgr8")
      
      self.frame = cv_image
      # Publish own Stream
      self.publish_stream()
      self.setTicker(True)
    except CvBridgeError, e:
      print e
      
 
  ''' ========================== '''
  ''' Publishes the video Stream '''
  ''' ========================== '''
  def publish_stream(self):
    self.counter = self.counter + 1
    if self.counter != self.PublishRate:
      return
    self.counter = 0
    try:      
      streamImage = cv.CreateMat(self.StreamSize[1], self.StreamSize[0], self.eventType)
      if self.streamTicker == 1 and self.eventImage is not None:
        cv.Resize(self.eventImage, streamImage)


        smallImage = cv.CreateMat(self.StreamPiPSize[1], self.StreamPiPSize[0], cv.CV_8UC3)
        cv.Resize(self.frame, smallImage)

        newImage = np.asarray(streamImage)
        newImage2 = np.asarray(smallImage)

        newImage[0:self.StreamPiPSize[1], 0:self.StreamPiPSize[0]] = newImage2[0:self.StreamPiPSize[1], 0:self.StreamPiPSize[0]]

        streamImage = cv.fromarray(newImage)

      else:
        cv.Resize( self.frame, streamImage) 
        cv.Circle(streamImage,(streamImage.cols / 2,streamImage.rows / 2),2, (0,0, 255),2) #red
      
      streamImage = self.bridge.cv_to_imgmsg(streamImage, "bgr8")
      self.stream.publish(streamImage)
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
    print "take_reference_picture()..."
    
    #cv.ShowImage("Image", currentFrame)
    self.last_reference_picture = cv.CreateMat(self.frame.rows, self.frame.cols, cv.CV_8UC3)
    cv.Resize(self.frame, self.last_reference_picture)
    cv.SaveImage(self.package_dir + "refpic.png", self.last_reference_picture)

    #self.showPicWithCircles(self.last_reference_picture)
    return []
    

  ''' ================================================================== '''
  ''' Calculates the Offset from the Bullseye to the Center of the Image '''
  ''' ================================================================== '''
  def get_bullseye_center_offset(self, req):
    self.set_circle_parameter_default()
    timestamp = str(datetime.datetime.now())
    self.package_dir = roslib.packages.get_pkg_dir(PACKAGE) + '/temp_images/'+ timestamp + '_'

    print "get_bullseye_center_offset()..."
  
    currentFrame = self.frame
    circles = self.detect_circles(currentFrame)

    xMid = self.frame.cols / 2
    yMid = self.frame.rows / 2
    
    print "Half x and y", xMid, yMid
    avg = self.getAverageCircleMiddle(circles)
    
    numpy_currentFrame = np.asarray(currentFrame)
    
    cv2.circle(numpy_currentFrame,(int(avg[0]),int(avg[1])),10, (255,255, 255),10)
    cv2.imwrite(self.package_dir + "dartboard_with_detected_center.png", numpy_currentFrame)
    
    print "xyMId", xMid, yMid

    xDiff = avg[0] -xMid
    yDiff = avg[1] -yMid
    
    print "xyDiffIn pixel", xDiff, yDiff
    
    xDiffInMeter = xDiff / self.pixel_per_meter
    yDiffInMeter = yDiff / self.pixel_per_meter
    
    xDiffInRobotFrame = xDiffInMeter
    yDiffInRobotFrame = -yDiffInMeter
    
    print "bullseye center offser: xyDiffInRobotFrame", xDiffInRobotFrame, yDiffInRobotFrame
    
    print "bullseye center offset: camera_dart_offset" ,self.camera_dart_offset


    #TODO: check
    return [xDiffInRobotFrame, yDiffInRobotFrame]


  ''' ============================================================== '''
  ''' Calculates the Offset from the Dart to the Center of the Image '''
  ''' ============================================================== '''
  def get_dart_center_offset(self, req):


    print "get_dart_center_offset()"
    
    timestamp = str(datetime.datetime.now())
    self.package_dir = roslib.packages.get_pkg_dir(PACKAGE) + '/temp_images/'+ timestamp + '_'

    if self.last_reference_picture is None:
      raise Exception("The function take_reference_picture was not called")

    dartboard = np.asarray(self.last_reference_picture)

    dartboard_with_arrow = np.asarray(self.frame)
    cv2.imwrite(self.package_dir + "dartboard_with_arrow.png", dartboard_with_arrow) 
    
    #extract circle from the dartboard
    circles = self.detect_circles(dartboard_with_arrow, False)
    detected_middle = self.getAverageCircleMiddle(circles)

    

    # Convert to GreyScale:
    dartboard = cv2.cvtColor(dartboard, cv2.COLOR_RGBA2GRAY)
    dartboard_with_arrow_colored = dartboard_with_arrow
    dartboard_with_arrow = cv2.cvtColor(dartboard_with_arrow, cv2.COLOR_RGBA2GRAY)


    cv2.imwrite(self.package_dir + "dartboard_full_grey.png", dartboard) 
    #TODO: somewhere around here make a circular mask to get rid of the reflections

    length = int(self.dartboard_radius_pixel * 2)
    
    print 'length1', length
    
    xStart = detected_middle[0] - self.dartboard_radius_pixel
    yStart = detected_middle[1] - self.dartboard_radius_pixel

    xEnd = xStart+length
    yEnd = yStart+length
    
    print 'length2', length
    
    '''TODO check this and the block below if its needed, 
    check why template is not created probably if dartboard is at the side of camera view'''
    #Avoid Error if image is too small to cut out the whole template
    
    xlength = length
    ylength = length
    
    if xStart < 0:
      xlength = xlength + xStart
      xStart = 0
    if yStart < 0:
      ylength = ylength + yStart
      yStart = 0
    
    # Filter negativ Values and set them to zero
    xStart = max(xStart, 0)
    yStart = max(yStart, 0)

    print 'Infos...'
    print 'xStart: ', xStart
    print 'xEnd: ', xEnd
    print '_______________'
    print 'yStart: ', yStart
    print 'yEnd: ', yEnd

    template = dartboard_with_arrow[yStart:yEnd, xStart:xEnd]

    cv2.imwrite(self.package_dir + "template.png", template)
    
    
    # match template
    try:
      result = cv2.matchTemplate(dartboard,template,cv2.TM_SQDIFF)
    except:
      print "Unexpected error: see template.png and dartboard_with_arrow.png", sys.exc_info()[0]
      return [0,0]

    (min_x,max_y,minloc,maxloc)=cv2.minMaxLoc(result)
    (min_loc_x, min_loc_y) = minloc
    
    xEndLoc = min_loc_x+xlength
    yEndLoc = min_loc_y+ylength
    
    print 'ylength', ylength
    print 'xlength', xlength
    
    #Avoid Error if image is too small to cut out the whole template
    '''
    if xEnd > len(dartboard_with_arrow[0]):
      xEnd = len(dartboard_with_arrow[0])-1
    if yEnd > len(dartboard_with_arrow):
      yEnd = len(dartboard_with_arrow)-1
    '''
    cut_from_dartboard = dartboard[min_loc_y:yEndLoc, min_loc_x:xEndLoc]
    cv2.imwrite(self.package_dir + "cut_from_dartboard.png", cut_from_dartboard) 
    

    div = cv2.absdiff(template, cut_from_dartboard)
    cv2.imwrite(self.package_dir + "div.png", div)

    #threshold(src, threshold, pixel_color_if_above_threshold, thresholdType)
    (retval, binary_threshold) = cv2.threshold(div, self.threshold_value, 255, cv2.THRESH_BINARY)  

    #binary_pic = cv.fromarray(binary_threshold[1])
    #cv.SaveImage(self.package_dir + "div_threshold.jpg", binary_threshold)

    cv2.imwrite(self.package_dir + "binary_threshold.png", binary_threshold)


    #element = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3,3))
    #eroded = cv2.erode(binary_threshold, element)

    #cv2.imwrite(self.package_dir + "eroded.png", eroded)

    #dilated = cv2.dilate(eroded, element)

    #cv2.imwrite(self.package_dir + "dilated.png", dilated)

    std_dev_limit = 10
    erode_size = 2
    erode_limit = 7
    
    element = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(erode_size,erode_size))
    eroded = cv2.erode(binary_threshold, element)
    
  
    
    while erode_size <= erode_limit:
      erode_size += 1
      (y_non_zero_array,x_non_zero_array) = np.nonzero(eroded > 0)
      
      #here
      std_dev_x = x_non_zero_array.std()
      std_dev_y = y_non_zero_array.std()
      
      print "Std_dev_x, Std_dev_y", std_dev_x, std_dev_y

      
      if std_dev_x < std_dev_limit and std_dev_y < std_dev_limit:
        print "Dart detected"
        break
        
      else:
        element = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(erode_size,erode_size))
        eroded = cv2.erode(binary_threshold, element)
  
        cv2.imwrite(self.package_dir + "eroded_" + str(erode_size) + ".png", eroded)
        
        rospy.loginfo("Number of detected Pixels for erod=" + str(erode_size) + ": " + str(len(x_non_zero_array)))
        
    if erode_size == erode_limit:
      print "Didn't find the dart after " + str(erode_limit) + " erods, abording!"
      return [0,0] 
    '''  
    
    if len(x_non_zero_array) > self.erode_treshold:
    else:
    '''
    #0,0 is in the left top corner of the image, min_loc_y are rows, min_loc_x are cols
    '''
      rospy.loginfo("Number of detected Pixels: " + str(len(x_non_zero_array)))
      
      
      

    '''
    if len(x_non_zero_array) < 1:
      print 'No Dart detected, has a Dart been dropped?'
      return [0,0] 
      
      
    y_median = np.median(y_non_zero_array)
    x_median = np.median(x_non_zero_array)
    
    print 'x_median' , x_median

    xPos = x_median + xStart
    yPos = y_median + yStart
    
    cv2.circle(dartboard_with_arrow_colored,(int(xPos),int(yPos)),4, (0,255, 0),4) #white
    
    cv2.line(dartboard_with_arrow_colored,(int(xPos),int(yPos)),(len(dartboard_with_arrow_colored[0])/2,len(dartboard_with_arrow_colored)/2),(0,255,0),2)
    
    cv2.circle(dartboard_with_arrow_colored,(len(dartboard_with_arrow_colored[0])/2,len(dartboard_with_arrow_colored)/2),3, (255,0, 0),2) #red

    print 'x_median + min_loc_x' , x_median
    
    cv2.circle(dartboard_with_arrow_colored,(int(detected_middle[0]),int(detected_middle[1])),2, (0, 0, 255),2) #blue
    
    cv2.imwrite(self.package_dir + "dartboard_with_detected_arrow.png", dartboard_with_arrow_colored)
    
    image = dartboard_with_arrow_colored
    image = cv.fromarray(image)
    self.eventImage = image  
    self.eventType = cv.CV_8UC3
    
    
    xOffset = xPos - len(dartboard_with_arrow[0])/2
    yOffset = yPos - len(dartboard_with_arrow)/2
    
    
    
    print 'Pixel per meter' , self.pixel_per_meter
    print 'xOffset' , xOffset
    print 'yOffset' , yOffset
    
    
    #save bis hier
    
    xOffsetMeter = float(xOffset) / float(self.pixel_per_meter)
    yOffsetMeter = float(yOffset) / float(self.pixel_per_meter)
  
    #set image for stream
    #self.event_image = binary_threshold
    
    #Conversion from image coordinate system to robot coordinate system
    OffsetInRobotFrame = [0,0]
    OffsetInRobotFrame[0] = xOffsetMeter
    OffsetInRobotFrame[1] = -yOffsetMeter
    
    print 'XYOffsetInRObotFrame' , OffsetInRobotFrame
    
    return OffsetInRobotFrame



  def set_camera_dart_offset(self, data):
    #TODO: change name
    self.camera_dart_offset[0] = data.x
    self.camera_dart_offset[1] = data.y
    print "Set offset to: ",self.camera_dart_offset
    return []


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
      cv.SaveImage(self.package_dir + "CircleImage.png", image)
      self.eventImage = image  
      self.eventType = cv.CV_8UC3    
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

    
    #if circles is not None:
      #for c in circles[0]:
       # print c
    showImage = cv.fromarray(currentFrame)
    cv.SaveImage(self.package_dir + "CircleImage.png", showImage)
    small = cv.CreateMat(showImage.rows / 4, showImage.cols / 4, cv.CV_8UC3)    
    cv.Resize( showImage, small);
    cv.ShowImage(windowName, small)

  ''' Sets circle detection Parameters '''  
  def set_circle_parameter_default(self):
    self.BGsample = 30 #number of frames to gather BG samples (average) from at start of capture
    self.circle_sample= 50 #number of Circles for calculate center
    #parameters for ObserveDartboard
    self.dp = 1 #Inverse ratio of the accumulator resolution to the image resolution. For example, if dp=1 , the accumulator has the same resolution as the input image. If dp=2 , the accumulator has half as big width and height.
    self.minDist = 2 #Minimum distance between the centers of the detected circles. If the parameter is too small, multiple neighbor circles may be falsely detected in addition to a true one. If it is too large, some circles may be missed.
    self.param1 = 20 #First method-specific parameter. In case of CV_HOUGH_GRADIENT , it is the higher threshold of the two passed to the Canny() edge detector (the lower one is twice smaller).

    self.param2 = 150 #Second method-specific parameter. In case of CV_HOUGH_GRADIENT , it is the accumulator threshold for the circle centers at the detection stage. The smaller it is, the more false circles may be detected. Circles, corresponding to the larger accumulator values, will be returned first.
    self.minRadius = 10 #Minimum circle radius.
    self.maxRadius = 900#Maximum circle radius.


if __name__ == '__main__':
  rospy.init_node('robodart_vision_node')
  
  try:
    my_robodart_vision = Robodart_vision()

  except rospy.ROSInterruptException: 
    rospy.logwarn("ROSInterruptException at Vision startup")

  ref_pic  = rospy.Service('robodart_vision/take_reference_picture', Empty, my_robodart_vision.take_reference_picture)
  bullseye = rospy.Service('robodart_vision/get_bullseye_center_offset', Point, my_robodart_vision.get_bullseye_center_offset)
  dart     = rospy.Service('robodart_vision/get_dart_center_offset', Point, my_robodart_vision.get_dart_center_offset)
  camOffset = rospy.Service('robodart_vision/set_camera_dart_offset', SetOffset, my_robodart_vision.set_camera_dart_offset)
  

  print "All Services ready"
  
  rospy.spin()
