#!/usr/bin/env python
# coding=utf8
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
import fnmatch

''' ============================================================== '''
''' Calculates the Offset from the Dart to the Center of the Image '''
''' ============================================================== '''
def get_dart_center_offset(req):
  pixel_per_meter = (1076.34 + 1085.169)/2
  dartboard_radius_pixel =  pixel_per_meter * 0.23
  threshold_value = 60
  
  dartboard_with_arrow_array = []
  dartboard_without_arrow_array = []
  
  for file in os.listdir(roslib.packages.get_pkg_dir(PACKAGE) + '/test_images/'):
    if fnmatch.fnmatch(file, '*_dartboard_with_arrow*' ):
        dartboard_with_arrow_array.append(file)
    if fnmatch.fnmatch(file, '*_dartboard_full_grey*' ):   
        dartboard_without_arrow_array.append(file)
    print file
        
  for image_number in range(len(dartboard_with_arrow_array)):
    
    print "get_dart_center_offset()"
     
    timestamp = str(datetime.datetime.now())
    package_dir = roslib.packages.get_pkg_dir(PACKAGE) + '/test_images_output/'+ timestamp + '_'
    
  
    dartboard = cv2.imread(roslib.packages.get_pkg_dir(PACKAGE) + '/test_images/' + dartboard_without_arrow_array[image_number]) 
      
    dartboard_with_arrow = cv2.imread(roslib.packages.get_pkg_dir(PACKAGE) + '/test_images/' + dartboard_with_arrow_array[image_number])

    ''' Ab hier --- '''  
    #extract circle from the dartboard
    circles = detect_circles(dartboard, False)
    detected_middle = getAverageCircleMiddle(circles)
  
      
  
    # Convert to GreyScale:
    dartboard = cv2.cvtColor(dartboard, cv2.COLOR_RGBA2GRAY)
    dartboard_with_arrow = cv2.cvtColor(dartboard_with_arrow, cv2.COLOR_RGBA2GRAY)
  
  
    cv2.imwrite(package_dir + "dartboard_full_grey.png", dartboard) 
    #TODO: somewhere around here make a circular mask to get rid of the reflections
  
    length = int(dartboard_radius_pixel * 2)
      
    print 'length1', length
      
    xStart = detected_middle[0] - dartboard_radius_pixel
    yStart = detected_middle[1] - dartboard_radius_pixel
  
    xEnd = xStart+length
    yEnd = yStart+length
      
    print 'length2', length
      
    '''TODO check this and the block below if its needed, 
    check why template is not created probably if dartboard is at the side of camera view'''
    #Avoid Error if image is too small to cut out the whole template
    '''
    if xStart > len(dartboard[0]):
      xStart = len(dartboard[0])-1
    if yStart > len(dartboard):
      yStart = len(dartboard)-1
    '''
  
    # Filter negativ Values and set them to zero
    xStart = max(xStart, 0)
    yStart = max(yStart, 0)
  
    template = dartboard[yStart:yEnd, xStart:xEnd]
  
    cv2.imwrite(package_dir + "template.png", template)
    cv2.imwrite(package_dir + "dartboard_with_arrow.png", dartboard_with_arrow) 
      
    # match template
    try:
      result = cv2.matchTemplate(dartboard_with_arrow,template,cv2.TM_SQDIFF)
    except:
      print "Unexpected error: see template.png and dartboard_with_arrow.png", sys.exc_info()[0]
      return [0,0]
  
    (min_x,max_y,minloc,maxloc)=cv2.minMaxLoc(result)
    (min_loc_x, min_loc_y) = minloc
     
    xEndLoc = min_loc_x+length
    yEndLoc = min_loc_y+length
      
    print 'length3', length
      
    #Avoid Error if image is too small to cut out the whole template
    '''
    if xEnd > len(dartboard_with_arrow[0]):
      xEnd = len(dartboard_with_arrow[0])-1
    if yEnd > len(dartboard_with_arrow):
      yEnd = len(dartboard_with_arrow)-1
    '''
    cut_from_dartboard = dartboard_with_arrow[min_loc_y:yEndLoc, min_loc_x:xEndLoc]
    cv2.imwrite(package_dir + "cut_from_dartboard.png", cut_from_dartboard) 
      
  
    div = cv2.absdiff(template, cut_from_dartboard)
    cv2.imwrite(package_dir + "div.png", div)
  
    #threshold(src, threshold, pixel_color_if_above_threshold, thresholdType)
    (retval, binary_threshold) = cv2.threshold(div, threshold_value, 255, cv2.THRESH_BINARY)  
  
    #binary_pic = cv.fromarray(binary_threshold[1])
    #cv.SaveImage(self.package_dir + "div_threshold.jpg", binary_threshold)
  
    cv2.imwrite(package_dir + "binary_threshold.png", binary_threshold)
  
  
    #element = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3,3))
    #eroded = cv2.erode(binary_threshold, element)
  
    #cv2.imwrite(self.package_dir + "eroded.png", eroded)
  
    #dilated = cv2.dilate(eroded, element)
  
    #cv2.imwrite(self.package_dir + "dilated.png", dilated)
  
    (y_non_zero_array,x_non_zero_array) = np.nonzero(binary_threshold > 0)
      
    if len(x_non_zero_array) > 300:
      element = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(2,2))
      eroded = cv2.erode(binary_threshold, element)
  
      cv2.imwrite(package_dir + "eroded.png", eroded)
        
      (y_non_zero_array,x_non_zero_array) = np.nonzero(eroded > 0)
      rospy.loginfo("Number of detected Pixels: " + str(len(x_non_zero_array)))
    else:
      '''
      0,0 is in the left top corner of the image, min_loc_y are rows, min_loc_x are cols
      '''
      rospy.loginfo("Number of detected Pixels: " + str(len(x_non_zero_array)))
        
        
      if len(x_non_zero_array) < 17:
        print 'No Dart detected, has a Dart been dropped?'
        return [0,0]
  
        
    y_median = np.median(y_non_zero_array)
    x_median = np.median(x_non_zero_array)
      
    print 'x_median' , x_median
  
    xPos = x_median + min_loc_x
    yPos = y_median + min_loc_y
      
    cv2.circle(dartboard_with_arrow,(int(xPos),int(yPos)),10, (255,255, 255),10) #white
  
    print 'x_median + min_loc_x' , x_median
      
    cv2.circle(dartboard_with_arrow,(int(detected_middle[0]),int(detected_middle[1])),10, (255,0, 0),10) #red
      
    cv2.imwrite(package_dir + "dartboard_with_detected_arrow.png", dartboard_with_arrow)
    '''
    image = dartboard_with_arrow
      
    image = cv.fromarray(image)
    #cv.SaveImage(self.package_dir + "CircleImage.png", image)
    self.eventImage = image  
    self.eventType = cv.CV_8UC1    
    small = cv.CreateMat(image.rows / 4, image.cols / 4, cv.CV_8UC1)
    print "image:", image
    print "small:", small    
    cv.Resize( image, small);
    '''
      
      
    xOffset = xPos - detected_middle[0]
    yOffset = yPos - detected_middle[1]
      
   
      
    #print 'Pixel per meter' , .pixel_per_meter
    print 'xOffset' , xOffset
    print 'yOffset' , yOffset
      
      
    xOffsetMeter = float(xOffset) / float(pixel_per_meter)
    yOffsetMeter = float(yOffset) / float(pixel_per_meter)
    
    #set image for stream
    #self.event_image = binary_threshold
      
    #Conversion from image coordinate system to robot coordinate system
    OffsetInRobotFrame = [0,0]
    OffsetInRobotFrame[0] = xOffsetMeter
    OffsetInRobotFrame[1] = -yOffsetMeter
      
    print 'XYOffsetInRObotFrame' , OffsetInRobotFrame
     
    return OffsetInRobotFrame
  
''' ========================================== '''
''' Returns an Array with all Detected Circles '''
''' ========================================== '''
def detect_circles(image, draw=True):
  BGsample = 30 #number of frames to gather BG samples (average) from at start of capture
  circle_sample= 50 #number of Circles for calculate center
  #parameters for ObserveDartboard
  dp = 1 #Inverse ratio of the accumulator resolution to the image resolution. For example, if dp=1 , the accumulator has the same resolution as the input image. If dp=2 , the accumulator has half as big width and height.
  minDist = 2 #Minimum distance between the centers of the detected circles. If the parameter is too small, multiple neighbor circles may be falsely detected in addition to a true one. If it is too large, some circles may be missed.
  param1 = 40 #First method-specific parameter. In case of CV_HOUGH_GRADIENT , it is the higher threshold of the two passed to the Canny() edge detector (the lower one is twice smaller).

  param2 = 300 #Second method-specific parameter. In case of CV_HOUGH_GRADIENT , it is the accumulator threshold for the circle centers at the detection stage. The smaller it is, the more false circles may be detected. Circles, corresponding to the larger accumulator values, will be returned first.
  minRadius = 80 #Minimum circle radius.
  maxRadius = 700#Maximum circle radius.
  
  frameGrey = cv2.cvtColor(image, cv2.COLOR_RGBA2GRAY)
  frameBlur = cv2.GaussianBlur(frameGrey, (0,0), 2)
  circles = cv2.HoughCircles(frameBlur, cv2.cv.CV_HOUGH_GRADIENT, 
                             dp, minDist, np.array([]), param1, 
                             param2, minRadius, maxRadius)
  if circles is None:
    raise Exception("I didn't find any Circles in the Image.")

  for c in circles[0]:
    print c

  if draw == True:
    for c in circles[0]:
      cv2.circle(image, (c[0],c[1]),c[2], (0,255,0),2)
      cv2.circle(image,(c[0],c[1]),1, (0,255,0),2)

    
    image = cv.fromarray(image)
    cv.SaveImage(package_dir + "CircleImage.png", image)
    #self.eventImage = image  
    #self.eventType = cv.CV_8UC3    
    small = cv.CreateMat(image.rows / 4, image.cols / 4, cv.CV_8UC3)    
    cv.Resize( image, small);
    
    
    
    #cv.ShowImage("Circles", small)

  return circles[0]


''' ========================================================== '''
''' Returns an Array (aka Point) with the Average CircleMiddle '''
''' ========================================================== '''
def getAverageCircleMiddle(circles):
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


  
if __name__ == '__main__':
  get_dart_center_offset(None)